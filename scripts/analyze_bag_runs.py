#!/usr/bin/env python3
"""Analyze paired marker/velocity ROS bag runs and generate plots."""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path("bags_plots/.matplotlib").resolve()))
os.environ.setdefault("XDG_CACHE_HOME", str(Path("bags_plots/.cache").resolve()))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
from rosbag.bag import ROSBagException, ROSBagFormatException, ROSBagUnindexedException


RUN_RE = re.compile(r"^(markers|vel)_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})\.bag$")


def values(df: pd.DataFrame, column: str) -> np.ndarray:
    return df[column].to_numpy()


def find_runs(root: Path) -> list[tuple[str, Path, Path]]:
    marker_bags: dict[str, Path] = {}
    vel_bags: dict[str, Path] = {}

    for bag_path in root.rglob("*.bag"):
        match = RUN_RE.match(bag_path.name)
        if not match:
            continue
        kind, stamp = match.groups()
        if kind == "markers":
            marker_bags[stamp] = bag_path
        elif kind == "vel":
            vel_bags[stamp] = bag_path

    stamps = sorted(set(marker_bags) & set(vel_bags))
    return [(stamp, marker_bags[stamp], vel_bags[stamp]) for stamp in stamps]


def read_velocity_bag(path: Path, topic: str = "/high_cmd") -> pd.DataFrame:
    rows = []
    with rosbag.Bag(str(path), allow_unindexed=True) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            rows.append(
                {
                    "time": t.to_sec(),
                    "vx_cmd": float(msg.velocity[0]),
                    "vy_cmd": float(msg.velocity[1]),
                    "yaw_rate_cmd": float(msg.yawSpeed),
                    "mode": int(msg.mode),
                    "gait_type": int(msg.gaitType),
                    "speed_level": int(msg.speedLevel),
                    "foot_raise_height": float(msg.footRaiseHeight),
                    "body_height": float(msg.bodyHeight),
                }
            )

    df = pd.DataFrame(rows)
    if df.empty:
        return df
    df["t_rel"] = df["time"] - df["time"].iloc[0]
    df["speed_cmd"] = np.hypot(df["vx_cmd"], df["vy_cmd"])
    return df


def read_marker_bag(path: Path, topic: str = "/detections_markers") -> tuple[pd.DataFrame, pd.DataFrame]:
    marker_rows = []
    summary_rows = []

    with rosbag.Bag(str(path), allow_unindexed=True) as bag:
        first_time = None
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = t.to_sec()
            if first_time is None:
                first_time = stamp
            t_rel = stamp - first_time
            distances = []

            for marker in msg.markers:
                x = float(marker.pose.position.x)
                y = float(marker.pose.position.y)
                z = float(marker.pose.position.z)
                dist_xy = math.hypot(x, y)
                distances.append(dist_xy)
                marker_rows.append(
                    {
                        "time": stamp,
                        "t_rel": t_rel,
                        "marker_id": int(marker.id),
                        "ns": marker.ns,
                        "x": x,
                        "y": y,
                        "z": z,
                        "dist_xy_from_origin": dist_xy,
                        "scale_x": float(marker.scale.x),
                        "scale_y": float(marker.scale.y),
                        "scale_z": float(marker.scale.z),
                    }
                )

            summary_rows.append(
                {
                    "time": stamp,
                    "t_rel": t_rel,
                    "obstacle_count": len(msg.markers),
                    "nearest_obstacle_dist": min(distances) if distances else np.nan,
                    "mean_obstacle_dist": float(np.mean(distances)) if distances else np.nan,
                }
            )

    return pd.DataFrame(marker_rows), pd.DataFrame(summary_rows)


def merge_timeseries(vel_df: pd.DataFrame, marker_summary_df: pd.DataFrame) -> pd.DataFrame:
    if vel_df.empty:
        return marker_summary_df.copy()
    if marker_summary_df.empty:
        return vel_df.copy()

    left = vel_df.sort_values("time")
    right = marker_summary_df.sort_values("time")
    return pd.merge_asof(left, right, on="time", suffixes=("", "_markers"), direction="nearest")


def dead_reckon_commands(vel_df: pd.DataFrame) -> pd.DataFrame:
    if vel_df.empty:
        return pd.DataFrame(columns=["t_rel", "x_cmd_int", "y_cmd_int", "theta_cmd_int"])

    x = 0.0
    y = 0.0
    theta = 0.0
    rows = []
    prev_t = float(vel_df["t_rel"].iloc[0])

    for row in vel_df.itertuples(index=False):
        t_rel = float(row.t_rel)
        dt = max(0.0, t_rel - prev_t)
        vx = float(row.vx_cmd)
        yaw_rate = float(row.yaw_rate_cmd)

        x += dt * vx * math.cos(theta)
        y += dt * vx * math.sin(theta)
        theta += dt * yaw_rate
        prev_t = t_rel
        rows.append({"t_rel": t_rel, "x_cmd_int": x, "y_cmd_int": y, "theta_cmd_int": theta})

    return pd.DataFrame(rows)


def build_obstacle_tracks(marker_df: pd.DataFrame) -> pd.DataFrame:
    if marker_df.empty:
        return pd.DataFrame(
            columns=[
                "time",
                "t_rel",
                "marker_id",
                "x",
                "y",
                "z",
                "dt",
                "dx",
                "dy",
                "speed_xy",
                "heading_xy",
            ]
        )

    tracks = marker_df.sort_values(["marker_id", "time"]).copy()
    grouped = tracks.groupby("marker_id", sort=False)
    tracks["dt"] = grouped["time"].diff()
    tracks["dx"] = grouped["x"].diff()
    tracks["dy"] = grouped["y"].diff()
    tracks["speed_xy"] = np.hypot(tracks["dx"], tracks["dy"]) / tracks["dt"].replace(0.0, np.nan)
    tracks["heading_xy"] = np.arctan2(tracks["dy"], tracks["dx"])
    return tracks


def save_obstacle_tracks_xy(marker_df: pd.DataFrame, tracks_df: pd.DataFrame, path_df: pd.DataFrame, out_dir: Path) -> None:
    if marker_df.empty:
        return

    fig, ax = plt.subplots(figsize=(9, 8))
    cmap = plt.get_cmap("tab10")
    ids = list(tracks_df["marker_id"].drop_duplicates())

    for idx, marker_id in enumerate(ids):
        track = tracks_df[tracks_df["marker_id"] == marker_id].sort_values("time")
        if len(track) == 0:
            continue
        color = cmap(idx % 10)
        ax.plot(values(track, "x"), values(track, "y"), color=color, linewidth=1.8, alpha=0.8)
        ax.scatter(values(track, "x"), values(track, "y"), c=values(track, "t_rel"), cmap="viridis", s=16, alpha=0.65)
        ax.scatter([track["x"].iloc[0]], [track["y"].iloc[0]], marker="o", color=color, edgecolor="black", s=55)
        ax.scatter([track["x"].iloc[-1]], [track["y"].iloc[-1]], marker="s", color=color, edgecolor="black", s=55)
        if idx < 10:
            ax.plot([], [], color=color, linewidth=2, label=f"obstacle {marker_id}")

    if not path_df.empty:
        ax.plot(values(path_df, "x_cmd_int"), values(path_df, "y_cmd_int"), color="black", linewidth=2.2, label="command path")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title("Moving Obstacle Tracks in World XY")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / "moving_obstacle_tracks_xy.png", dpi=180)
    plt.close(fig)


def save_obstacle_speed_timeline(tracks_df: pd.DataFrame, out_dir: Path) -> None:
    if tracks_df.empty or "speed_xy" not in tracks_df:
        return

    valid = tracks_df[np.isfinite(tracks_df["speed_xy"])].copy()
    if valid.empty:
        return

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for marker_id, track in valid.groupby("marker_id"):
        axes[0].plot(values(track, "t_rel"), values(track, "speed_xy"), alpha=0.75, linewidth=1.3, label=f"id {marker_id}")

    speed_summary = (
        valid.groupby("time", as_index=False)
        .agg(t_rel=("t_rel", "first"), max_obstacle_speed=("speed_xy", "max"), mean_obstacle_speed=("speed_xy", "mean"))
        .sort_values("t_rel")
    )
    axes[1].plot(values(speed_summary, "t_rel"), values(speed_summary, "max_obstacle_speed"), label="max speed")
    axes[1].plot(values(speed_summary, "t_rel"), values(speed_summary, "mean_obstacle_speed"), label="mean speed", alpha=0.8)

    axes[0].set_title("Estimated Obstacle Speeds by Track")
    axes[0].set_ylabel("m/s")
    axes[0].grid(True, alpha=0.3)
    if valid["marker_id"].nunique() <= 8:
        axes[0].legend(fontsize=8)

    axes[1].set_title("Obstacle Speed Summary")
    axes[1].set_xlabel("time since run start [s]")
    axes[1].set_ylabel("m/s")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(out_dir / "obstacle_speed_timeline.png", dpi=160)
    plt.close(fig)


def save_control_obstacle_response(merged_df: pd.DataFrame, out_dir: Path) -> None:
    required = {"t_rel", "vx_cmd", "yaw_rate_cmd", "obstacle_count", "nearest_obstacle_dist"}
    if merged_df.empty or not required.issubset(merged_df.columns):
        return

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(values(merged_df, "t_rel"), values(merged_df, "nearest_obstacle_dist"), color="tab:orange")
    axes[0].set_title("Nearest Obstacle Distance")
    axes[0].set_ylabel("m")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(values(merged_df, "t_rel"), values(merged_df, "vx_cmd"), label="vx", color="tab:blue")
    axes[1].plot(values(merged_df, "t_rel"), values(merged_df, "yaw_rate_cmd"), label="yaw rate", color="tab:red")
    axes[1].set_title("Robot Command Response")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].step(values(merged_df, "t_rel"), values(merged_df, "obstacle_count"), where="post", color="tab:green")
    axes[2].set_title("Detected Obstacle Count")
    axes[2].set_xlabel("time since run start [s]")
    axes[2].set_ylabel("count")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "control_obstacle_response.png", dpi=160)
    plt.close(fig)


def save_velocity_plot(vel_df: pd.DataFrame, out_dir: Path) -> None:
    if vel_df.empty:
        return
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(values(vel_df, "t_rel"), values(vel_df, "vx_cmd"), label="vx")
    axes[0].plot(values(vel_df, "t_rel"), values(vel_df, "vy_cmd"), label="vy")
    axes[0].set_ylabel("m/s")
    axes[0].set_title("Commanded Linear Velocity")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(values(vel_df, "t_rel"), values(vel_df, "yaw_rate_cmd"), color="tab:red")
    axes[1].set_ylabel("rad/s")
    axes[1].set_title("Commanded Yaw Rate")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(values(vel_df, "t_rel"), values(vel_df, "speed_cmd"), color="tab:green")
    axes[2].set_xlabel("time since run start [s]")
    axes[2].set_ylabel("m/s")
    axes[2].set_title("Commanded Speed Magnitude")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "commanded_velocity.png", dpi=160)
    plt.close(fig)


def save_obstacle_timeline(marker_summary_df: pd.DataFrame, out_dir: Path) -> None:
    if marker_summary_df.empty:
        return
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].step(values(marker_summary_df, "t_rel"), values(marker_summary_df, "obstacle_count"), where="post")
    axes[0].set_ylabel("count")
    axes[0].set_title("Detected Obstacle Count")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(values(marker_summary_df, "t_rel"), values(marker_summary_df, "nearest_obstacle_dist"), label="nearest")
    axes[1].plot(values(marker_summary_df, "t_rel"), values(marker_summary_df, "mean_obstacle_dist"), label="mean", alpha=0.8)
    axes[1].set_xlabel("time since run start [s]")
    axes[1].set_ylabel("distance in world XY [m]")
    axes[1].set_title("Obstacle Distance From World Origin")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "obstacle_timeline.png", dpi=160)
    plt.close(fig)


def save_obstacle_xy(marker_df: pd.DataFrame, out_dir: Path) -> None:
    if marker_df.empty:
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    scatter = ax.scatter(
        values(marker_df, "x"),
        values(marker_df, "y"),
        c=values(marker_df, "t_rel"),
        s=np.clip(values(marker_df, "scale_x") * values(marker_df, "scale_y") * 80.0, 12.0, 180.0),
        alpha=0.55,
        cmap="viridis",
    )
    ax.scatter([0.0], [0.0], marker="x", color="black", label="world origin")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title("Detected Obstacle Positions")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.colorbar(scatter, ax=ax, label="time since run start [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "obstacle_xy.png", dpi=160)
    plt.close(fig)


def save_command_path(path_df: pd.DataFrame, marker_df: pd.DataFrame, out_dir: Path) -> None:
    if path_df.empty:
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    if not marker_df.empty:
        ax.scatter(values(marker_df, "x"), values(marker_df, "y"), s=18, alpha=0.25, label="obstacle markers")
    ax.plot(values(path_df, "x_cmd_int"), values(path_df, "y_cmd_int"), color="tab:red", linewidth=2, label="integrated command path")
    ax.scatter([path_df["x_cmd_int"].iloc[0]], [path_df["y_cmd_int"].iloc[0]], color="tab:green", label="start")
    ax.scatter([path_df["x_cmd_int"].iloc[-1]], [path_df["y_cmd_int"].iloc[-1]], color="tab:red", label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Dead-Reckoned Path From Commanded vx/yaw")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "command_integrated_path.png", dpi=160)
    plt.close(fig)


def save_overview(merged_df: pd.DataFrame, out_dir: Path) -> None:
    if merged_df.empty:
        return
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    if {"t_rel", "vx_cmd", "yaw_rate_cmd"}.issubset(merged_df.columns):
        axes[0].plot(values(merged_df, "t_rel"), values(merged_df, "vx_cmd"), label="vx")
        axes[0].plot(values(merged_df, "t_rel"), values(merged_df, "yaw_rate_cmd"), label="yaw rate")
    axes[0].set_title("Control Commands")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    if "obstacle_count" in merged_df:
        axes[1].step(values(merged_df, "t_rel"), values(merged_df, "obstacle_count"), where="post")
    axes[1].set_title("Camera Obstacle Count")
    axes[1].grid(True, alpha=0.3)

    if "nearest_obstacle_dist" in merged_df:
        axes[2].plot(values(merged_df, "t_rel"), values(merged_df, "nearest_obstacle_dist"), color="tab:orange")
    axes[2].set_title("Nearest Obstacle Distance")
    axes[2].grid(True, alpha=0.3)

    if "speed_cmd" in merged_df:
        axes[3].plot(values(merged_df, "t_rel"), values(merged_df, "speed_cmd"), color="tab:green")
    axes[3].set_title("Commanded Speed")
    axes[3].set_xlabel("time since run start [s]")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "run_overview.png", dpi=160)
    plt.close(fig)


def write_run_summary(stamp: str, marker_bag: Path, vel_bag: Path, out_dir: Path, vel_df: pd.DataFrame, marker_df: pd.DataFrame) -> None:
    duration = 0.0
    if not vel_df.empty:
        duration = float(vel_df["t_rel"].iloc[-1])

    summary = {
        "run": stamp,
        "marker_bag": marker_bag.name,
        "vel_bag": vel_bag.name,
        "duration_s": duration,
        "velocity_messages": len(vel_df),
        "marker_rows": len(marker_df),
        "max_vx_cmd": float(vel_df["vx_cmd"].max()) if not vel_df.empty else np.nan,
        "min_vx_cmd": float(vel_df["vx_cmd"].min()) if not vel_df.empty else np.nan,
        "max_abs_yaw_rate_cmd": float(vel_df["yaw_rate_cmd"].abs().max()) if not vel_df.empty else np.nan,
        "max_obstacle_count": int(marker_df.groupby("time").size().max()) if not marker_df.empty else 0,
    }
    pd.Series(summary).to_frame("value").to_csv(out_dir / "summary.csv")


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_position_bag(path: Path, topic: str = "position") -> pd.DataFrame:
    rows = []
    with rosbag.Bag(str(path), allow_unindexed=True) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            qx = float(msg.pose.orientation.x)
            qy = float(msg.pose.orientation.y)
            qz = float(msg.pose.orientation.z)
            qw = float(msg.pose.orientation.w)
            rows.append(
                {
                    "time": t.to_sec(),
                    "x": float(msg.pose.position.x),
                    "y": float(msg.pose.position.y),
                    "z": float(msg.pose.position.z),
                    "qx": qx,
                    "qy": qy,
                    "qz": qz,
                    "qw": qw,
                    "yaw": quat_to_yaw(qx, qy, qz, qw),
                }
            )
    df = pd.DataFrame(rows)
    if df.empty:
        return df
    df["t_rel"] = df["time"] - df["time"].iloc[0]
    dx = df["x"].diff()
    dy = df["y"].diff()
    dt = df["t_rel"].diff()
    df["speed_xy_est"] = np.hypot(dx, dy) / dt.replace(0.0, np.nan)
    return df


def save_position_plots(pos_df: pd.DataFrame, out_dir: Path) -> None:
    if pos_df.empty:
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    scatter = ax.scatter(values(pos_df, "x"), values(pos_df, "y"), c=values(pos_df, "t_rel"), s=14, cmap="viridis")
    ax.plot(values(pos_df, "x"), values(pos_df, "y"), color="black", alpha=0.35, linewidth=1)
    ax.scatter([pos_df["x"].iloc[0]], [pos_df["y"].iloc[0]], color="tab:green", label="start")
    ax.scatter([pos_df["x"].iloc[-1]], [pos_df["y"].iloc[-1]], color="tab:red", label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title("Robot XY Position")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.colorbar(scatter, ax=ax, label="time since bag start [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "robot_xy_position.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(values(pos_df, "t_rel"), values(pos_df, "x"), label="x")
    axes[0].plot(values(pos_df, "t_rel"), values(pos_df, "y"), label="y")
    axes[0].set_ylabel("m")
    axes[0].set_title("Robot Position Components")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(values(pos_df, "t_rel"), values(pos_df, "z"), color="tab:purple")
    axes[1].set_ylabel("m")
    axes[1].set_title("Robot Z")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(values(pos_df, "t_rel"), values(pos_df, "yaw"), color="tab:red")
    axes[2].set_ylabel("rad")
    axes[2].set_title("Robot Yaw From Quaternion")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(values(pos_df, "t_rel"), values(pos_df, "speed_xy_est"), color="tab:green")
    axes[3].set_xlabel("time since bag start [s]")
    axes[3].set_ylabel("m/s")
    axes[3].set_title("Estimated XY Speed From Position Differences")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "robot_position_timeseries.png", dpi=160)
    plt.close(fig)


def analyze_position_bags(root: Path, out_root: Path) -> list[dict[str, object]]:
    results = []
    position_root = out_root / "position_bags"
    for bag_path in sorted(root.glob("pos*.bag")):
        out_dir = position_root / bag_path.stem
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"Analyzing position bag {bag_path.name}")
        try:
            pos_df = read_position_bag(bag_path)
            pos_df.to_csv(out_dir / "position.csv", index=False)
            save_position_plots(pos_df, out_dir)
            results.append(
                {
                    "bag": bag_path.name,
                    "status": "ok" if not pos_df.empty else "empty",
                    "messages": len(pos_df),
                    "output_dir": str(out_dir),
                    "error": "",
                }
            )
        except (ROSBagException, ROSBagFormatException, ROSBagUnindexedException, OSError, ValueError) as exc:
            print(f"  skipped position bag: {type(exc).__name__}: {exc}")
            results.append(
                {
                    "bag": bag_path.name,
                    "status": "skipped",
                    "messages": 0,
                    "output_dir": "",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
    return results


def analyze_run(stamp: str, marker_bag: Path, vel_bag: Path, out_root: Path) -> dict[str, object]:
    out_dir = out_root / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    vel_df = read_velocity_bag(vel_bag)
    marker_df, marker_summary_df = read_marker_bag(marker_bag)
    merged_df = merge_timeseries(vel_df, marker_summary_df)
    path_df = dead_reckon_commands(vel_df)

    vel_df.to_csv(out_dir / "velocity.csv", index=False)
    marker_df.to_csv(out_dir / "markers.csv", index=False)
    marker_summary_df.to_csv(out_dir / "marker_summary.csv", index=False)
    merged_df.to_csv(out_dir / "combined_timeseries.csv", index=False)
    path_df.to_csv(out_dir / "command_integrated_path.csv", index=False)
    write_run_summary(stamp, marker_bag, vel_bag, out_dir, vel_df, marker_df)
    tracks_df = build_obstacle_tracks(marker_df)
    tracks_df.to_csv(out_dir / "obstacle_tracks.csv", index=False)

    for bag_path in (marker_bag, vel_bag):
        dest = out_dir / bag_path.name
        if bag_path.resolve() != dest.resolve():
            shutil.copy2(bag_path, dest)

    save_velocity_plot(vel_df, out_dir)
    save_obstacle_timeline(marker_summary_df, out_dir)
    save_obstacle_xy(marker_df, out_dir)
    save_command_path(path_df, marker_df, out_dir)
    save_overview(merged_df, out_dir)
    save_obstacle_tracks_xy(marker_df, tracks_df, path_df, out_dir)
    save_obstacle_speed_timeline(tracks_df, out_dir)
    save_control_obstacle_response(merged_df, out_dir)

    return {
        "run": stamp,
        "status": "ok" if not vel_df.empty or not marker_df.empty else "empty",
        "marker_bag": marker_bag.name,
        "vel_bag": vel_bag.name,
        "velocity_messages": len(vel_df),
        "marker_rows": len(marker_df),
        "output_dir": str(out_dir),
        "error": "",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", default="bags_plots", type=Path, help="Directory containing markers_*.bag and vel_*.bag files.")
    parser.add_argument("--out-dir", default="bags_plots", type=Path, help="Output directory for per-run CSVs, plots, and source bag copies.")
    parser.add_argument("--limit", type=int, default=None, help="Analyze only the first N paired runs.")
    args = parser.parse_args()

    runs = find_runs(args.bag_dir)
    if args.limit is not None:
        runs = runs[: args.limit]

    if not runs:
        raise SystemExit(f"No marker/velocity bag pairs found in {args.bag_dir}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for stamp, marker_bag, vel_bag in runs:
        print(f"Analyzing {stamp}: {marker_bag.name} + {vel_bag.name}")
        try:
            results.append(analyze_run(stamp, marker_bag, vel_bag, args.out_dir))
        except (ROSBagException, ROSBagFormatException, ROSBagUnindexedException, OSError, ValueError) as exc:
            print(f"  skipped: {type(exc).__name__}: {exc}")
            results.append(
                {
                    "run": stamp,
                    "status": "skipped",
                    "marker_bag": marker_bag.name,
                    "vel_bag": vel_bag.name,
                    "velocity_messages": 0,
                    "marker_rows": 0,
                    "output_dir": "",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )

    pd.DataFrame(results).to_csv(args.out_dir / "batch_summary.csv", index=False)
    position_results = analyze_position_bags(args.bag_dir, args.out_dir)
    if position_results:
        pd.DataFrame(position_results).to_csv(args.out_dir / "position_batch_summary.csv", index=False)
    ok_count = sum(1 for result in results if result["status"] == "ok")
    skipped_count = sum(1 for result in results if result["status"] == "skipped")
    empty_count = sum(1 for result in results if result["status"] == "empty")
    print(f"Done. Parsed {ok_count} run(s), skipped {skipped_count}, empty {empty_count}.")
    print(f"Batch summary: {args.out_dir / 'batch_summary.csv'}")
    if position_results:
        print(f"Position summary: {args.out_dir / 'position_batch_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
