#!/usr/bin/env python3
"""Export Unitree HighCmd velocity messages from a ROS bag to CSV."""

import argparse
import csv
from pathlib import Path



def default_output_path(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.stem}_velocity.csv")


def export_velocity(input_bag: Path, output_csv: Path, topic: str) -> int:
    import rosbag

    rows = 0
    with rosbag.Bag(str(input_bag), allow_unindexed=True) as bag, output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "time",
            "vx",
            "vy",
            "yaw_rate",
            "mode",
            "gait_type",
            "speed_level",
            "foot_raise_height",
            "body_height",
        ])

        for _, msg, t in bag.read_messages(topics=[topic]):
            writer.writerow([
                t.to_sec(),
                msg.velocity[0],
                msg.velocity[1],
                msg.yawSpeed,
                msg.mode,
                msg.gaitType,
                msg.speedLevel,
                msg.footRaiseHeight,
                msg.bodyHeight,
            ])
            rows += 1

    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a Unitree HighCmd ROS bag topic to CSV.")
    parser.add_argument("input_bag", type=Path, help="Input velocity bag, e.g. vel_2026-05-05_12-42-33.bag")
    parser.add_argument("-o", "--output", type=Path, help="Output CSV path. Defaults to <bag_stem>_velocity.csv")
    parser.add_argument("--topic", default="/high_cmd", help="HighCmd topic to export")
    args = parser.parse_args()

    output = args.output or default_output_path(args.input_bag)
    rows = export_velocity(args.input_bag, output, args.topic)
    print(f"Wrote {rows} velocity rows to {output}")


if __name__ == "__main__":
    main()
