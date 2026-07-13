#!/usr/bin/env python3
"""Export PoseStamped messages from a ROS bag to CSV."""

import argparse
import csv
from pathlib import Path



def default_output_path(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.stem}_position.csv")


def export_pose(input_bag: Path, output_csv: Path, topic: str) -> int:
    import rosbag

    rows = 0
    with rosbag.Bag(str(input_bag), allow_unindexed=True) as bag, output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw"])

        for _, msg, t in bag.read_messages(topics=[topic]):
            writer.writerow([
                t.to_sec(),
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ])
            rows += 1

    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a PoseStamped ROS bag topic to CSV.")
    parser.add_argument("input_bag", type=Path, help="Input pose bag, e.g. pos2026-05-05_12-42-33.bag")
    parser.add_argument("-o", "--output", type=Path, help="Output CSV path. Defaults to <bag_stem>_position.csv")
    parser.add_argument("--topic", default="/camera_pose", help="PoseStamped topic to export")
    args = parser.parse_args()

    output = args.output or default_output_path(args.input_bag)
    rows = export_pose(args.input_bag, output, args.topic)
    print(f"Wrote {rows} pose rows to {output}")


if __name__ == "__main__":
    main()
