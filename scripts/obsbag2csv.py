#!/usr/bin/env python3
"""Export detection marker arrays from a ROS bag to CSV."""

import argparse
import csv
from pathlib import Path



def default_output_path(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.stem}_obstacles.csv")


def export_markers(input_bag: Path, output_csv: Path, topic: str) -> int:
    import rosbag

    rows = 0
    with rosbag.Bag(str(input_bag), allow_unindexed=True) as bag, output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "time",
            "marker_id",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
            "scale_x",
            "scale_y",
            "scale_z",
        ])

        for _, msg, t in bag.read_messages(topics=[topic]):
            for marker in msg.markers:
                writer.writerow([
                    t.to_sec(),
                    marker.id,
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                    marker.pose.orientation.w,
                    marker.scale.x,
                    marker.scale.y,
                    marker.scale.z,
                ])
                rows += 1

    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a /detections_markers ROS bag topic to CSV.")
    parser.add_argument("input_bag", type=Path, help="Input marker bag, e.g. markers_2026-05-05_12-42-33.bag")
    parser.add_argument("-o", "--output", type=Path, help="Output CSV path. Defaults to <bag_stem>_obstacles.csv")
    parser.add_argument("--topic", default="/detections_markers", help="MarkerArray topic to export")
    args = parser.parse_args()

    output = args.output or default_output_path(args.input_bag)
    rows = export_markers(args.input_bag, output, args.topic)
    print(f"Wrote {rows} marker rows to {output}")


if __name__ == "__main__":
    main()
