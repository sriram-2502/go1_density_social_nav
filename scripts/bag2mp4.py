import argparse

import cv2
import rosbag
from cv_bridge import CvBridge


def bag_to_mp4(input_bag, output_mp4, topic, fps):
    bridge = CvBridge()
    video_writer = None
    frame_count = 0

    with rosbag.Bag(input_bag) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if video_writer is None:
                height, width, _ = cv_image.shape
                video_writer = cv2.VideoWriter(
                    output_mp4,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    fps,
                    (width, height),
                )
                if not video_writer.isOpened():
                    raise RuntimeError(f"Could not open video writer for {output_mp4}")

            video_writer.write(cv_image)
            frame_count += 1

    if video_writer is None:
        raise RuntimeError(f"No frames found on topic {topic!r} in {input_bag}")

    video_writer.release()
    return frame_count


def main():
    parser = argparse.ArgumentParser(description="Convert a ROS image bag topic to MP4.")
    parser.add_argument("input_bag", nargs="?", default="video.bag")
    parser.add_argument("output_mp4", nargs="?", default="output.mp4")
    parser.add_argument("--topic", default="video")
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args()

    frames = bag_to_mp4(args.input_bag, args.output_mp4, args.topic, args.fps)
    print(f"Wrote {frames} frames to {args.output_mp4}")


if __name__ == "__main__":
    main()
