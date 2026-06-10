#!/usr/bin/env python3

import argparse
import csv
import sys
from pathlib import Path

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args():
    parser = argparse.ArgumentParser(
        description="Save a single PointCloud2 frame from a ROS topic to CSV."
    )
    parser.add_argument(
        "--topic",
        default="/cepton3/points",
        help="PointCloud2 topic name (default: /cepton3/points)",
    )
    parser.add_argument(
        "--output",
        default=str(SCRIPT_DIR / "cepton_frame.csv"),
        help="Output CSV path (default: cepton_frame.csv next to this script)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for one frame (default: 10.0)",
    )
    parser.add_argument(
        "--include-header",
        action="store_true",
        help="Add ROS header columns at the start of each row",
    )
    return parser.parse_args(rospy.myargv(sys.argv)[1:])


def main():
    args = parse_args()
    rospy.init_node("save_cepton_frame_to_csv", anonymous=True, disable_signals=True)
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        msg = rospy.wait_for_message(args.topic, PointCloud2, timeout=args.timeout)
    except rospy.ROSException as exc:
        print(f"Failed to receive a frame from {args.topic}: {exc}", file=sys.stderr)
        return 1

    field_names = [field.name for field in msg.fields]
    header = list(field_names)
    if args.include_header:
        header = ["stamp_sec", "stamp_nsec", "frame_id"] + header

    with output_path.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

        points = point_cloud2.read_points(
            msg, field_names=field_names, skip_nans=False
        )
        for point in points:
            row = list(point)
            if args.include_header:
                row = [msg.header.stamp.secs, msg.header.stamp.nsecs, msg.header.frame_id] + row
            writer.writerow(row)

    print(
        f"Saved 1 frame from {args.topic} to {output_path} "
        f"({msg.width * msg.height} points before NaN filtering)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
