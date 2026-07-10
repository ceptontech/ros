#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Save a single PointCloud2 frame from a ROS 2 topic to CSV.'
    )
    parser.add_argument(
        '--topic',
        default='/cepton_pcl2',
        help='PointCloud2 topic name (default: /cepton_pcl2)',
    )
    parser.add_argument(
        '--output',
        default=str(SCRIPT_DIR / 'cepton_frame.csv'),
        help='Output CSV path (default: cepton_frame.csv next to this script)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help='Seconds to wait for one frame (default: 10.0)',
    )
    parser.add_argument(
        '--include-header',
        action='store_true',
        help='Add ROS header columns at the start of each row',
    )
    return parser.parse_args(argv)


class OneFrameSubscriber(Node):

    def __init__(self, topic):
        super().__init__('save_cepton_frame_to_csv')
        self.future = rclpy.task.Future()
        self.subscription = self.create_subscription(
            PointCloud2, topic, self._on_message, 10
        )

    def _on_message(self, msg):
        if not self.future.done():
            self.future.set_result(msg)


def point_to_row(point):
    if hasattr(point, 'tolist'):
        point = point.tolist()
    if isinstance(point, tuple):
        return list(point)
    if isinstance(point, list):
        return point
    return [point]


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    node = OneFrameSubscriber(args.topic)

    try:
        rclpy.spin_until_future_complete(
            node, node.future, timeout_sec=args.timeout
        )
        if not node.future.done():
            print(
                f'Failed to receive a frame from {args.topic}: '
                f'timed out after {args.timeout} seconds',
                file=sys.stderr,
            )
            return 1

        msg = node.future.result()
        field_names = [field.name for field in msg.fields]
        header = list(field_names)
        if args.include_header:
            header = ['stamp_sec', 'stamp_nsec', 'frame_id'] + header

        with output_path.open('w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)

            points = point_cloud2.read_points(
                msg, field_names=field_names, skip_nans=False
            )
            for point in points:
                row = point_to_row(point)
                if args.include_header:
                    row = [
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                        msg.header.frame_id,
                    ] + row
                writer.writerow(row)

        print(
            f'Saved 1 frame from {args.topic} to {output_path} '
            f'({msg.width * msg.height} points before NaN filtering).'
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
