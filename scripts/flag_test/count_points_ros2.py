#!/usr/bin/env python3

"""Receive one PointCloud2 message and print its point count."""

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Receive one PointCloud2 message and count its points."
    )
    parser.add_argument(
        "--topic",
        default="/cepton_pcl2",
        help="PointCloud2 topic to subscribe to (default: /cepton_pcl2)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for a message (default: 30)",
    )
    return parser.parse_args(argv)


class OneMessageSubscriber(Node):

    def __init__(self, topic):
        super().__init__("flag_test_point_counter")
        self.message = None
        self.subscription = self.create_subscription(
            PointCloud2, topic, self._on_message, 10
        )

    def _on_message(self, message):
        if self.message is None:
            self.message = message


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.timeout <= 0:
        print("ERROR: --timeout must be greater than zero", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = OneMessageSubscriber(args.topic)

    try:
        rclpy.spin_once(node, timeout_sec=args.timeout)
        if node.message is None:
            print(
                f"ERROR: timed out waiting for one message on {args.topic}",
                file=sys.stderr,
            )
            return 1

        message = node.message
        point_count = message.width * message.height
        print(f"TOPIC={args.topic}")
        print(f"WIDTH={message.width}")
        print(f"HEIGHT={message.height}")
        print(f"POINT_COUNT={point_count}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
