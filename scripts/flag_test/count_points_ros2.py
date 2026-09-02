#!/usr/bin/env python3

"""Receive one PointCloud2 message and print its point count."""

import argparse
import sys
import time

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
    parser.add_argument(
        "--target-stamp",
        help="Only count a message with this header stamp, formatted as sec.nanosec",
    )
    return parser.parse_args(argv)


class OneMessageSubscriber(Node):

    def __init__(self, topic, target_stamp):
        super().__init__("flag_test_point_counter")
        self.message = None
        self.target_stamp = target_stamp
        self.subscription = self.create_subscription(
            PointCloud2, topic, self._on_message, 10
        )

    def _on_message(self, message):
        if self.target_stamp is not None:
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp != self.target_stamp:
                return
        if self.message is None:
            self.message = message


def parse_stamp(value):
    if value is None:
        return None

    parts = value.split(".", 1)
    if len(parts) != 2:
        raise ValueError("--target-stamp must be formatted as sec.nanosec")

    sec = int(parts[0])
    nanosec_text = parts[1]
    if len(nanosec_text) > 9:
        raise ValueError("--target-stamp nanosecond part must be 9 digits or fewer")

    nanosec = int(nanosec_text.ljust(9, "0"))
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError("--target-stamp nanosecond part is out of range")

    return sec, nanosec


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.timeout <= 0:
        print("ERROR: --timeout must be greater than zero", file=sys.stderr)
        return 2
    try:
        target_stamp = parse_stamp(args.target_stamp)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = OneMessageSubscriber(args.topic, target_stamp)

    try:
        deadline = time.monotonic() + args.timeout
        while node.message is None and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(node, timeout_sec=min(1.0, max(0.0, remaining)))

        if node.message is None:
            print(
                f"ERROR: timed out waiting for one message on {args.topic}",
                file=sys.stderr,
            )
            return 1

        message = node.message
        point_count = message.width * message.height
        print(f"TOPIC={args.topic}")
        print(f"FRAME_ID={message.header.frame_id}")
        print(f"STAMP={message.header.stamp.sec}.{message.header.stamp.nanosec:09d}")
        print(f"WIDTH={message.width}")
        print(f"HEIGHT={message.height}")
        print(f"POINT_COUNT={point_count}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
