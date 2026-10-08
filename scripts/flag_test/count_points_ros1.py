#!/usr/bin/env python3

"""Receive one ROS1 PointCloud2 message and print its point count."""

import argparse
import sys
import time

import rospy
from sensor_msgs.msg import PointCloud2


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Receive one PointCloud2 message and count its points."
    )
    parser.add_argument(
        "--topic",
        default="/cepton3/points",
        help="PointCloud2 topic to subscribe to (default: /cepton3/points)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for a message (default: 30)",
    )
    parser.add_argument(
        "--target-stamp",
        help=(
            "Only count a message with this header stamp, "
            "formatted as sec.nanosec"
        ),
    )
    return parser.parse_args(argv)


def parse_stamp(value):
    if value is None:
        return None

    parts = value.split(".", 1)
    if len(parts) != 2:
        raise ValueError("--target-stamp must be formatted as sec.nanosec")

    sec = int(parts[0])
    nanosec_text = parts[1]
    if len(nanosec_text) > 9:
        raise ValueError("--target-stamp nanosecond part is too long")

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

    rospy.init_node("flag_test_point_counter", anonymous=True)
    message = [None]

    def on_message(candidate):
        stamp = (candidate.header.stamp.secs, candidate.header.stamp.nsecs)
        if target_stamp is None or stamp == target_stamp:
            message[0] = candidate

    rospy.Subscriber(args.topic, PointCloud2, on_message, queue_size=1)
    deadline = time.monotonic() + args.timeout
    while (
        message[0] is None
        and time.monotonic() < deadline
        and not rospy.is_shutdown()
    ):
        time.sleep(0.01)

    if message[0] is None:
        print(
            f"ERROR: timed out waiting for one message on {args.topic}",
            file=sys.stderr,
        )
        return 1

    received = message[0]
    print(f"TOPIC={args.topic}")
    print(f"FRAME_ID={received.header.frame_id}")
    print(
        f"STAMP={received.header.stamp.secs}."
        f"{received.header.stamp.nsecs:09d}"
    )
    print(f"WIDTH={received.width}")
    print(f"HEIGHT={received.height}")
    print(f"POINT_COUNT={received.width * received.height}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
