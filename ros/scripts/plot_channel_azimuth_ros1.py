#!/usr/bin/env python3

import argparse
import math
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


DEFAULT_OUTPUT_DIR = Path.home() / "デスクトップ" / "cepton_azimuth_plots"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Plot one second of azimuth versus timestamp for one channel, "
            "repeating every ten seconds."
        )
    )
    parser.add_argument(
        "--topic",
        default="/cepton3/points",
        help="PointCloud2 topic (default: /cepton3/points)",
    )
    parser.add_argument(
        "--channel-id",
        type=int,
        default=200,
        help="Channel ID to plot (default: 200)",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=1.0,
        help="Collection window in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=10.0,
        help="Interval between collection-window starts (default: 10.0)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help=(
            "Directory for PNG files "
            "(default: ~/デスクトップ/cepton_azimuth_plots)"
        ),
    )
    parser.add_argument(
        "--degrees",
        action="store_true",
        help="Plot azimuth in degrees instead of radians",
    )
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    if args.window <= 0:
        parser.error("--window must be greater than zero")
    if args.interval < args.window:
        parser.error("--interval must be greater than or equal to --window")
    if not 0 <= args.channel_id <= 65535:
        parser.error("--channel-id must be between 0 and 65535")
    return args


class AzimuthPlotter:
    REQUIRED_FIELDS = {"channel_id", "azimuth", "relative_timestamp"}

    def __init__(self, args):
        self.args = args
        self.output_dir = Path(args.output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.timestamps = []
        self.azimuths = []
        self.cycle_start = None
        self.collecting = False
        self.fields_checked = False
        self.stopped = False

        self.subscriber = rospy.Subscriber(
            args.topic, PointCloud2, self.on_cloud, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.on_timer)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "Collecting channel %d for %.3f s every %.3f s from %s; output: %s",
            args.channel_id,
            args.window,
            args.interval,
            args.topic,
            self.output_dir,
        )

    def on_cloud(self, msg):
        with self.lock:
            if self.cycle_start is None:
                self.cycle_start = time.monotonic()
                self.collecting = True
                rospy.loginfo(
                    "Received first cloud; started %.3f s collection window",
                    self.args.window,
                )
            if not self.collecting:
                return

        if not self.fields_checked:
            available = {field.name for field in msg.fields}
            missing = self.REQUIRED_FIELDS - available
            if missing:
                rospy.logfatal(
                    "Topic %s is missing required PointCloud2 fields: %s. "
                    "Build cepton_ros with WITH_TS_CH_F=ON and WITH_POLAR=ON.",
                    self.args.topic,
                    ", ".join(sorted(missing)),
                )
                rospy.signal_shutdown("required PointCloud2 fields are missing")
                return
            self.fields_checked = True

        point_timestamp = msg.header.stamp.to_sec()
        new_timestamps = []
        new_azimuths = []
        points = point_cloud2.read_points(
            msg,
            field_names=("channel_id", "azimuth", "relative_timestamp"),
            skip_nans=True,
        )
        for channel_id, azimuth, relative_timestamp in points:
            # relative_timestamp is the delta from the previous point in us.
            # Accumulate every point before filtering by channel.
            point_timestamp += relative_timestamp * 1.0e-6
            if channel_id != self.args.channel_id or not math.isfinite(azimuth):
                continue
            new_timestamps.append(point_timestamp)
            new_azimuths.append(
                math.degrees(azimuth) if self.args.degrees else azimuth
            )

        with self.lock:
            if self.collecting:
                self.timestamps.extend(new_timestamps)
                self.azimuths.extend(new_azimuths)

    def on_timer(self, _event):
        now = time.monotonic()
        data_to_plot = None

        with self.lock:
            if self.cycle_start is None:
                return
            elapsed = now - self.cycle_start
            if self.collecting and elapsed >= self.args.window:
                self.collecting = False
                data_to_plot = (self.timestamps, self.azimuths)
                self.timestamps = []
                self.azimuths = []

            if not self.collecting and elapsed >= self.args.interval:
                skipped = int(elapsed // self.args.interval)
                self.cycle_start += skipped * self.args.interval
                self.collecting = True
                rospy.loginfo(
                    "Started %.3f s collection window for channel %d",
                    self.args.window,
                    self.args.channel_id,
                )

        if data_to_plot is not None:
            self.save_plot(*data_to_plot)

    def save_plot(self, timestamps, azimuths):
        if not timestamps:
            rospy.logwarn(
                "No points for channel %d in this collection window; "
                "no graph was saved",
                self.args.channel_id,
            )
            return

        first_timestamp = min(timestamps)
        relative_timestamps = [
            timestamp - first_timestamp for timestamp in timestamps
        ]
        stamp = datetime.fromtimestamp(first_timestamp).strftime(
            "%Y%m%d_%H%M%S_%f"
        )
        output_path = self.output_dir / (
            f"channel_{self.args.channel_id}_azimuth_{stamp}.png"
        )

        figure, axes = plt.subplots(figsize=(10, 5))
        axes.scatter(relative_timestamps, azimuths, s=2, alpha=0.7)
        axes.set_xlabel(
            f"Timestamp offset from {first_timestamp:.6f} [s]"
        )
        axes.set_ylabel(
            "Azimuth [deg]" if self.args.degrees else "Azimuth [rad]"
        )
        axes.set_title(
            f"Channel {self.args.channel_id}: azimuth vs timestamp "
            f"({self.args.window:g} s window)"
        )
        axes.grid(True, alpha=0.3)
        figure.tight_layout()
        figure.savefig(str(output_path), dpi=150)
        plt.close(figure)
        rospy.loginfo("Saved %d points to %s", len(timestamps), output_path)

    def on_shutdown(self):
        with self.lock:
            if self.stopped:
                return
            self.stopped = True
            timestamps = self.timestamps
            azimuths = self.azimuths
            self.timestamps = []
            self.azimuths = []
        if timestamps:
            self.save_plot(timestamps, azimuths)


def main():
    args = parse_args()
    rospy.init_node("cepton_channel_azimuth_plotter")
    AzimuthPlotter(args)
    rospy.spin()


if __name__ == "__main__":
    main()
