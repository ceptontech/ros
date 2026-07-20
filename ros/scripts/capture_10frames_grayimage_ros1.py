#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CAPTURE_SCRIPT = SCRIPT_DIR / "capture_1frame_ros1.py"
DEFAULT_GRAYIMAGE_SCRIPT = (
    SCRIPT_DIR.parents[1] / "ros2" / "scripts" / "pointcloudgrayimage.py"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run the ROS1 one-frame capture script 10 times, then convert "
            "each CSV to a grayscale PNG with the ROS2 gray image script."
        )
    )
    parser.add_argument(
        "--topic",
        default="/cepton3/points",
        help=(
            "PointCloud2 topic name for ROS1 capture "
            "(default: /cepton3/points)"
        ),
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="Number of frames to capture and convert (default: 10)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for each frame (default: 10.0)",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.0,
        help="Seconds to wait between captures (default: 0.0)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(SCRIPT_DIR / "capture_grayimage_output"),
        help="Directory for generated CSV and PNG files",
    )
    parser.add_argument(
        "--prefix",
        default="cepton_frame",
        help="Output filename prefix (default: cepton_frame)",
    )
    parser.add_argument(
        "--gamma",
        type=float,
        default=1.0,
        help="Gamma value passed to pointcloudgrayimage.py (default: 1.0)",
    )
    parser.add_argument(
        "--include-header",
        action="store_true",
        help="Pass --include-header to capture_1frame_ros1.py",
    )
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Continue with later frames if one capture or conversion fails",
    )
    parser.add_argument(
        "--capture-script",
        default=str(DEFAULT_CAPTURE_SCRIPT),
        help="Path to capture_1frame_ros1.py",
    )
    parser.add_argument(
        "--grayimage-script",
        default=str(DEFAULT_GRAYIMAGE_SCRIPT),
        help="Path to pointcloudgrayimage.py",
    )
    return parser.parse_args()


def run_command(command):
    print("+ " + " ".join(str(part) for part in command), flush=True)
    return subprocess.run(command)


def main():
    args = parse_args()

    if args.count <= 0:
        print("--count must be greater than 0", file=sys.stderr)
        return 2

    if args.gamma <= 0:
        print("--gamma must be greater than 0", file=sys.stderr)
        return 2

    if args.delay < 0:
        print("--delay must be 0 or greater", file=sys.stderr)
        return 2

    capture_script = Path(args.capture_script).expanduser().resolve()
    grayimage_script = Path(args.grayimage_script).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    csv_dir = output_dir / "csv"
    image_dir = output_dir / "grayimage"

    if not capture_script.is_file():
        print(f"capture script not found: {capture_script}", file=sys.stderr)
        return 2

    if not grayimage_script.is_file():
        print(
            f"gray image script not found: {grayimage_script}",
            file=sys.stderr)
        return 2

    csv_dir.mkdir(parents=True, exist_ok=True)
    image_dir.mkdir(parents=True, exist_ok=True)

    failures = 0

    for index in range(1, args.count + 1):
        stem = f"{args.prefix}_{index:02d}"
        csv_path = csv_dir / f"{stem}.csv"
        png_path = image_dir / f"{stem}_gray.png"

        print(f"\n[{index}/{args.count}] capturing: {csv_path}", flush=True)
        capture_command = [
            sys.executable,
            str(capture_script),
            "--topic",
            args.topic,
            "--output",
            str(csv_path),
            "--timeout",
            str(args.timeout),
        ]
        if args.include_header:
            capture_command.append("--include-header")

        capture_result = run_command(capture_command)
        if capture_result.returncode != 0:
            failures += 1
            print(
                f"capture failed for frame {index}: "
                f"exit {capture_result.returncode}",
                file=sys.stderr,
            )
            if not args.keep_going:
                return capture_result.returncode
            continue

        print(f"[{index}/{args.count}] converting: {png_path}", flush=True)
        convert_command = [
            sys.executable,
            str(grayimage_script),
            "--input",
            str(csv_path),
            "--output",
            str(png_path),
            "--gamma",
            str(args.gamma),
        ]

        convert_result = run_command(convert_command)
        if convert_result.returncode != 0:
            failures += 1
            print(
                f"conversion failed for frame {index}: "
                f"exit {convert_result.returncode}",
                file=sys.stderr,
            )
            if not args.keep_going:
                return convert_result.returncode

        if args.delay > 0 and index != args.count:
            time.sleep(args.delay)

    print(f"\nCSV files: {csv_dir}")
    print(f"PNG files: {image_dir}")

    if failures:
        print(f"Completed with {failures} failure(s).", file=sys.stderr)
        return 1

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
