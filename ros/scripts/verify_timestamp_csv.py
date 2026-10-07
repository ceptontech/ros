#!/usr/bin/env python3
"""Verify per-point timestamp semantics in a captured ROS1 PointCloud2 CSV."""

import argparse
import csv
import json
import sys
from pathlib import Path


MODE_RELATIVE = "relative"
MODE_FRAME_OFFSET = "frame_offset"
MODE_ABSOLUTE = "absolute"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate a PointCloud2 CSV captured by capture_1frame_ros1.py."
    )
    parser.add_argument(
        "--input", required=True, type=Path, help="Input CSV captured with --include-header"
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=(MODE_RELATIVE, MODE_FRAME_OFFSET, MODE_ABSOLUTE),
        help="Timestamp mode used when building the driver",
    )
    parser.add_argument(
        "--min-max-offset-us",
        type=int,
        default=0,
        help=(
            "Require the largest frame offset to be at least this many microseconds. "
            "Use 75000 to test a 10 Hz product."
        ),
    )
    parser.add_argument(
        "--allow-trimmed-frame",
        action="store_true",
        help="Allow a capture whose first retained point is not at the frame start",
    )
    parser.add_argument("--report", type=Path, help="Write the JSON result to this path")
    return parser.parse_args()


def parse_int(value, field_name, row_number):
    if value is None or value == "":
        raise ValueError(f"row {row_number}: {field_name} is empty")
    try:
        return int(value)
    except ValueError:
        try:
            return int(float(value))
        except ValueError as exc:
            raise ValueError(
                f"row {row_number}: {field_name}={value!r} is not an integer"
            ) from exc


def add_error(errors, message):
    # A few representative errors are enough; the summary retains the count.
    if len(errors) < 20:
        errors.append(message)


def load_rows(input_path):
    try:
        csv_file = input_path.open(newline="")
    except OSError as exc:
        raise RuntimeError(f"cannot open {input_path}: {exc}") from exc

    with csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise RuntimeError("CSV has no header row")
        rows = list(reader)

    if not rows:
        raise RuntimeError("CSV contains no points")
    return reader.fieldnames, rows


def verify_relative(rows, errors):
    nonzero_non_channel_zero = 0
    channel_zero_count = 0
    max_relative_timestamp = 0

    for row_number, row in enumerate(rows, start=2):
        try:
            channel_id = parse_int(row["channel_id"], "channel_id", row_number)
            relative_timestamp = parse_int(
                row["relative_timestamp"], "relative_timestamp", row_number
            )
        except ValueError as exc:
            add_error(errors, str(exc))
            continue

        if channel_id == 0:
            channel_zero_count += 1
        elif relative_timestamp != 0:
            nonzero_non_channel_zero += 1
        max_relative_timestamp = max(max_relative_timestamp, relative_timestamp)

    if channel_zero_count == 0:
        add_error(errors, "no channel_id == 0 points were captured")

    return {
        "channel_zero_count": channel_zero_count,
        "max_relative_timestamp_us": max_relative_timestamp,
        "nonzero_relative_timestamp_on_nonzero_channel_count": nonzero_non_channel_zero,
    }


def verify_timestamped(rows, mode, min_max_offset_us, allow_trimmed_frame, errors):
    previous_timestamp = None
    packet_timestamp = None
    channel_zero_count = 0
    packet_boundary_count = 0
    packet_mismatch_count = 0
    timestamps = []

    header_timestamp_us = None
    if mode == MODE_ABSOLUTE:
        first_row = rows[0]
        try:
            header_timestamp_us = (
                parse_int(first_row["stamp_sec"], "stamp_sec", 2) * 1_000_000
                + parse_int(first_row["stamp_nsec"], "stamp_nsec", 2) // 1_000
            )
        except ValueError as exc:
            add_error(errors, str(exc))

    for row_number, row in enumerate(rows, start=2):
        try:
            timestamp = parse_int(row["timestamp"], "timestamp", row_number)
            channel_id = parse_int(row["channel_id"], "channel_id", row_number)
        except ValueError as exc:
            add_error(errors, str(exc))
            continue

        timestamps.append(timestamp)
        if previous_timestamp is not None and timestamp < previous_timestamp:
            add_error(
                errors,
                f"row {row_number}: timestamp regressed from {previous_timestamp} to {timestamp}",
            )
        previous_timestamp = timestamp

        if channel_id == 0:
            channel_zero_count += 1
            packet_boundary_count += 1
            packet_timestamp = timestamp
        elif packet_timestamp is not None and timestamp != packet_timestamp:
            packet_mismatch_count += 1
            add_error(
                errors,
                f"row {row_number}: timestamp {timestamp} differs from packet timestamp "
                f"{packet_timestamp}",
            )

    if not timestamps:
        add_error(errors, "no valid timestamp rows were captured")
        return {}

    if channel_zero_count == 0:
        add_error(errors, "no channel_id == 0 points were captured")

    if mode == MODE_FRAME_OFFSET:
        offsets = timestamps
    else:
        if header_timestamp_us is None:
            offsets = []
        else:
            offsets = [timestamp - header_timestamp_us for timestamp in timestamps]

    if offsets:
        if min(offsets) < 0:
            add_error(errors, f"timestamp offset is negative ({min(offsets)} us)")
        if not allow_trimmed_frame and min(offsets) != 0:
            add_error(
                errors,
                f"first retained packet is not the frame start (minimum offset is {min(offsets)} us)",
            )
        if max(offsets) < min_max_offset_us:
            add_error(
                errors,
                f"maximum offset is {max(offsets)} us; expected at least {min_max_offset_us} us",
            )

    return {
        "channel_zero_count": channel_zero_count,
        "packet_boundary_count": packet_boundary_count,
        "packet_timestamp_mismatch_count": packet_mismatch_count,
        "header_timestamp_us": header_timestamp_us,
        "min_offset_us": min(offsets) if offsets else None,
        "max_offset_us": max(offsets) if offsets else None,
    }


def main():
    args = parse_args()
    errors = []

    try:
        field_names, rows = load_rows(args.input)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    required_fields = {"channel_id"}
    if args.mode == MODE_RELATIVE:
        required_fields.add("relative_timestamp")
    else:
        required_fields.add("timestamp")
    if args.mode == MODE_ABSOLUTE:
        required_fields.update(("stamp_sec", "stamp_nsec"))

    missing_fields = sorted(required_fields.difference(field_names))
    if missing_fields:
        errors.append("missing required CSV fields: " + ", ".join(missing_fields))
        details = {}
    elif args.mode == MODE_RELATIVE:
        details = verify_relative(rows, errors)
    else:
        details = verify_timestamped(
            rows,
            args.mode,
            args.min_max_offset_us,
            args.allow_trimmed_frame,
            errors,
        )

    report = {
        "input": str(args.input),
        "mode": args.mode,
        "point_count": len(rows),
        "fields": field_names,
        "passed": not errors,
        "errors": errors,
        **details,
    }
    report_text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(report_text)

    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(report_text + "\n")

    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
