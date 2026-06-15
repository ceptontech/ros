import argparse
import csv
from pathlib import Path
import numpy as np
from PIL import Image

real_height = 520

min_percentile = 1
max_percentile = 99

parser = argparse.ArgumentParser(
    description="Convert point cloud CSV intensity data to a grayscale PNG."
)
parser.add_argument(
    "-i",
    "--input",
    default="frame2time300.csv",
    help="input CSV path (default: frame2time300.csv)",
)
parser.add_argument(
    "-o",
    "--output",
    default="frame2time300_gray.png",
    help="output PNG path (default: frame2time300_gray.png)",
)
parser.add_argument(
    "--gamma",
    type=float,
    default=1.0,
    help="gamma correction value; must be greater than 0 (default: 1.0)",
)
args = parser.parse_args()

gamma = args.gamma
csv_path = Path(args.input)
output_path = Path(args.output)

if gamma <= 0:
    parser.error("--gamma must be greater than 0")

if not csv_path.exists():
    parser.error(f"input CSV not found: {csv_path}")

if not csv_path.is_file():
    parser.error(f"input path is not a file: {csv_path}")

if output_path.parent != Path(".") and not output_path.parent.exists():
    parser.error(f"output directory not found: {output_path.parent}")

points = []

with open(csv_path, newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)

    required_columns = {"channel_id", "intensity"}
    fieldnames = set(reader.fieldnames or [])
    missing_columns = sorted(required_columns - fieldnames)
    if missing_columns:
        parser.error(
            "input CSV must include columns: "
            + ", ".join(sorted(required_columns))
            + f" (missing: {', '.join(missing_columns)})"
        )

    x = 0
    prev_channel = None

    for row in reader:
        channel = int(row["channel_id"])
        intensity = float(row["intensity"])

        if prev_channel is not None and channel < prev_channel:
            x += 1

        y = channel

        points.append((x,y,intensity))

        prev_channel = channel

if not points:
    parser.error("input CSV has no point rows")

intensities = np.array([p[2] for p in points], dtype = np.float32)

lo = np.percentile(intensities, min_percentile)
hi = np.percentile(intensities, max_percentile)

if hi == lo:
    parser.error(f"intensity percentile range is zero: {lo} .. {hi}")

real_width = max(p[0] for p in points) + 1
image = np.zeros((real_height, real_width),dtype = np.uint8)

used_x_min = real_width
used_x_max = 0
used_y_min = real_height
used_y_max = 0
skipped = 0

for x,y,intensity in points:
    draw_x = real_width - 1 - x

    if draw_x < 0 or draw_x >= real_width or y < 0 or y >= real_height:
        skipped += 1
        continue

    gray = (intensity - lo) / (hi - lo)
    gray = max(0.0, min(1.0,gray))
    gray = gray ** gamma
    value = int(gray*255)

    image[y,draw_x] = max(image[y,draw_x],value)

    used_x_min = min(used_x_min, draw_x)
    used_x_max = max(used_x_max, draw_x)
    used_y_min = min(used_y_min, y)
    used_y_max = max(used_y_max, y)

img = Image.fromarray(image, mode="L")
img.save(output_path)


print(f"gamma: {gamma}")
print(f"saved: {output_path}")
print(f"image size: {real_width} x {real_height}")
print(f"used x: {used_x_min} .. {used_x_max}")
print(f"used y: {used_y_min} .. {used_y_max}")
print(f"skipped: {skipped}")
print(f"intensity percentile range: {lo} .. {hi}")
