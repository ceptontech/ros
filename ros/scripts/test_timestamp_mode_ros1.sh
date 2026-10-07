#!/usr/bin/env bash
# Build one timestamp mode, capture one live frame, then validate its CSV.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: test_timestamp_mode_ros1.sh --workspace PATH --mode MODE [options]

Builds cepton_ros, starts the ROS1 driver, captures one PointCloud2 frame, and
validates the resulting CSV. MODE is relative, frame_offset, or absolute.

Options:
  --workspace PATH             Catkin workspace containing cepton_ros (required)
  --mode MODE                  relative, frame_offset, or absolute (required)
  --topic TOPIC                PointCloud2 topic (default: /cepton3/points)
  --config PATH                YAML passed as publisher.launch's config_path
  --timeout SECONDS            Timeout while waiting for a point cloud (default: 15)
  --output-dir PATH            Directory for CSV, reports, and ROS logs
  --min-max-offset-us VALUE    Require this minimum maximum offset
  --allow-trimmed-frame        Do not require an offset of zero in the CSV
  --ros-setup PATH             ROS setup.bash (default: /opt/ros/noetic/setup.bash)
  -h, --help                   Show this help
EOF
}

workspace=""
mode=""
topic="/cepton3/points"
config_path=""
timeout_seconds="15"
output_dir=""
min_max_offset_us=""
allow_trimmed_frame=false
ros_setup="/opt/ros/noetic/setup.bash"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --topic) topic="$2"; shift 2 ;;
    --config) config_path="$2"; shift 2 ;;
    --timeout) timeout_seconds="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --min-max-offset-us) min_max_offset_us="$2"; shift 2 ;;
    --allow-trimmed-frame) allow_trimmed_frame=true; shift ;;
    --ros-setup) ros_setup="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$workspace" || -z "$mode" ]]; then
  echo "--workspace and --mode are required" >&2
  usage >&2
  exit 2
fi

workspace="$(cd "$workspace" && pwd)"
if [[ ! -f "$workspace/src/CMakeLists.txt" && ! -d "$workspace/src/cepton_ros" ]]; then
  echo "Not a catkin workspace: $workspace" >&2
  exit 2
fi

case "${mode,,}" in
  relative) cmake_mode="RELATIVE"; verify_mode="relative" ;;
  frame_offset|frame-offset) cmake_mode="FRAME_OFFSET"; verify_mode="frame_offset" ;;
  absolute) cmake_mode="ABSOLUTE"; verify_mode="absolute" ;;
  *) echo "Unsupported mode: $mode" >&2; exit 2 ;;
esac

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup file not found: $ros_setup" >&2
  exit 2
fi
source "$ros_setup"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
capture_script="$script_dir/capture_1frame_ros1.py"
verify_script="$script_dir/verify_timestamp_csv.py"

if [[ -z "$output_dir" ]]; then
  output_dir="$workspace/timestamp_test_results/${verify_mode}_$(date +%Y%m%dT%H%M%S)"
fi
mkdir -p "$output_dir"

manager_pid=""
publisher_pid=""
roscore_pid=""

cleanup() {
  local exit_code=$?
  for pid in "$publisher_pid" "$manager_pid" "$roscore_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  exit "$exit_code"
}
trap cleanup EXIT INT TERM

echo "Building TIMESTAMP_MODE=$cmake_mode in $workspace"
(
  cd "$workspace"
  catkin_make -DWITH_TS_CH_F=ON -DTIMESTAMP_MODE="$cmake_mode"
)
source "$workspace/devel/setup.bash"

if ! rosnode list >/dev/null 2>&1; then
  echo "Starting roscore"
  roscore >"$output_dir/roscore.log" 2>&1 &
  roscore_pid=$!
  for _ in $(seq 1 50); do
    rosnode list >/dev/null 2>&1 && break
    sleep 0.1
  done
  rosnode list >/dev/null 2>&1 || {
    echo "roscore did not become ready; see $output_dir/roscore.log" >&2
    exit 1
  }
fi

echo "Starting nodelet manager"
roslaunch cepton_ros manager.launch >"$output_dir/manager.log" 2>&1 &
manager_pid=$!
sleep 1

echo "Starting Cepton publisher"
if [[ -n "$config_path" ]]; then
  roslaunch cepton_ros publisher.launch "config_path:=$config_path" >"$output_dir/publisher.log" 2>&1 &
else
  roslaunch cepton_ros publisher.launch >"$output_dir/publisher.log" 2>&1 &
fi
publisher_pid=$!

csv_path="$output_dir/frame.csv"
report_path="$output_dir/report.json"
echo "Capturing one frame from $topic"
python3 "$capture_script" \
  --topic "$topic" \
  --output "$csv_path" \
  --timeout "$timeout_seconds" \
  --include-header

verify_args=(--input "$csv_path" --mode "$verify_mode" --report "$report_path")
if [[ -n "$min_max_offset_us" ]]; then
  verify_args+=(--min-max-offset-us "$min_max_offset_us")
fi
if [[ "$allow_trimmed_frame" == true ]]; then
  verify_args+=(--allow-trimmed-frame)
fi

echo "Verifying $csv_path"
python3 "$verify_script" "${verify_args[@]}"
echo "PASS: results saved in $output_dir"
