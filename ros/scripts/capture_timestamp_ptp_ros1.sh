#!/usr/bin/env bash
# Build one timestamp/PTP configuration and capture one live PointCloud2 frame.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: capture_timestamp_ptp_ros1.sh --workspace PATH --mode MODE [options]

Builds cepton_ros with PTP enabled and the requested timestamp mode, starts the
ROS1 driver, and saves one PointCloud2 frame as CSV. It does not validate the
CSV contents.

Options:
  --workspace PATH      Catkin workspace containing cepton_ros (required)
  --mode MODE           relative, frame_offset, or absolute (required)
  --topic TOPIC         PointCloud2 topic (default: /cepton3/points)
  --info-topic TOPIC    INFZ-derived sensor-information topic
                        (default: /cepton3/sensor_information)
  --config PATH         YAML passed as publisher.launch's config_path
  --timeout SECONDS     Timeout while waiting for a point cloud (default: 15)
  --output-dir PATH     Directory for CSV and ROS logs
  --ros-setup PATH      ROS setup.bash (default: /opt/ros/noetic/setup.bash)
  -h, --help            Show this help
EOF
}

workspace=""
mode=""
topic="/cepton3/points"
info_topic="/cepton3/sensor_information"
config_path=""
timeout_seconds="15"
output_dir=""
ros_setup="/opt/ros/noetic/setup.bash"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --topic) topic="$2"; shift 2 ;;
    --info-topic) info_topic="$2"; shift 2 ;;
    --config) config_path="$2"; shift 2 ;;
    --timeout) timeout_seconds="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
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
  relative) cmake_mode="RELATIVE"; mode_name="relative" ;;
  frame_offset|frame-offset) cmake_mode="FRAME_OFFSET"; mode_name="frame_offset" ;;
  absolute) cmake_mode="ABSOLUTE"; mode_name="absolute" ;;
  *) echo "Unsupported mode: $mode" >&2; exit 2 ;;
esac

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup file not found: $ros_setup" >&2
  exit 2
fi
source "$ros_setup"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
capture_script="$script_dir/capture_1frame_ros1.py"
configuration="${mode_name}_ptp_on"

if [[ -z "$output_dir" ]]; then
  output_dir="$workspace/timestamp_test_results/${configuration}_$(date +%Y%m%dT%H%M%S)"
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

echo "Building TIMESTAMP_MODE=$cmake_mode WITH_PTP=ON in $workspace"
(
  cd "$workspace"
  catkin_make -DWITH_TS_CH_F=ON -DTIMESTAMP_MODE="$cmake_mode" -DWITH_PTP=ON
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

info_path="$output_dir/sensor_information.yaml"
echo "Checking time_sync_offset from INFZ via $info_topic"
if ! timeout "$timeout_seconds" rostopic echo -n 1 "$info_topic" >"$info_path"; then
  echo "ERROR: INFZ sensor information was not received from $info_topic within ${timeout_seconds}s." >&2
  exit 1
fi

time_sync_offset="$(awk '$1 == "time_sync_offset:" { print $2; exit }' "$info_path")"
if [[ ! "$time_sync_offset" =~ ^-?[0-9]+$ ]]; then
  echo "ERROR: time_sync_offset was not found in INFZ sensor information: $info_path" >&2
  exit 1
fi
if [[ "$time_sync_offset" == "0" ]]; then
  echo "ERROR: INFZ time_sync_offset is 0. ptp4l を起動してください。PTP 同期後に再実行してください。" >&2
  exit 1
fi
echo "INFZ time_sync_offset: $time_sync_offset us"

csv_path="$output_dir/pointcloud_${configuration}.csv"
echo "Capturing one frame from $topic to $csv_path"
python3 "$capture_script" \
  --topic "$topic" \
  --output "$csv_path" \
  --timeout "$timeout_seconds" \
  --include-header

echo "CAPTURED: $csv_path"
