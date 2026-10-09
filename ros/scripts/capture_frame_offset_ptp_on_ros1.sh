#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$script_dir/capture_timestamp_ptp_ros1.sh" --mode frame_offset "$@"
