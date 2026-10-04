#!/usr/bin/env bash
# Prepare buffered travel, save raw only, then check nominal geometric wall coverage.
set -euo pipefail
[[ $# -ge 3 && $# -le 4 ]] || { echo 'usage: run_wall_capture.sh SESSION TARGET_START_M TARGET_LENGTH_M [DEMO]' >&2; exit 2; }
repo=$(cd "$(dirname "$0")/.." && pwd)
session=$(realpath -m "$1")
demo=$(realpath "${4:-$repo/local_data/stage_b/contact_demo}")
inputs="${session}_inputs"
[[ ! -e "$session" && ! -e "$inputs" ]] || { echo 'capture and input directories must be fresh' >&2; exit 2; }
mkdir -p "$(dirname "$session")"
set +u # ROS setup scripts inspect optional environment variables.
source /opt/ros/jazzy/setup.bash
source "$repo/install/setup.bash"
set -u
scale_args=()
if [[ ${SSB_RELATIVE_ENCODER_SCALE:-0} == 1 ]]; then
  scale_args=(--relative-encoder-scale)
fi
python3 -m ssb_tools.mission_plan --demo "$demo" --output "$inputs" \
  --target-start-m "$2" --target-length-m "$3" "${scale_args[@]}" > "${session}_plan.log" 2>&1
SSB_WORLD="$inputs/world.sdf" bash "$repo/tools/run_gz.sh" "$session" "$inputs/capture.yaml"
python3 -m ssb_tools.wall_coverage --session "$session" --calibration "$demo/calibration.json" \
  --output "$session/reconstruction/coverage"
