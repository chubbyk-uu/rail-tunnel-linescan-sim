#!/usr/bin/env bash
# Build before declaring/capturing; validate independent reimaging before D1/D2/D3.
set -euo pipefail
[[ $# -ge 2 && $# -le 5 ]] || { echo 'usage: run_d3_holdout.sh ROOT EVALUATION [START_M=12] [LENGTH_M=3] [DEMO]' >&2; exit 2; }
repo=$(cd "$(dirname "$0")/.." && pwd)
root=$(realpath -m "$1")
evaluation=$(realpath -m "$2")
demo=$(realpath "${5:-$repo/local_data/stage_b/contact_demo}")
[[ ! -e "$root" && ! -e "$evaluation" ]] || { echo 'use fresh output directories' >&2; exit 2; }
cd "$repo"
[[ -z $(git status --porcelain) ]] || { echo 'commit changes before holdout capture' >&2; exit 2; }
[[ "$evaluation" == */evaluation/* ]] || { echo 'private protocol must be inside evaluation/' >&2; exit 2; }
mkdir -p "$evaluation"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export COLCON_DEFAULTS_FILE="$repo/colcon_defaults.yaml"
# Full build updates the core source stamp even after documentation-only commits.
colcon build > "$evaluation/build.log" 2>&1
set +u
source "$repo/install/setup.bash"
set -u
python3 -m ssb_tools.holdout_protocol declare --workspace "$repo" --demo "$demo" \
  --start "${3:-12}" --length "${4:-3}" --output "$evaluation/protocol.json"

# Re-enter the WSL runtime and source ROS inside it (the wrapper resets LD_LIBRARY_PATH).
bash "$repo/tools/with_optix_runtime.sh" bash -s -- "$repo" "$root" "$evaluation" "$demo" "${3:-12}" "${4:-3}" <<'SH'
set -eo pipefail
repo=$1; root=$2; evaluation=$3; demo=$4; start=$5; length=$6
source /opt/ros/jazzy/setup.bash
source "$repo/install/setup.bash"
set -u
bash "$repo/tools/run_wall_capture.sh" "$root/capture" "$start" "$length" "$demo" > "$evaluation/capture.log" 2>&1
"$repo/install/ssb_core/lib/ssb_core/ssb_render" --config "$root/capture/evaluation/config_source.yaml" \
  --session "$root/reimage" --poses "$root/capture/evaluation/pose_stream.bin" --batch-rows 333 > "$evaluation/reimage.log" 2>&1
# Failure stops this run; an incomplete or mismatched capture is never scored as acceptance.
python3 -m ssb_tools.validate_stage_b "$root/capture" --compare "$root/reimage" > "$evaluation/stage_b.log" 2>&1
python3 -m ssb_tools.raw_quality --session "$root/capture" --output "$root/raw_quality" > "$evaluation/raw_quality.log" 2>&1
python3 -m ssb_tools.initial_unroll --session "$root/capture" --calibration "$demo/calibration.json" --backend cuda --output "$root/unroll" > "$evaluation/unroll.log" 2>&1
python3 -m ssb_tools.match_bands --unroll "$root/unroll" --spacing-m .2 --output "$root/matches" > "$evaluation/matches.log" 2>&1
python3 -m ssb_tools.optimize_bands --unroll "$root/unroll" --matches "$root/matches" --observable "$root/capture/config/observable_config.json" \
  --observed-knots --attitude-spacing-m .02 --output "$root/fit" > "$evaluation/fit.log" 2>&1
python3 "$repo/tools/test_global_optimization.py" --unroll "$root/unroll" --matches "$root/matches" --observable "$root/capture/config/observable_config.json" \
  --reference "$root/fit" --output "$root/public_replay" > "$evaluation/public_replay.log" 2>&1
python3 -m ssb_tools.holdout_protocol verify --protocol "$evaluation/protocol.json" --root "$root" \
  --output "$evaluation/protocol_verification.json" > "$evaluation/protocol_verification.log" 2>&1
python3 -m ssb_tools.evaluate_global_geometry --session "$root/capture" --unroll "$root/unroll" \
  --trajectory "$root/fit" --output "$evaluation/geometry" > "$evaluation/geometry.log" 2>&1
python3 - "$evaluation/geometry/report.json" <<'PY'
import json, sys
report = json.load(open(sys.argv[1]))
raise SystemExit(0 if all(g['status'] == 'pass' for g in report['gates'].values()) else 1)
PY
SH
