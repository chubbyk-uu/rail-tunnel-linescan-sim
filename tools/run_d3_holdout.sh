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
refinement_args=()
if [[ ${SSB_ADAPTIVE_ATTITUDE:-0} == 1 ]]; then
  refinement_args=(--adaptive-attitude)
fi
if [[ ${SSB_SURFACE_RELIEF:-0} == 1 ]]; then
  refinement_args+=(--surface-relief)
fi
if [[ ${SSB_SLOW_TRANSLATION:-0} == 1 ]]; then
  refinement_args+=(--slow-translation)
fi
search_args=()
if [[ -n ${SSB_D2_Q_SHIFT_MM:-} ]]; then
  search_args=(--max-q-shift-mm "$SSB_D2_Q_SHIFT_MM")
fi
python3 -m ssb_tools.holdout_protocol declare --workspace "$repo" --demo "$demo" \
  --start "${3:-12}" --length "${4:-3}" --output "$evaluation/protocol.json" \
  --spacing-m "${SSB_D2_SPACING_M:-.2}" --height "${SSB_D2_HEIGHT:-512}" "${refinement_args[@]}" "${search_args[@]}"

# Re-enter the WSL runtime and source ROS inside it (the wrapper resets LD_LIBRARY_PATH).
bash "$repo/tools/with_optix_runtime.sh" bash -s -- "$repo" "$root" "$evaluation" "$demo" "${3:-12}" "${4:-3}" <<'SH'
set -eo pipefail
repo=$1; root=$2; evaluation=$3; demo=$4; start=$5; length=$6
source /opt/ros/jazzy/setup.bash
source "$repo/install/setup.bash"
set -u
worker_args=()
if [[ -n ${SSB_OFFLINE_WORKERS:-} ]]; then
  worker_args=(--workers "$SSB_OFFLINE_WORKERS")
fi
refinement_args=()
if [[ ${SSB_ADAPTIVE_ATTITUDE:-0} == 1 ]]; then
  refinement_args=(--adaptive-attitude)
fi
if [[ ${SSB_SURFACE_RELIEF:-0} == 1 ]]; then
  refinement_args+=(--surface-relief)
fi
if [[ ${SSB_SLOW_TRANSLATION:-0} == 1 ]]; then
  refinement_args+=(--slow-translation)
fi
search_args=()
if [[ -n ${SSB_D2_Q_SHIFT_MM:-} ]]; then
  search_args=(--max-q-shift-mm "$SSB_D2_Q_SHIFT_MM")
fi
bash "$repo/tools/run_wall_capture.sh" "$root/capture" "$start" "$length" "$demo" > "$evaluation/capture.log" 2>&1
"$repo/install/ssb_core/lib/ssb_core/ssb_render" --config "$root/capture/evaluation/config_source.yaml" \
  --session "$root/reimage" --poses "$root/capture/evaluation/pose_stream.bin" --batch-rows 333 > "$evaluation/reimage.log" 2>&1
# Failure stops this run; an incomplete or mismatched capture is never scored as acceptance.
python3 -m ssb_tools.validate_stage_b "$root/capture" --compare "$root/reimage" > "$evaluation/stage_b.log" 2>&1
python3 -m ssb_tools.raw_quality --session "$root/capture" --output "$root/raw_quality" > "$evaluation/raw_quality.log" 2>&1
python3 -m ssb_tools.initial_unroll --session "$root/capture" --calibration "$demo/calibration.json" --backend cuda --output "$root/unroll" > "$evaluation/unroll.log" 2>&1
# D2/D3 run once from staged public inputs under the private-input audit; no separate replay.
python3 -m ssb_tools.public_reconstruction --unroll "$root/unroll" --observable "$root/capture/config/observable_config.json" \
  --root "$root" --strict "${worker_args[@]}" --spacing-m "${SSB_D2_SPACING_M:-.2}" --height "${SSB_D2_HEIGHT:-512}" --attitude-spacing-m .02 \
  "${refinement_args[@]}" "${search_args[@]}" > "$evaluation/public_reconstruction.log" 2>&1
python3 -m ssb_tools.holdout_protocol verify --protocol "$evaluation/protocol.json" --root "$root" \
  --output "$evaluation/protocol_verification.json" > "$evaluation/protocol_verification.log" 2>&1
python3 -m ssb_tools.evaluate_global_geometry --session "$root/capture" --unroll "$root/unroll" \
  --trajectory "$root/fit" --output "$evaluation/geometry" --strict --spacing-m "${SSB_D2_SPACING_M:-.2}" \
  "${worker_args[@]}" > "$evaluation/geometry.log" 2>&1
python3 - "$evaluation/geometry/report.json" "$evaluation/protocol.json" <<'PY'
import hashlib, json, sys
from pathlib import Path
report = json.load(open(sys.argv[1]))
protocol = json.load(open(sys.argv[2]))
plan = Path(sys.argv[1]).parent/'sampling_plan.json'
assert report['sampling']['schema'] == protocol['sampling']['schema']
for key in ('spacing_q_m', 'phase_fractions', 'samples_across'):
    assert report['sampling'][key] == protocol['sampling'][key], key
assert hashlib.sha256(plan.read_bytes()).hexdigest() == report['sampling']['plan_sha256']
raise SystemExit(0 if all(g['status'] == 'pass' for g in report['gates'].values()) else 1)
PY
SH
