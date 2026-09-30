#!/usr/bin/env bash
# Stage B GUI, initially paused. Play starts encoder capture; close drains it.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
session=$(realpath -m "${1:-$repo/sessions/gui_$(date +%Y%m%d_%H%M%S)}")
config=$(realpath "${2:-$repo/local_data/stage_b/gui_optics_v11/capture.yaml}")
world=$(realpath "${3:-$repo/local_data/stage_b/gui_strip_shadow_final_v10/world/world.sdf}")
[[ ! -e "$session" ]] || { echo "Session already exists: $session" >&2; exit 2; }
mesa_wrapper=${SSB_MESA_WRAPPER:-$repo/../4WIDS_agv/tools/with_mesa_runtime.py}
[[ -f "$mesa_wrapper" ]] || { echo "Private Mesa launcher missing: $mesa_wrapper" >&2; exit 2; }
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA QT_QPA_PLATFORM=xcb
export GZ_PARTITION="ssb_gui_$$" SSB_CONFIG="$config" SSB_SESSION="$session" SSB_WORLD="$world"
log_dir="$repo/local_data/gui_logs/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$log_dir"
echo "World: $world"
echo "Paused: click Play to capture. Session: $session"
echo "Gazebo log: $log_dir/gazebo.log"
python3 "$mesa_wrapper" bash "$repo/tools/with_optix_runtime.sh" bash -c '
  source /opt/ros/jazzy/setup.bash
  source "$1/install/setup.bash"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
  export GZ_GUI_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
  exec gz sim -v 3 --gui-config "$1/src/ssb_gazebo/worlds/stage_b_gui.config" "$SSB_WORLD"
' _ "$repo" > "$log_dir/gazebo.log" 2>&1
if [[ -d "$session" ]]; then
  python3 "$repo/tools/check_session.py" "$session"
  calibration=${SSB_OPTICAL_CALIBRATION:-$repo/local_data/stage_b/optical_bench_v2/calibration_v2.json}
  echo "Applying measured optical calibration: $calibration"
  PYTHONPATH="$repo/src/ssb_tools${PYTHONPATH:+:$PYTHONPATH}" python3 -m ssb_tools.optical_calibration apply \
    --session "$session" --calibration "$calibration" --output "$session/processed/optical"
fi
