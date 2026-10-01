#!/usr/bin/env bash
# RViz remains open after capture; one manager owns all simulation processes.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
export GZ_PARTITION="ssb_mission_$$" GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA QT_QPA_PLATFORM=xcb
log_dir="$repo/local_data/mission_logs/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$log_dir"
echo "RViz task panel; logs: $log_dir"
python3 "$repo/tools/with_mesa_runtime.py" bash "$repo/tools/with_optix_runtime.sh" bash -c '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  source "$1/install/setup.bash"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
  export GZ_GUI_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
  cd "$1"
  shift
  manager=""; viewer=""
  cleanup() {
    if [[ -n "$manager" ]]; then kill -INT -- "-$manager" 2>/dev/null || true; wait "$manager" 2>/dev/null || true; fi
    if [[ -n "$viewer" ]]; then kill -INT -- "-$viewer" 2>/dev/null || true; wait "$viewer" 2>/dev/null || true; fi
  }
  trap cleanup EXIT
  logs=$1; shift
  setsid python3 -m ssb_tools.mission_manager "$@" > "$logs/manager.log" 2>&1 & manager=$!
  setsid rviz2 -d install/ssb_rviz/share/ssb_rviz/config/mission.rviz --ros-args -p use_sim_time:=true > "$logs/rviz.log" 2>&1 & viewer=$!
  while kill -0 "$viewer" 2>/dev/null; do
    kill -0 "$manager" 2>/dev/null || { echo "Task manager exited; inspect manager.log"; exit 2; }
    sleep .5
  done
  wait "$viewer"; viewer=""
' _ "$repo" "$log_dir" "$@" > "$log_dir/launcher.log" 2>&1
