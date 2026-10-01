#!/usr/bin/env bash
# Stage B GUI, initially paused. Play starts encoder capture; close drains it.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
session=$(realpath -m "${1:-$repo/sessions/gui_$(date +%Y%m%d_%H%M%S)}")
demo="$repo/local_data/stage_b/contact_demo"
config=$(realpath "${2:-$demo/capture.yaml}")
world=$(realpath "${3:-$(dirname "$config")/world/world.sdf}")
calibration=${SSB_OPTICAL_CALIBRATION:-$(dirname "$config")/calibration.json}
"$repo/install/ssb_core/lib/ssb_core/ssb_optical_identity" --config "$config" --calibration "$calibration" >/dev/null
[[ ! -e "$session" ]] || { echo "Session already exists: $session" >&2; exit 2; }
mesa_wrapper=${SSB_MESA_WRAPPER:-$repo/tools/with_mesa_runtime.py}
[[ -f "$mesa_wrapper" ]] || { echo "Private Mesa launcher missing: $mesa_wrapper" >&2; exit 2; }
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA QT_QPA_PLATFORM=xcb
export GZ_PARTITION="${GZ_PARTITION:-ssb_gui_$$}" SSB_CONFIG="$config" SSB_SESSION="$session" SSB_WORLD="$world"
log_dir="$repo/local_data/gui_logs/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$log_dir"
export SSB_GUI_LOG_DIR="$log_dir"
echo "World: $world"
echo "Paused: click Play to capture. Session: $session"
echo "Gazebo log: $log_dir/gazebo.log"
python3 "$mesa_wrapper" bash "$repo/tools/with_optix_runtime.sh" bash -c '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  source "$1/install/setup.bash"
  set -u
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
  export GZ_GUI_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
  # Manage server and GUI separately: closing the GUI must drain/stop the server
  # before checking the raw session. Only these owned process groups are stopped.
  server_pid=""
  gui_pid=""
  cleanup() {
    for pid in "$gui_pid" "$server_pid"; do
      if [[ -n "$pid" ]]; then kill -INT -- "-$pid" 2>/dev/null || true; fi
    done
    for pid in "$gui_pid" "$server_pid"; do
      if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
    done
  }
  trap cleanup EXIT
  setsid gz sim -s -v 3 "$SSB_WORLD" > "$SSB_GUI_LOG_DIR/server.log" 2>&1 &
  server_pid=$!
  setsid gz sim -g -v 3 --gui-config "$1/src/ssb_gazebo/worlds/stage_b_gui.config" \
    > "$SSB_GUI_LOG_DIR/gui.log" 2>&1 &
  gui_pid=$!
  while kill -0 "$gui_pid" 2>/dev/null; do
    if ! kill -0 "$server_pid" 2>/dev/null; then
      echo "Gazebo server exited before GUI; inspect server.log" >&2
      exit 2
    fi
    sleep 0.5
  done
  wait "$gui_pid"
  gui_pid=""
  kill -INT -- "-$server_pid"
  wait "$server_pid"
  server_pid=""
  trap - EXIT
' _ "$repo" > "$log_dir/gazebo.log" 2>&1
if [[ -d "$session" ]]; then
  python3 "$repo/tools/check_session.py" "$session"
  echo "Raw capture saved: $session. Apply optical calibration separately before stitching."
fi
