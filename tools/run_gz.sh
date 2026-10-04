#!/usr/bin/env bash
# Headless Gazebo capture: tools/run_gz.sh SESSION_DIR [CONFIG] [extra gz sim args...]
# Runs the stage world for the config's motion profile; the server exits only after
# the imaging pipeline has written every row.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
[[ $# -ge 1 ]] || { echo "usage: tools/run_gz.sh SESSION_DIR [CONFIG] [gz args...]" >&2; exit 2; }
session=$(realpath -m "$1"); shift
config=$(realpath "${1:-$repo/src/ssb_core/config/stage_a.yaml}"); [[ $# -ge 1 ]] && shift
world=$(realpath "${SSB_WORLD:-$repo/install/ssb_gazebo/share/ssb_gazebo/worlds/stage_a.sdf}")
# The plugin refuses a world whose physics step differs from motion.sample_period_s,
# so the iteration count below always covers the planned motion.
iterations=$(python3 -c "import yaml,sys,math; c=yaml.safe_load(open(sys.argv[1])); m=c['motion']; settle=c.get('contact',{}).get('settle_s',2) if c.get('contact',{}).get('enabled') else 0; end=m.get('distance_stop',{}).get('timeout_s',m['profile'][-1][0]); print(math.ceil((end+settle)/m['sample_period_s']-1e-9)+2)" "$config")
exec bash "$repo/tools/with_optix_runtime.sh" bash -c '
  source /opt/ros/jazzy/setup.bash && source "$0/install/setup.bash"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$0/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
  export SSB_CONFIG="$1" SSB_SESSION="$2" SSB_WORLD="$4"
  # Rails, wheels and springs in the world must be exactly the configured truth.
  python3 -m ssb_tools.physical_world check --config "$SSB_CONFIG" --world "$SSB_WORLD" >/dev/null || { echo "physical world check failed; see: python3 -m ssb_tools.physical_world check --config $SSB_CONFIG --world $SSB_WORLD" >&2; exit 2; }
  gz sim -s -r -v 3 --iterations "$3" "${@:5}" "$SSB_WORLD" || exit $?
  python3 "$0/tools/check_session.py" "$SSB_SESSION"
' "$repo" "$config" "$session" "$iterations" "$world" "$@"
