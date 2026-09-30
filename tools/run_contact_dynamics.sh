#!/usr/bin/env bash
# Physics-only contact validation. No OptiX startup and no camera images.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
[[ $# == 3 ]] || { echo 'usage: run_contact_dynamics.sh SESSION CONFIG WORLD' >&2; exit 2; }
export SSB_SESSION=$(realpath -m "$1") SSB_CONFIG=$(realpath "$2") SSB_WORLD=$(realpath "$3")
export SSB_DYNAMICS_ONLY=1 GZ_PARTITION="ssb_contact_$$"
iterations=$(python3 - "$SSB_CONFIG" <<'PY'
import sys,yaml,math
c=yaml.safe_load(open(sys.argv[1]));print(math.ceil((c['contact'].get('settle_s',2)+c['motion']['profile'][-1][0])/c['motion']['sample_period_s'])+2)
PY
)
bash "$repo/tools/with_optix_runtime.sh" bash -c '
 source /opt/ros/jazzy/setup.bash
 source "$1/install/setup.bash"
 export GZ_SIM_SYSTEM_PLUGIN_PATH="$1/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
 gz sim -s -r -v 3 --iterations "$2" "$SSB_WORLD"
' _ "$repo" "$iterations"
python3 - "$SSB_SESSION" <<'PY'
import sys,json
from pathlib import Path
s=json.loads(Path(sys.argv[1]+'_dynamics/summary.json').read_text())
assert s['complete'];print(json.dumps(s))
PY
