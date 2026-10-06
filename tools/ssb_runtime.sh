#!/usr/bin/env bash
# Run COMMAND in the GPU/graphics runtime this host needs, so every launcher, test
# and production self-check shares one decision (DESIGN.md §12, docs/deployment).
#
#   bash tools/ssb_runtime.sh [--gui] COMMAND [ARG...]
#
# SSB_RUNTIME=wsl|native|auto (default auto: WSL when the kernel release names
# Microsoft). wsl: isolated OptiX user-space runtime (tools/with_optix_runtime.sh);
# --gui also selects d3d12 Mesa through tools/with_mesa_runtime.py. native: the
# system NVIDIA/OpenGL stack, environment unchanged. An undecidable host is an
# error, never a guess. The selected mode is exported as SSB_RUNTIME_MODE.
set -euo pipefail
gui=0
if [[ ${1:-} == --gui ]]; then gui=1; shift; fi
if [[ $# -eq 0 ]]; then
  echo "Usage: bash tools/ssb_runtime.sh [--gui] COMMAND [ARG...]" >&2
  exit 2
fi
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
mode=${SSB_RUNTIME:-auto}
if [[ $mode == auto ]]; then
  release=${SSB_OSRELEASE_FILE:-/proc/sys/kernel/osrelease}
  if [[ ! -r $release ]]; then
    echo "Cannot read $release to detect the runtime; set SSB_RUNTIME=wsl or native" >&2
    exit 2
  fi
  if grep -qi microsoft "$release"; then mode=wsl; else mode=native; fi
fi
case $mode in
  wsl)
    export SSB_RUNTIME_MODE=wsl
    if (( gui )); then
      mesa=${SSB_MESA_WRAPPER:-$here/with_mesa_runtime.py}
      [[ -f $mesa ]] || { echo "Private Mesa launcher missing: $mesa" >&2; exit 2; }
      export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA QT_QPA_PLATFORM=xcb
      exec python3 "$mesa" bash "$here/with_optix_runtime.sh" "$@"
    fi
    exec bash "$here/with_optix_runtime.sh" "$@"
    ;;
  native)
    export SSB_RUNTIME_MODE=native
    exec "$@"
    ;;
  *)
    echo "SSB_RUNTIME must be wsl, native or auto, not: $mode" >&2
    exit 2
    ;;
esac
