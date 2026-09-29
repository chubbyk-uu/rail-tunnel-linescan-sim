#!/usr/bin/env bash
# Run COMMAND with the isolated WSL OptiX runtime (adapted from 4WIDS_agv; see its
# docs/OPTIX_SETUP.md). Only the child process sees these paths; no driver files change.
# The inherited LD_LIBRARY_PATH is dropped on purpose (CUDA stubs must never be loaded
# at run time), so ROS or workspace setup scripts have to be sourced inside COMMAND.
set -euo pipefail
runtime="${SSB_OPTIX_RUNTIME:-${HOME}/opt/optix-runtime-610.57.04}"
if [[ $# -eq 0 ]]; then
  echo "Usage: bash tools/with_optix_runtime.sh COMMAND [ARG...]" >&2
  exit 2
fi
for library in libnvoptix.so.1 libnvidia-rtcore.so.610.57.04 libnvidia-gpucomp.so.610.57.04; do
  [[ -r "$runtime/$library" ]] || { echo "Missing runtime library: $runtime/$library" >&2; exit 1; }
done
exec env SSB_OPTIX_RUNTIME="$runtime" LD_LIBRARY_PATH="$runtime:/usr/lib/wsl/lib:/usr/local/cuda/lib64" "$@"
