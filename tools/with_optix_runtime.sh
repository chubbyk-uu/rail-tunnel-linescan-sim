#!/usr/bin/env bash
# Run COMMAND with the isolated WSL OptiX runtime (adapted from 4WIDS_agv; see its
# docs/OPTIX_SETUP.md). Only the child process sees these paths; no driver files change.
# The inherited LD_LIBRARY_PATH is dropped on purpose (CUDA stubs must never be loaded
# at run time), so ROS or workspace setup scripts have to be sourced inside COMMAND.
# Launchers call tools/ssb_runtime.sh, which uses this wrapper only on WSL.
#
# The runtime directory is SSB_OPTIX_RUNTIME, else ~/opt/optix-runtime-$SSB_OPTIX_DRIVER_VERSION,
# else the single ~/opt/optix-runtime-* directory. The driver version is
# SSB_OPTIX_DRIVER_VERSION, else read from the one libnvidia-rtcore.so.<version>
# present. Zero or several candidates are an error: components of different driver
# packages must never be mixed.
set -euo pipefail
if [[ $# -eq 0 ]]; then
  echo "Usage: bash tools/with_optix_runtime.sh COMMAND [ARG...]" >&2
  exit 2
fi
version=${SSB_OPTIX_DRIVER_VERSION:-}
runtime=${SSB_OPTIX_RUNTIME:-}
if [[ -z $runtime ]]; then
  if [[ -n $version ]]; then
    runtime="$HOME/opt/optix-runtime-$version"
  else
    shopt -s nullglob
    candidates=("$HOME"/opt/optix-runtime-*/)
    shopt -u nullglob
    if [[ ${#candidates[@]} -ne 1 ]]; then
      echo "Expected one ~/opt/optix-runtime-* directory, found ${#candidates[@]}; set SSB_OPTIX_RUNTIME" >&2
      exit 1
    fi
    runtime=${candidates[0]%/}
  fi
fi
if [[ -z $version ]]; then
  shopt -s nullglob
  rtcore=("$runtime"/libnvidia-rtcore.so.*)
  shopt -u nullglob
  if [[ ${#rtcore[@]} -ne 1 ]]; then
    echo "Expected one libnvidia-rtcore.so.<version> in $runtime, found ${#rtcore[@]}; set SSB_OPTIX_DRIVER_VERSION" >&2
    exit 1
  fi
  version=${rtcore[0]##*/libnvidia-rtcore.so.}
fi
for library in libnvoptix.so.1 "libnvidia-rtcore.so.$version" "libnvidia-gpucomp.so.$version"; do
  [[ -r "$runtime/$library" ]] || { echo "Missing runtime library: $runtime/$library" >&2; exit 1; }
done
exec env SSB_OPTIX_RUNTIME="$runtime" SSB_OPTIX_DRIVER_VERSION="$version" \
  LD_LIBRARY_PATH="$runtime:/usr/lib/wsl/lib:/usr/local/cuda/lib64" "$@"
