# Subway scan bot simulation

Tunnel inspection robot with a rotating line-scan camera: Gazebo dynamics, OptiX line
imaging, and reconstruction of the unrolled tunnel wall. Design and status: [DESIGN.md](DESIGN.md).

## Build and test

Requires ROS 2 Jazzy (Gazebo Harmonic), CUDA 12.8, OptiX SDK 9.1 at `~/opt/optix-sdk-9.1.0`
and, on WSL, the isolated OptiX runtime described in 4WIDS_agv `docs/OPTIX_SETUP.md`.
The OptiX backend is mandatory; configuration fails without it.

```bash
source /opt/ros/jazzy/setup.bash
export COLCON_DEFAULTS_FILE=$PWD/colcon_defaults.yaml
colcon build && colcon test && colcon test-result --all
```

GPU tests run through `tools/with_optix_runtime.sh`, which drops the inherited
`LD_LIBRARY_PATH`; anything ROS-dependent must be sourced inside the wrapped command.

## Stage A runs

```bash
# Gazebo, headless; the process exits after every row is on disk.
tools/run_gz.sh sessions/gz_a

# Kinematic pose source, or re-imaging an archived pose stream without Gazebo.
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_render \
  --config src/ssb_core/config/stage_a.yaml --session sessions/kin --realtime
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_render \
  --config src/ssb_core/config/stage_a.yaml --session sessions/rerender \
  --poses sessions/gz_a/evaluation/pose_stream.bin --batch-rows 333

# Acceptance report (evaluation/reports/stage_a.json); --compare checks byte identity.
source install/setup.bash
ros2 run ssb_tools validate_stage_a sessions/gz_a --compare sessions/rerender
```

A stage A session is about 0.9 GB (`sessions/` is not tracked).
