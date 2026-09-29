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

## Stage B development

The OptiX backend now traces the prepared tunnel triangles with tiled concrete,
sparse metric cracks, moving strip illumination and pixel/exposure integration.
The operating speed is 0.2 m/s (20 rpm, 28.444 kHz triggers). The GUI imaging target
is RTF >= 0.6; improve precision within that constraint. Texture budgets are 2 GiB
on GPU and 1 GiB on CPU, plus a separate 768 MiB GUI texture allowance.
Bind an optical scene with `stage_b_optics --integrated` before capture.
Commands and remaining Stage B acceptance work: [docs/STAGE_B.md](docs/STAGE_B.md).
The current 2K-source / 1 mm bake passes short-run data checks but has not passed
visual-quality review. The proposed Wall 04 16K upgrade, lighting, crack and joint
corrections are documented in [the quality plan](docs/STAGE_B_QUALITY_PLAN.md);
B1 now provides native 16K source preparation, on-demand GPU tiles and optical
sample comparisons. Wall 04 did not pass the user's visual review; a
[source-to-render audit and native Concrete030 8K comparison](docs/STAGE_B_TEXTURE_AUDIT.md)
now separate source contrast from interpolation and seam feathering losses.
Crack/joint corrections and the new GUI scene remain pending.

The prepared local scene with visible concrete and the orange/white patent-based car
can be opened with `tools/run_gz_gui.sh`. It starts paused; Play starts capture.
This uses the existing private Mesa launcher from the adjacent 4WIDS_agv workspace.
