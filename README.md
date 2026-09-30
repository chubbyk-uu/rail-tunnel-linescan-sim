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
on GPU and 1 GiB on CPU, plus a separate 1280 MiB GUI texture allowance.
Bind an optical scene with `stage_b_optics --integrated` before capture.
Commands and remaining Stage B acceptance work: [docs/STAGE_B.md](docs/STAGE_B.md).
The accepted Concrete034 runtime surface, refined cracks and filled joints are retained.
The current demo uses rigid wheel/rail friction contact, two front motors, two rear
encoders and four side guide bearings. Scanning follows estimated rear-wheel travel,
so calibrated wheel diameter errors change the real scan pitch. Four chassis work lights, offset sideways/down by 15 degrees with 90-degree
full cones, cast shadows and avoid the nominal instantaneous camera stripe; reflected acquisition light is an explicit weak approximation.
The work lamp faces and scanning COB lens emit warm white in the preview. A GUI-only
lens flare follows the strongest visible source, with direction and occlusion checks;
it does not change OptiX illumination or raw images.

Open the prepared 3 m contact demo with `tools/run_gz_gui.sh` (paused initially).
Play includes 2 s of settling before capture. The default assets are in
`local_data/stage_b/gui_contact_glare90_v9/`. To derive another demo from the accepted
local assets, use `python3 tools/prepare_contact_demo.py --output NEW_DIRECTORY`;
then pass its `capture.yaml` and `world/world.sdf` as the second/third GUI arguments.
Old ideal configurations/worlds remain usable when supplied explicitly.

The final GUI run at 0.2 m/s and 64 crack samples achieved imaging RTF 0.994,
284445 rows over 3 m, no missing rows in the valid region and byte-identical replay.
A separate 20 m contact-only run passed stability checks; full 20 m reconstruction
is still subsequent work. See [the acceptance record](docs/STAGE_B.md).

This uses the existing private Mesa launcher from the adjacent 4WIDS_agv workspace.
