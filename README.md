# Subway scan bot simulation

Simulation of a tunnel inspection robot with a rotating line-scan camera. Gazebo runs the
vehicle dynamics, a custom OptiX backend images every encoder-triggered line, and later
stages reconstruct the unrolled tunnel wall.

Status (2026-10-02): stage A (geometry and timing) is accepted. Stage B (scene,
lighting, rail contact, lens calibration) has a validated 3 m capture and a user-accepted
visual baseline ([review record](docs/STAGE_B.md#71-画面复核与基线冻结2026-10-01)).
The RViz mission UI is implemented ([usage](docs/STAGE_B.md#10-rviz-与任务控制));
stage C has accepted a full 20 m raw capture with Gazebo and RViz open, public-only
nominal coverage checks and byte-identical replay ([results](docs/STAGE_C.md#6-完整-20-m-采集验收2026-10-02)).
Stitching and global optimization are still pending.

## Documentation

| Document | Contents |
|---|---|
| [DESIGN.md](DESIGN.md) | Design specification: goals, parameters, geometry, timing, optics, reconstruction, stage plan |
| [docs/STAGE_B.md](docs/STAGE_B.md) | Current stage B scene and assets, commands, calibration, acceptance results, known limits |
| [docs/STAGE_C.md](docs/STAGE_C.md) | Wall target planning, public-only coverage, short capture verification and scale benchmark |
| [docs/STAGE_D.md](docs/STAGE_D.md) | Native optical correction, initial cylinder unroll, independent bands and source provenance |
| [docs/DEVELOPMENT_RULES.md](docs/DEVELOPMENT_RULES.md) | Development rules learned from the neighbouring projects |
| [docs/REPAIR_PLAN_2026-10-02.md](docs/REPAIR_PLAN_2026-10-02.md) | Pending review fixes, GUI strip-light occlusion checks and WSL I/O constraints |
| [docs/history/](docs/history/README.md) | Archived development logs, texture study, quality plan, review record, design v0.8 |

## Build and test

Requires ROS 2 Jazzy (Gazebo Harmonic), CUDA 12.8, OptiX SDK 9.1 at `~/opt/optix-sdk-9.1.0`,
and on WSL the isolated OptiX runtime described in 4WIDS_agv `docs/OPTIX_SETUP.md`.
The OptiX backend is mandatory; configuration fails without it.

```bash
source /opt/ros/jazzy/setup.bash
export COLCON_DEFAULTS_FILE=$PWD/colcon_defaults.yaml
colcon build && colcon test && colcon test-result --all
```

GPU tests run through `tools/with_optix_runtime.sh`, which drops the inherited
`LD_LIBRARY_PATH`; anything ROS-dependent must be sourced inside the wrapped command.
Rebuild before collecting after any new commit, including documentation-only commits:
capture provenance checks both the Git revision and the source tree fingerprint.
The real Gazebo server regression (both plugins, stage A default command, assembly
mismatch rejection) runs separately:
`python3 tools/test_gazebo_plugins.py --output /tmp/ssb_plugin_regression_NEW`.

## Stage A

```bash
# Use fresh output directories; sessions/gz_a is the retained acceptance baseline.
# Gazebo, headless; the process exits after every row is on disk.
tools/run_gz.sh sessions/gz_a_NEW

# Kinematic pose source, or re-imaging an archived pose stream without Gazebo.
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_render \
  --config src/ssb_core/config/stage_a.yaml --session sessions/kin_NEW --realtime
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_render \
  --config src/ssb_core/config/stage_a.yaml --session sessions/rerender_NEW \
  --poses sessions/gz_a_NEW/evaluation/pose_stream.bin --batch-rows 333

# Acceptance report (evaluation/reports/stage_a.json); --compare checks byte identity.
source install/setup.bash
ros2 run ssb_tools validate_stage_a sessions/gz_a_NEW --compare sessions/rerender_NEW
```

Choose other fresh names if the `_NEW` directories already exist; capture refuses to overwrite them.
A stage A session is about 0.9 GB. Stage A has no persistent `truth.optical_key`, so its
`optical_signature` changes on every run and cannot be compared across runs (DESIGN §8.4).

## Stage B demo

For the RViz mission panel (start position, travel distance, start/pause/resume/stop),
run `tools/run_mission.sh` after building all packages. The panel uses English, requires
the configured inspection range (20 m by default), at least 1 m of travel, and shows a low-rate raw image thumbnail at the lower left, with mission controls on the right.
Add `--gz-gui` for both viewers.
Each task creates a fresh session; closing RViz drains raw capture. Optical correction is
run separately before stitching.

```bash
tools/run_gz_gui.sh            # opens local_data/stage_b/contact_demo, paused; press Play
```

The demo is a 3 m run on rigid wheel/rail contact (2 s settling, then 0.2 m/s, 20 rpm,
28.444 kHz line triggers). Capture commits raw blocks and metadata durably at
shutdown, preserving per-block read-back hashes; no per-row sync is used. The head starts pointing straight down, rotates to the
right lower gate to begin capture, then scans through the top to the left lower gate.
The launcher checks the optical calibration before starting
Gazebo. After the GUI closes, it waits for all raw rows to reach disk and checks the
session. Dark/flat/geometric correction is run separately before stitching, writing
to `SESSION/processed/optical/` without changing the raw data.
Optional rail irregularity — vertical (flat / 2 mm / 5 mm, Beijing Subway spectrum shape) and
cross-level/twist (0 / 2 / 4 mm) — and polyurethane wheel compliance are generated into the world with
`tools/prepare_contact_demo.py` (`--track-chord-mm`, `--track-cross-level-mm`, `--wheel-deflection-mm`);
the default demo uses the 2 mm vertical and 2 mm cross-level tiers and 0.2 mm wheel deflection.
In contact mode the odometry encoders sit on two spring-loaded 80 mm measuring wheels behind the rear axle
(one per rail), so running-wheel lift on a twisted track does not reach the odometer.
Details, asset regeneration, replay and validation: [docs/STAGE_B.md](docs/STAGE_B.md).

## Data

`local_data/` (assets) and `sessions/` (captures) are not tracked by Git. A fresh checkout
needs a complete demo bundle at `local_data/stage_b/contact_demo` (docs/STAGE_B.md §2).
The default now uses this portable bundle directly; `bundle.json` records runtime file
hashes and historical sources. Derivation uses `--demo`, without legacy intermediate directories.
Generated `capture.yaml` files and bundles contain private simulation truth (the HMAC
optical key and lens distortion): use them to generate captures, never as reconstruction
input. On 2026-10-01 unused data was removed; the list is in
`local_data/data_inventory_2026-10-01.tsv`.
