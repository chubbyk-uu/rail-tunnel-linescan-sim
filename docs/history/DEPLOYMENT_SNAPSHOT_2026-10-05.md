> 2026-10-05 整理前快照。文中的“当前”“最新”“待完成”及路径只代表当时记录；现行状态以 [20米里程碑](../MILESTONE_20M.md) 为准。旧失败及首次报告不改写；文件存在性以现行数据清单为准。

# 安装部署：WSL 与原生 Linux

本页集中记录依赖安装、构建、自检、两种环境的运行差异和排障。快速入口见 [README](../../README.md)，从网站下载并生成资产见 [ASSETS](../ASSETS.md)。命令默认在仓库根目录执行。

## 安装部署：先选择运行环境

目前完整 GUI 和 20 米性能验收在 **Windows + WSL2/WSLg + Ubuntu 24.04 + RTX 5080** 上完成。原生 Linux 使用系统 NVIDIA 驱动，下面给出独立的部署和启动路径；尚未在另一台原生 Linux 主机完成整套验收。

| 依赖 | 本项目已验证版本/用途 |
|---|---|
| 系统 | Ubuntu 24.04，x86_64 |
| ROS 2 | Jazzy，含 RViz 2 |
| Gazebo | Harmonic / gz-sim 8，Ogre 2 |
| CUDA Toolkit | 12.8；需要 `nvcc`，仅有 `nvidia-smi` 不够 |
| OptiX SDK | 9.1.0；用于编译，运行时还需要 OptiX 驱动组件 |
| Python | 系统 Python 3.12；NumPy、SciPy、OpenCV、Pillow、PyYAML 等 |
| 本机 GPU | RTX 5080，16 GiB 显存；这不是声明所有其他 GPU 均已验证 |

### 路线一：WSL2 / WSLg

1. 在 Windows 安装支持 CUDA on WSL 的 NVIDIA 驱动，启用 WSL2 和 WSLg，使用 Ubuntu 24.04。安装流程参考 [NVIDIA CUDA on WSL 指南](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)。**不要在 WSL 内安装 Linux 显卡驱动**；配置 NVIDIA Toolkit 软件源后安装 `cuda-toolkit-12-8`，不使用会附带 Linux 驱动的 `cuda` / `cuda-drivers` 元包。
2. 项目、资产和采集文件放在 Linux 文件系统，如 `~/robot_ws/`，避免在 `/mnt/c` 下执行高频小文件读写。
3. 安装 CUDA Toolkit 12.8 和后面的 ROS/Gazebo 公共依赖，再准备两套 WSL 专用用户态环境：

```bash
# 本项目默认查找的位置；允许在当前终端覆盖。
export SSB_OPTIX_RUNTIME="$HOME/opt/optix-runtime-610.57.04"
export SSB_MESA_PREFIX="$HOME/opt/agv-mesa-25.2.8/install"
```

OptiX 隔离目录需要同一版本的 `libnvoptix.so.1`、`libnvidia-rtcore.so.610.57.04`、`libnvidia-gpucomp.so.610.57.04` 和配套 `nvoptix.bin`。这是本机使用的 WSL 实验运行环境，不是安装 Linux 驱动；不要覆盖 `/usr/lib/wsl/lib`。SDK 与隔离组件的准备步骤见 [WSL OptiX 部署](../deployment/WSL.md#wsl-optix-运行库)。

私有 Mesa 为 WSLg/D3D12 下的图形修复环境。本仓库包含运行包装器，但不包含 Mesa 构建器；新机器需要另行构建或恢复这套环境，具体依赖和来源见 [WSL Mesa 部署](../deployment/WSL.md#wsl-mesa-图形环境)。缺少它时默认 GUI 启动器会明确退出。

检查 GPU 和显示接口：

```bash
nvidia-smi
/usr/local/cuda/bin/nvcc --version
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "$DISPLAY" "$WAYLAND_DISPLAY"
```

**WSL 启动器**会在子进程内选择 D3D12、私有 Mesa 和 OptiX 库，不修改全局系统库。OptiX 包装器会重置继承的 `LD_LIBRARY_PATH`；需要 ROS 的命令应在包装器内部重新 `source` ROS 和工作区。

### 路线二：原生 Ubuntu Linux

1. 使用原生 Linux NVIDIA 驱动及其配套 OptiX 运行组件，安装 CUDA Toolkit 12.8；按 [NVIDIA Linux CUDA 安装指南](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) 配置驱动与 Toolkit。
2. 安装后面的公共依赖及 OptiX SDK。确认 `nvidia-smi`、`nvcc` 和系统的 `libnvoptix.so.1` 正常。
3. 使用系统 OpenGL 驱动和正常桌面显示，不设置 WSL 的 `GALLIUM_DRIVER=d3d12`，不加载私有 WSL Mesa，也不复制上述 WSL 隔离库。

**当前 `run_mission.sh`、`run_gz_gui.sh`、`run_gz.sh` 及 GPU 测试包装器默认面向 WSL。** 原生 Linux 请使用 [原生 Linux 启动步骤](../deployment/LINUX.md#原生-linux-启动)，直接运行相同的管理器、Gazebo 和 RViz；不要直接套用 WSL 启动脚本。原生路线能否正常运行以该机器的后端自检和实际采集结果为准。

### 两种环境共用：ROS、依赖和源码

先按 [ROS 2 Jazzy 官方安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 配置 ROS 软件源；需要独立安装 Gazebo 时参考 [Harmonic 官方说明](https://gazebosim.org/docs/harmonic/install_ubuntu/)。然后安装项目依赖：

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  python3-colcon-common-extensions python3-rosdep \
  build-essential cmake ninja-build pkg-config git curl libvips-tools \
  libyaml-cpp-dev nlohmann-json3-dev libssl-dev qtbase5-dev \
  python3-numpy python3-scipy python3-opencv python3-pil \
  python3-yaml python3-matplotlib python3-pytest python3-psutil

mkdir -p ~/robot_ws
cd ~/robot_ws
git clone https://github.com/chubbyk-uu/rail-tunnel-linescan-sim.git Subway_scan_bot_sim
cd Subway_scan_bot_sim

# 官方 OptiX SDK；默认目录与项目 CMake 一致。
mkdir -p ~/opt
git clone --branch v9.1.0 --depth 1 \
  https://github.com/NVIDIA/optix-sdk.git ~/opt/optix-sdk-9.1.0

# 首次使用 rosdep 时执行 init；已经初始化的机器跳过这一行。
sudo rosdep init
rosdep update
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

本仓库已公开，克隆无需 GitHub 登录；WSL 私有 Mesa 构建来源的独立仓库权限要求见部署补充。`gz.transport13`、`gz.msgs10` 来自 Jazzy 的 Gazebo vendor 环境，先加载 ROS，再检查 Python 导入；不要用其他 Python/Conda 环境替代系统 Python。

```bash
source /opt/ros/jazzy/setup.bash
python3 -c 'import gz.transport13, gz.msgs10, numpy, scipy, cv2, PIL, yaml; print("Python dependencies OK")'
```

## 构建与后端自检

下文命令除另有说明均在仓库根目录运行。

```bash
source /opt/ros/jazzy/setup.bash
export COLCON_DEFAULTS_FILE="$PWD/colcon_defaults.yaml"
colcon build > /tmp/ssb_build.log 2>&1
source install/setup.bash
```

默认构建采用 `RelWithDebInfo` 和 `symlink-install`。若 SDK/Toolkit 位于其他目录：

```bash
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DSSB_OPTIX_SDK=/path/to/optix-sdk-9.1.0 \
  -DCUDAToolkit_ROOT=/path/to/cuda-12.8 > /tmp/ssb_build.log 2>&1
```

构建同时安装 `libssb_ray_numeric.so` 与 `libssb_ray_cuda.so`。D3 生产 CLI 默认 CUDA 残差/雅可比，支持 `--geometry-backend cpu` 显式使用融合 CPU；公开留出脚本可设 `SSB_D3_BACKEND=cpu`。CUDA 优化工作分配上限 4 GiB，20 m 实测约 2.11 GiB，另需约 7.57 GiB 主机峰值 RSS，详见 [性能与范围](../D3_PERFORMANCE.md)。

缺少 CUDA 或 OptiX 会直接使配置失败，项目不静默降级成其他成像后端。每次更新 Git 提交或源码后，采集前重新构建；采集会核对构建版本与运行源码身份，包括文档提交。

```bash
# WSL：通过隔离运行库执行实际射线后端自检。
bash tools/with_optix_runtime.sh \
  install/ssb_core/lib/ssb_core/ssb_selfcheck \
  src/ssb_core/config/stage_a.yaml > /tmp/ssb_selfcheck.log 2>&1

# 原生 Linux：在已加载 ROS/工作区的终端中直接执行。
install/ssb_core/lib/ssb_core/ssb_selfcheck \
  src/ssb_core/config/stage_a.yaml > /tmp/ssb_selfcheck.log 2>&1
```

只执行与你环境对应的一条。自检验证的是 OptiX 初始化与真实射线计算，不只是 CUDA 设备是否可见。

## WSL OptiX 运行库

SDK 提供头文件和 API，实际光线追踪还依赖驱动侧的 `libnvoptix.so.1`。当前 WSL 运行包装器明确检查 610.57.04 组件；更换组件版本必须重新核对包装器和运行兼容性，不能混合不同驱动包的文件。

参考 [NVIDIA 论坛中的 WSL 实验方案](https://forums.developer.nvidia.com/t/running-optix-on-wsl-2026-version/382414)，以下仅解包用户态组件，不安装 Linux 驱动、不改 Windows 驱动目录和 `/usr/lib/wsl/lib`。这是项目使用的实验部署方式，不是 NVIDIA 对所有 WSL/驱动组合的兼容保证。

在新的临时目录操作，下载来源为 [NVIDIA 官方组件目录](https://download.nvidia.com/XFree86/Linux-x86_64/610.57.04/)：

```bash
mkdir -p /tmp/ssb-optix-extract
cd /tmp/ssb-optix-extract
curl --fail --location --retry 2 \
  https://download.nvidia.com/XFree86/Linux-x86_64/610.57.04/NVIDIA-Linux-x86_64-610.57.04.run \
  -o NVIDIA-Linux-x86_64-610.57.04.run
# 不加 sudo，只解包，不执行驱动安装流程。
sh NVIDIA-Linux-x86_64-610.57.04.run --extract-only

export SSB_OPTIX_RUNTIME="$HOME/opt/optix-runtime-610.57.04"
# 此处应是新的目录；已有安装先核对，不覆盖或混装。
mkdir -p "$SSB_OPTIX_RUNTIME"
cp NVIDIA-Linux-x86_64-610.57.04/libnvoptix.so.610.57.04 \
   NVIDIA-Linux-x86_64-610.57.04/libnvidia-rtcore.so.610.57.04 \
   NVIDIA-Linux-x86_64-610.57.04/libnvidia-gpucomp.so.610.57.04 \
   NVIDIA-Linux-x86_64-610.57.04/nvoptix.bin "$SSB_OPTIX_RUNTIME/"
ln -s libnvoptix.so.610.57.04 "$SSB_OPTIX_RUNTIME/libnvoptix.so.1"
```

回到项目根目录，执行 README 的 WSL `ssb_selfcheck`。驱动更新后再次自检。仅把 CUDA `stubs` 用于必要的链接检查，不能把它放进运行时 `LD_LIBRARY_PATH`。需要代理时在当前终端设置自己的代理环境，不把地址或凭据写入仓库。

`tools/with_optix_runtime.sh` 只影响子进程，设置隔离库、WSL CUDA 接口和 Toolkit 的运行库路径。包装器会丢弃继承的 `LD_LIBRARY_PATH`，ROS 命令必须在它内部加载环境，例如：

```bash
bash tools/with_optix_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  # 在这里运行需要 ROS 和 OptiX 的命令。
'
```

## WSL Mesa 图形环境

本机使用私有 Mesa 25.2.8，默认安装目录 `~/opt/agv-mesa-25.2.8/install`。`tools/with_mesa_runtime.py` 只为子进程加载对应的 GL/EGL/GBM 库，避免替换系统库。

本仓库**没有收录私有 Mesa 构建器及补丁**。已有安装可以恢复到上述路径，或用 `SSB_MESA_PREFIX` 指向另一份同结构安装；新机器需另行取得构建器。构建方法来自 [4W 项目的 MESA_SETUP.md](https://github.com/chubbyk-uu/four-wheel-steering-agv-inspection/blob/main/docs/MESA_SETUP.md)，需要该仓库的访问权限。版本锁、补丁和构建脚本应作为一套使用，不单独复制启动器。

若已取得完整的 4W 源码，在 **4W 仓库根目录**执行其构建步骤：

```bash
sudo apt-get update
sudo apt-get install -y build-essential ninja-build pkg-config dpkg-dev patch python3 \
  libglvnd-dev zlib1g-dev libzstd-dev libexpat1-dev libdrm-dev libudev-dev \
  libelf-dev libunwind-dev libwayland-dev libx11-dev libxext-dev libx11-xcb-dev \
  libxxf86vm-dev libxrandr-dev x11proto-dev spirv-tools \
  libxcb-glx0 libxcb-shm0 libxcb-shape0 libxcb-dri2-0 libxcb-dri3-0 \
  libxcb-randr0 libxcb-present0 libxcb-sync1 libxcb-xfixes0 libxcb-render0 libxshmfence1
python3 tools/build_private_mesa.py > /tmp/ssb_private_mesa_build.log 2>&1
```

固定版本依赖和下载哈希以该构建器的锁文件为准；上游固定包不可用时需明确处理，不能声称只克隆本项目就能恢复整套 WSL 图形环境。构建结果放在 `~/opt/`，不要放进可清理的 `local_data/`。

回到本项目，先检查包装器：

```bash
export SSB_MESA_PREFIX="$HOME/opt/agv-mesa-25.2.8/install"
python3 tools/with_mesa_runtime.py /usr/bin/true
```

它检查私有库和驱动目录是否存在，不等于图形渲染验收。然后用 README 的联合任务确认 Gazebo / RViz 可见，日志中无渲染错误。`--system` 是这个包装器的显式选项，但现有联合启动器仍选择 D3D12；不能据此把联合启动器当作原生 Linux 启动器。

## 两种环境的离线 CPU 预算

D2 匹配和独立接缝评价默认 `min(8, 当前进程可用 CPU 数)`，优先读取 CPU affinity，系统不支持时回退到 `os.cpu_count()`。因此 WSL 只分配 4 核或 Linux 进程限制为 4 核时默认使用 4 个，不会照宿主机总核数启动。可用 `--workers N` 设置不超过可用核心数的正整数；`run_d3_holdout.sh` 接受 `SSB_OFFLINE_WORKERS=N` 同时设置 D2 与评价。D3 分块线程采用相同默认预算，未将上述 D2/评价参数解释成 D3 线程设置。完整测试入口仍默认 4 个 Python 工作进程。

## 原生 Linux 启动

先完成 README 的公共依赖、构建、[素材下载及资产生成](../ASSETS.md)与**原生后端自检**。系统 NVIDIA 驱动必须能够提供 CUDA、OpenGL 和 OptiX，使用本机驱动配套的运行库，不加载 WSL 隔离组件。

以下命令在原生 Ubuntu 的正常桌面终端、项目根目录执行。若之前在该终端手工设置过 WSL 的 Mesa `LD_PRELOAD` 或 D3D12 环境，重新打开干净终端。

### 联合任务：分别打开管理器和 RViz

**终端一：**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_PARTITION=ssb_native_mission
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_GUI_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
python3 -m ssb_tools.mission_manager --gz-gui \
  > /tmp/ssb_native_manager.log 2>&1
```

**终端二：**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_PARTITION=ssb_native_mission
rviz2 -d install/ssb_rviz/share/ssb_rviz/config/mission.rviz \
  --ros-args -p use_sim_time:=true > /tmp/ssb_native_rviz.log 2>&1
```

两个终端使用同一 `GZ_PARTITION`，ROS domain 也须一致。管理器启动后，RViz 面板用法与 WSL 相同；点击 Start 后管理器才创建 Gazebo 服务器和 GUI。只需 RViz 时去掉终端一的 `--gz-gui`。

这组命令不经过 WSL 包装器。与 `run_mission.sh` 的差别是两个终端各自管理进程：关闭 RViz **不会自动关闭另一个终端的管理器**。先在面板等待任务完成或点 Stop 保存，随后在终端一 Ctrl+C，让管理器排空并关闭自身 Gazebo 进程，最后关闭 RViz。

### 无界面壁面采集

下面给出目标 `[3,6] m` 的独立步骤，不调用 WSL 的 `run_gz.sh` 或 `run_wall_capture.sh`。输出目录必须是新的：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m ssb_tools.mission_plan \
  --demo local_data/stage_b/contact_demo_buffered \
  --output sessions/native_wall_3m_NEW_inputs \
  --target-start-m 3 --target-length-m 3 > /tmp/ssb_native_plan.log 2>&1

export GZ_PARTITION=ssb_native_wall
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export SSB_CONFIG="$PWD/sessions/native_wall_3m_NEW_inputs/capture.yaml"
export SSB_WORLD="$PWD/sessions/native_wall_3m_NEW_inputs/world.sdf"
export SSB_SESSION="$PWD/sessions/native_wall_3m_NEW"
python3 -m ssb_tools.physical_world check \
  --config "$SSB_CONFIG" --world "$SSB_WORLD"
```

计算与运动剖面匹配的步数（这里使用 Bash 变量 `ssb_iterations`，不硬编码任务时长）：

```bash
ssb_iterations=$(python3 - <<'PY'
import os, math, yaml
with open(os.environ['SSB_CONFIG']) as f:
    c = yaml.safe_load(f)
m = c['motion']
settle = c.get('contact', {}).get('settle_s', 2) if c.get('contact', {}).get('enabled') else 0
print(math.ceil((m['profile'][-1][0] + settle) / m['sample_period_s'] - 1e-9) + 2)
PY
)
gz sim -s -r -v 3 --iterations "$ssb_iterations" "$SSB_WORLD" \
  > /tmp/ssb_native_capture.log 2>&1
python3 tools/check_session.py "$SSB_SESSION"
python3 -m ssb_tools.wall_coverage \
  --session "$SSB_SESSION" \
  --calibration local_data/stage_b/contact_demo_buffered/calibration.json \
  --output "$SSB_SESSION/reconstruction/coverage"
```

真实步长变更会被插件拒绝。最终检查须确认会话完成且原始块完整；仅仅 `gz sim` 退出不等于采集验收。后续展开和匹配用 README 的共同命令。

### 原生测试边界

当前 CMake 的 GPU 测试仍调用 `with_optix_runtime.sh`，`optical_bench --render` 等部分生成工具也使用该包装器。这些工具属于现有 WSL 路径，不能直接宣称原生整套 `colcon test` 已通过。

原生主机可以在正常 ROS/工作区终端中直接运行构建后的 GPU 测试，使用构建目录 PTX：

```bash
SSB_PTX="$PWD/build/ssb_core/ssb_scan.ptx" build/ssb_core/test_render \
  > /tmp/ssb_native_gpu_test.log 2>&1
```

测试成功后还需真实短程采集、哈希和覆盖检查，以及 GUI 联合运行验证。默认资产包已含测量标定，运行任务时不需要在原生主机重跑 WSL 包装的标靶生成；需要重新生成光学标定时，应先改用系统运行库的探针入口，再做独立验收。原生适配自动启动器、完整 GPU 测试和资产重建入口尚未作为本次文档更新的实现内容。

## 数据和排障

- `local_data/`、`sessions/`、`build/`、`install/`、`log/` 不进 Git。先部署系统运行环境和源码，再按 ASSETS.md 从公开素材生成完整演示；新部署不以旧机器资产迁移为前提。
- WSL 下数据放 Linux 文件系统；尽量用一个资产压缩包传输，采集和重建使用顺序块写入，避免逐行同步或数千小文件。
- 采集结束不自动补偿畸变/平场；原图保留。生产重建禁止读取资产真值或 `evaluation/`。
- 更新提交后采集前重新构建；仅文档变更也会影响 Git 溯源身份。
- WSL 启动日志位于 `local_data/mission_logs/` 或 `local_data/gui_logs/`；管理器为每次任务保存生成配置和 `server.log` / `gui.log`。
- 原生启动示例的进程输出位于上面的 `/tmp/ssb_native_*.log`；任务生成日志仍由管理器保存到数据目录。
