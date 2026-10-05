# 安装部署入口

命令在仓库根目录执行；先选择平台，再安装公共依赖、构建、自检和生成资产。

| 环境 | 部署和启动 |
|---|---|
| Windows + WSL2 / WSLg | [WSL 专用步骤](deployment/WSL.md)：Windows 驱动、隔离 OptiX、私有 Mesa |
| 原生 Ubuntu Linux | [Linux 专用步骤](deployment/LINUX.md)：系统 NVIDIA/OpenGL/OptiX、独立启动命令 |

已验证环境为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic/gz-sim 8、CUDA Toolkit 12.8、OptiX SDK 9.1.0、系统 Python 3.12，WSL 主机 GPU 为 RTX 5080 16 GiB。原生 Linux 尚未在另一台主机完成全套验收。WSL 的自动 GUI、采集及测试包装器不能直接当作原生启动器。

## 公共依赖与源码

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

构建同时安装 `libssb_ray_numeric.so` 与 `libssb_ray_cuda.so`。D3 生产 CLI 默认 CUDA 残差/雅可比，支持 `--geometry-backend cpu` 显式使用融合 CPU；公开留出脚本可设 `SSB_D3_BACKEND=cpu`。CUDA 优化工作分配上限 4 GiB，20 m 实测约 2.11 GiB，另需约 7.57 GiB 主机峰值 RSS，详见 [性能与范围](D3_PERFORMANCE.md)。

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

## 两种环境的离线 CPU 预算

D2 匹配和独立接缝评价默认 `min(8, 当前进程可用 CPU 数)`，优先读取 CPU affinity，系统不支持时回退到 `os.cpu_count()`。因此 WSL 只分配 4 核或 Linux 进程限制为 4 核时默认使用 4 个，不会照宿主机总核数启动。可用 `--workers N` 设置不超过可用核心数的正整数；`run_d3_holdout.sh` 接受 `SSB_OFFLINE_WORKERS=N` 同时设置 D2 与评价。D3 分块线程采用相同默认预算，未将上述 D2/评价参数解释成 D3 线程设置。完整测试入口仍默认 4 个 Python 工作进程。

## 数据和排障

- `local_data/`、`sessions/`、`build/`、`install/`、`log/` 不进 Git。先部署系统运行环境和源码，再按 ASSETS.md 从公开素材生成完整演示；新部署不以旧机器资产迁移为前提。
- WSL 下数据放 Linux 文件系统；尽量用一个资产压缩包传输，采集和重建使用顺序块写入，避免逐行同步或数千小文件。
- 采集结束不自动补偿畸变/平场；原图保留。生产重建禁止读取资产真值或 `evaluation/`。
- 更新提交后采集前重新构建；仅文档变更也会影响 Git 溯源身份。
- WSL 启动日志位于 `local_data/mission_logs/` 或 `local_data/gui_logs/`；管理器为每次任务保存生成配置和 `server.log` / `gui.log`。
- 原生启动示例的进程输出位于原生启动页中的 `/tmp/ssb_native_*.log`；任务生成日志仍由管理器保存到数据目录。

资产从网站下载并在本机生成，见 [ASSETS](ASSETS.md)；最短闭环见 [快速运行](QUICKSTART.md)。
