# WSL2 / WSLg 部署

本页仅用于 WSL。公共系统包、源码与构建见 [DEPLOYMENT](../DEPLOYMENT.md)，任务运行见 [QUICKSTART](../QUICKSTART.md)。

## 驱动与 Toolkit

1. 在 Windows 安装支持 CUDA on WSL 的 NVIDIA 驱动，启用 WSL2 和 WSLg，使用 Ubuntu 24.04。安装流程参考 [NVIDIA CUDA on WSL 指南](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)。**不要在 WSL 内安装 Linux 显卡驱动**；配置 NVIDIA Toolkit 软件源后安装 `cuda-toolkit-12-8`，不使用会附带 Linux 驱动的 `cuda` / `cuda-drivers` 元包。
2. 项目、资产和采集文件放在 Linux 文件系统，如 `~/robot_ws/`，避免在 `/mnt/c` 下执行高频小文件读写。
3. 安装 CUDA Toolkit 12.8 和[公共部署页](../DEPLOYMENT.md)中的 ROS/Gazebo 依赖，再准备两套 WSL 专用用户态环境：

```bash
# 本项目默认查找的位置；允许在当前终端覆盖。
export SSB_OPTIX_RUNTIME="$HOME/opt/optix-runtime-610.57.04"
export SSB_MESA_PREFIX="$HOME/opt/agv-mesa-25.2.8/install"
```

OptiX 隔离目录需要同一版本的 `libnvoptix.so.1`、`libnvidia-rtcore.so.610.57.04`、`libnvidia-gpucomp.so.610.57.04` 和配套 `nvoptix.bin`。这是本机使用的 WSL 实验运行环境，不是安装 Linux 驱动；不要覆盖 `/usr/lib/wsl/lib`。SDK 与隔离组件的准备步骤见 [WSL OptiX 部署](#wsl-optix-运行库)。

私有 Mesa 为 WSLg/D3D12 下的图形修复环境。本仓库包含运行包装器，但不包含 Mesa 构建器；新机器需要另行构建或恢复这套环境，具体依赖和来源见 [WSL Mesa 部署](#wsl-mesa-图形环境)。缺少它时默认 GUI 启动器会明确退出。

检查 GPU 和显示接口：

```bash
nvidia-smi
/usr/local/cuda/bin/nvcc --version
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "$DISPLAY" "$WAYLAND_DISPLAY"
```

**WSL 启动器**会在子进程内选择 D3D12、私有 Mesa 和 OptiX 库，不修改全局系统库。OptiX 包装器会重置继承的 `LD_LIBRARY_PATH`；需要 ROS 的命令应在包装器内部重新 `source` ROS 和工作区。

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

回到项目根目录，执行公共部署页的 WSL `ssb_selfcheck`。驱动更新后再次自检。仅把 CUDA `stubs` 用于必要的链接检查，不能把它放进运行时 `LD_LIBRARY_PATH`。需要代理时在当前终端设置自己的代理环境，不把地址或凭据写入仓库。

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

它检查私有库和驱动目录是否存在，不等于图形渲染验收。然后用快速运行页的联合任务确认 Gazebo / RViz 可见，日志中无渲染错误。`--system` 是这个包装器的显式选项，但现有联合启动器仍选择 D3D12；不能据此把联合启动器当作原生 Linux 启动器。
