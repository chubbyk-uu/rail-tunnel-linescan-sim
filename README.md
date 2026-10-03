# 地铁隧道轨道巡检机器人仿真

本项目复现轨道巡检机器人搭载旋转线阵相机的连续螺旋采集：车辆沿钢轨前进，相机与 COB 光源共同旋转，编码器按角度触发每一行，在上方 240° 范围内采集隧道内壁。Gazebo 负责轮轨接触和运动，OptiX 负责高速线阵成像，ROS 2 / RViz 提供任务控制与状态显示。后续通过图像展开、特征匹配和全局拼接优化，复原隧道内壁全图。

**当前进度（2026-10-03）：** 已完成历史 20 米原始采集、采后光学校正、CUDA 初始展开和相邻条带特征匹配；D3 连续轨迹优化已实现，开发场景结果不作为独立数据验收。完整构建后按冻结参数复采不同区段 3 米，通过阶段 B、协议核验及窗口内/窗口间 1 px 接缝门限；全图覆盖、最终融合及完整成果输出仍在进行。先完成 20 米全图，后续最多扩展到 **50 米**；100–150 米不再列为交付目标。

## 运行效果

### Gazebo：轨道接触、车载照明与旋转扫描

![Gazebo 旋转扫描动图](docs/media/gazebo_scan.gif)

[查看高清静态图：扫描架、车体与轨道](docs/media/gazebo.png)

### RViz：任务控制、车辆显示与原始图像预览

![RViz 采集任务动图](docs/media/rviz_capture.gif)

[查看高清静态图：任务面板与原图预览](docs/media/rviz.png)

以上由实际 3 米任务的窗口视频转换，取景跟随车体。RViz 环境为轻量预览，左下角是未经光学校正的原图缩略图；界面清晰度不代表 OptiX 原始成像精度。录制方式与版本见 [媒体说明](docs/media/README.md)。这些是增益调整前的历史界面媒体；当前增益 2.4 已重新标定，并于 2026-10-03 获用户确认为新画质基线；旧动画仅展示历史界面与机构运动。

## 功能与当前参数

| 项目 | 当前实现 |
|---|---|
| 环境 | 有效长度 20 m、内半径 2.75 m；模型两端各有 1.5 m 缓冲区 |
| 壁面 | Concrete034 混凝土背景、管片错缝、砂浆填缝和 0.2–0.6 mm 裂缝 |
| 轨道车 | 约 120 kg、橙白色外观；前轮驱动、后轮从动；独立双测量轮编码器 |
| 动力学 | 刚体轮轨接触、摩擦、导向约束；轨道起伏和车轮等效柔性 |
| 相机 | 4096 像素 Mono8，90 mm 镜头，约 0.852 m 轴向视场，8 μs 曝光 |
| 运动与触发 | 名义 0.2 m/s、估计里程 0.6 m/圈、约 20 rpm；巡航行频约 28.444 kHz |
| 门控 | 初始朝下；当前采集 −125°～+125°（含两侧各 5° 保护区），最终输出固定上方 240° |
| 光学 | 同转 COB 条形照明、像素足迹与曝光积分、0.6% 畸变仿真及图像标定 |
| 控制 | RViz 英文面板：任务起点、距离、开始、暂停、继续、停止和原图缩略图 |
| 重建 | 采后暗场/平场处理、测量镜头映射、0.2 mm 网格初始展开、图像对应点 |

每圈螺距由机器人估计里程决定；轮径误差会使实际螺距不同。当前默认真实与标定测量轮径均为 80 mm，固定安装偏差为零，但包含车体运动带来的少量姿态变化。轮径偏差、扫描轴偏心/方向偏差及估计里程停车的后续工作见 [阶段 D 计划](docs/STAGE_D.md)。

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

OptiX 隔离目录需要同一版本的 `libnvoptix.so.1`、`libnvidia-rtcore.so.610.57.04`、`libnvidia-gpucomp.so.610.57.04` 和配套 `nvoptix.bin`。这是本机使用的 WSL 实验运行环境，不是安装 Linux 驱动；不要覆盖 `/usr/lib/wsl/lib`。SDK 与隔离组件的准备步骤见 [WSL OptiX 部署](docs/DEPLOYMENT.md#wsl-optix-运行库)。

私有 Mesa 为 WSLg/D3D12 下的图形修复环境。本仓库包含运行包装器，但不包含 Mesa 构建器；新机器需要另行构建或恢复这套环境，具体依赖和来源见 [WSL Mesa 部署](docs/DEPLOYMENT.md#wsl-mesa-图形环境)。缺少它时默认 GUI 启动器会明确退出。

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

**当前 `run_mission.sh`、`run_gz_gui.sh`、`run_gz.sh` 及 GPU 测试包装器默认面向 WSL。** 原生 Linux 请使用 [原生 Linux 启动步骤](docs/DEPLOYMENT.md#原生-linux-启动)，直接运行相同的管理器、Gazebo 和 RViz；不要直接套用 WSL 启动脚本。原生路线能否正常运行以该机器的后端自检和实际采集结果为准。

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

## 下载素材并生成演示资产

Git 不包含大型混凝土原图和派生隧道资产。**新克隆应从公开素材网站下载原图，然后在本机生成演示；不需要从已有机器拷贝 `local_data`。** 裂缝候选 PNG、矢量目录和生成配方已经在 Git 中，不需要重新调用生图服务。

| 用途 | 素材页面 | 下载规格 |
|---|---|---|
| 混凝土细节 | [ambientCG Concrete034](https://ambientcg.com/view?id=Concrete034) | `16K-PNG.zip`；使用 Color、NormalGL、Roughness 三张图，实际 16384×8192，约 1.1×0.55 m |
| 大尺度色调 | [Poly Haven Painted Plaster Wall](https://polyhaven.com/a/painted_plaster_wall) | 4K、PNG、Diffuse；仅取 ≥20 mm 的明暗变化 |
| 灰色填缝砂浆 | [Poly Haven Grey Plaster](https://polyhaven.com/a/grey_plaster) | 8K、PNG、Diffuse；仅用于填缝细节 |

以上来源为 CC0。下载器使用这些网站的官方文件地址，继承当前终端的代理环境，校验仓库中固定的 SHA-256，并生成后续工具所需的 `downloads.json`。它只解压需要的三张 Concrete034 贴图，不解压整个包；已完整下载的素材可校验复用。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 可选：先设置你自己的 https_proxy / http_proxy，再执行下载。
# 不在仓库中保存代理地址或凭据。
python3 tools/download_demo_sources.py --output local_data/stage_b/sources \
  > /tmp/ssb_source_download.log 2>&1

# WSL：由原图生成 20 米隧道、纹理、裂缝、轨道车和独立标靶标定。
# work 和 output 都必须是尚不存在的新目录。
python3 tools/build_demo_from_sources.py --runtime wsl \
  --sources local_data/stage_b/sources \
  --work local_data/stage_b/build_from_sources_NEW \
  --output local_data/stage_b/contact_demo \
  > /tmp/ssb_asset_generation.log 2>&1
```

**原生 Linux** 使用同一下载命令，生成命令将 `--runtime wsl` 改为 `--runtime native`，标靶渲染直接加载系统 OptiX 运行库；无需 WSL Mesa。素材预处理还需要公共依赖中的 `libvips-tools`。原生主机整套生成与采集仍待实机验收。

生成流程自动完成：混凝土分层合成 → 砂浆 → 裂缝布局/细化/深度 → 隧道网格及漏光审计 → OptiX 场景 → Gazebo 预览 → 轮轨接触世界 → 标靶渲染与图像标定 → 相对引用的完整演示包。完整日志在 `work/generation.log`。生成结束后 `contact_demo` 可直接用于下面的启动命令。

如果本机已有默认演示，给 `--output` 换一个新目录进行复现，不覆盖旧资产。新生成的光学身份不同，必须使用新包自带的标定；不能混用旧的 `calibration.json`。输出包含仿真生成端真值和光学密钥，供生成采集使用，不能作为生产重建输入。

网站手动下载方法、目录结构、每一步命令和验证边界见 [素材下载与从零生成](docs/ASSETS.md)。默认仍生成 **20 米**；50 米需要重建长度相符的场景并另行验收，不能只改 RViz 的任务范围。

## 快速运行

### WSL：Gazebo + RViz 联合任务

完成构建、自检和资产生成后：

```bash
tools/run_mission.sh --gz-gui > /tmp/ssb_mission.log 2>&1
```

只开 RViz、让 Gazebo 服务器在后台运行：

```bash
tools/run_mission.sh > /tmp/ssb_mission.log 2>&1
```

在右侧面板选择任务模式：

| 模式/按钮 | 含义 |
|---|---|
| `Vehicle travel` | 设置车体起点和行程，默认可用范围 0–20 m；不承诺整个同长度壁面已覆盖 |
| `Wall coverage` | 设置目标壁面起点和长度，管理器按标定视场规划前后超扫 |
| `Start` | 在规划车体起点初始化车辆并开始新任务，不从上次位置自行行驶过去 |
| `Pause` / `Resume` | 暂停/恢复仿真；已经触发的图像仍会排空保存 |
| `Stop` | 提前结束并保存已采集原图，保留任务未完成标记 |

建议首次用 `Wall coverage`，壁面起点 **3 m**、长度 **3 m**，这样采集结果可直接用于后面的 D1 展开；最短任务 1 m。`Vehicle travel` 仅用于观察车体运动，其会话没有预先声明壁面目标，当前 D1 不接受这种会话。扫描头初始朝正下方，转到右下门控才开始曝光。左下角显示最近保存块的降采样原图，右侧显示估计里程、速度、扫描角、行数、成像滞后和输出路径。

关闭 RViz 时管理器会停止并排空自身采集进程。结束时先等待状态变为完成；不要强杀进程来省略写盘。Gazebo GUI 使用同一服务器；联合任务请通过 RViz 面板控制。

### WSL：只开 Gazebo 的固定演示

```bash
tools/run_gz_gui.sh > /tmp/ssb_gz_gui.log 2>&1
```

初始暂停，点击 Play 运行默认 3 米采集。关闭 GUI 后启动器等待服务器排空并检查会话；不会自动做畸变或平场校正。

### WSL：无界面采集指定壁面

```bash
# 目标壁面 [3,6] m，SESSION 及 SESSION_inputs 必须是新目录。
bash tools/run_wall_capture.sh sessions/wall_3m_NEW 3 3 \
  > /tmp/ssb_wall_3m.log 2>&1

# 20 米目标；确认磁盘空间充足后运行。
bash tools/run_wall_capture.sh sessions/wall_20m_NEW 0 20 \
  > /tmp/ssb_wall_20m.log 2>&1
```

这些入口保存原始图像并检查公开名义几何覆盖。原生 Linux 的联合任务与无界面步骤单列在 [部署文档](docs/DEPLOYMENT.md#原生-linux-启动)。

## 采后处理与查看结果

采集只保存原始 Mono8 图像。畸变、暗场和平场校正在拼接前执行，保留原始数据。D1 在原始列上校正亮度，将镜头几何校正合并进展开采样，避免先拉直图像再展开造成多次空间插值。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 将 SESSION 替换为已完成的 Wall coverage 会话，例如 sessions/wall_3m_NEW。
# Vehicle travel 会话不能用于当前 D1；不能事后篡改归档配置补目标。
ros2 run ssb_tools initial_unroll \
  --session SESSION \
  --calibration local_data/stage_b/contact_demo/calibration.json \
  --target-x 3 6 --pitch-mm 0.2 --backend cuda \
  --output sessions/d1_3m_NEW > /tmp/ssb_d1.log 2>&1

ros2 run ssb_tools match_bands \
  --unroll sessions/d1_3m_NEW \
  --output sessions/d2_3m_NEW > /tmp/ssb_d2.log 2>&1

python3 -m http.server 8765 --bind 127.0.0.1 --directory sessions/d1_3m_NEW
```

浏览器打开 `http://localhost:8765/review.html` 查看初始展开。要查看匹配诊断，可将服务器目录换成 `sessions/d2_3m_NEW`。D2 的局部对齐图用于诊断对应点，尚不是全局优化后的成果。

第一版 D3 可进一步用图像对应点拟合连续轨迹，并从原始列生成无融合对比图：

```bash
ros2 run ssb_tools optimize_bands \
  --unroll sessions/d1_3m_NEW --matches sessions/d2_3m_NEW \
  --observable SESSION/config/observable_config.json \
  --output sessions/d3_3m_NEW > /tmp/ssb_d3.log 2>&1

ros2 run ssb_tools review_global_bands \
  --unroll sessions/d1_3m_NEW --trajectory sessions/d3_3m_NEW \
  --output sessions/d3_review_NEW > /tmp/ssb_d3_review.log 2>&1

python3 -m http.server 8765 --bind 127.0.0.1 --directory sessions/d3_review_NEW
```

对比页显示同一网格上的编码器名义拼接和全局优化拼接，以及 100% 像素的硬接缝局部图。初始展开已补偿名义螺旋位移；优化进一步修正图像估计的轨迹误差，两侧都不做融合或锐化。正式带起伏 3 米留出采集的独立网格接缝 P95，窗口内/窗口间分别为 **0.547/0.775 px**，通过现行 **P95≤1 px** 目标。全原图无饱和；后续已导出 `57596×15000` 的整幅名义/优化图，约 8.64 亿像素的覆盖均无缺口，保存数据回读检查通过。整幅内部的覆盖通过不等于全像素几何误差小于 1 px；完整 20 米成果、融合和噪声场景仍待完成。详见 [3 米整幅实测](docs/STAGE_D.md#3-m-整幅实测)。

需要整幅优化结果及每像素覆盖统计时，运行以下命令。默认同时保存名义展开与优化全图，3 米对比约占 4.8 GiB；可用 `--optimized-only` 减半。输出为 `uint16`（DN×64）与覆盖计数，`review.html` 显示概览；不会用融合或补洞掩盖缺口。

```bash
python3 -m ssb_tools.global_mosaic \
  --unroll sessions/d1_3m_NEW --trajectory sessions/d3_3m_NEW \
  --output sessions/d3_full_NEW > /tmp/ssb_d3_full.log 2>&1
python3 tools/validate_global_mosaic.py \
  --unroll sessions/d1_3m_NEW --trajectory sessions/d3_3m_NEW \
  --mosaic sessions/d3_full_NEW --output local_data/evaluation/d3_full_NEW \
  > /tmp/ssb_d3_full_validation.log 2>&1
```

要同时查看未经补偿的斜向原始条带，以及板缝、裂缝形态附近的接缝，可另外生成图像复核页（需要 CUDA 和 v2 原图）：

```bash
python3 -m ssb_tools.feature_review \
  --unroll sessions/d1_3m_NEW --trajectory sessions/d3_3m_NEW \
  --observable SESSION/config/observable_config.json \
  --output sessions/d3_features_NEW > /tmp/ssb_d3_features.log 2>&1

python3 -m http.server 8765 --bind 0.0.0.0 --directory sessions/d3_features_NEW
```

打开 `http://localhost:8765/review.html`；WSL 下 Windows 的 localhost 转发不可用时，运行 `hostname -I`，改用 `http://WSL_IP:8765/review.html`。页面依次显示原始条带摆放、编码器名义展开和全局优化。原始示意没有做圈内前进、畸变或平场补偿，不是 D3 精度基线；局部位置仅从公开图像选取，原来的最大误差窗口也保留。

D1 默认使用 CUDA；可显式选择 `--backend cpu`，不会在 CUDA 失败时偷偷回退。处理输出目录必须是新的。当前 v2 按需读取原图、不保存原始列浮点缓存：3 米默认展开产物约 25 MB，20 米约 137 MB；只有显式选择 `--mosaic full` 才另存全分辨率图，3 米另加约 2.4 GiB。后续匹配仍需要相关原图块，搬家后用 D2 的 `--raw` 指定原图目录；不能只保留小型展开产物就删除原始采集。独立光学校正导出及验证命令见 [阶段 B §6](docs/STAGE_B.md#6-光学标定与校正) 和 [阶段 D](docs/STAGE_D.md)。

可选的传感器噪声档位及独立标定生成方法见 [传感器噪声](docs/SENSOR_NOISE.md)。默认演示仍关闭噪声，保持已确认的画质基线；噪声参数为仿真假设，不能当作 DALSA 实测规格。

## 性能与资源

以下为保留验收记录，不代表每台机器都能达到同样性能，也不是本次 GIF 录制的性能测试：

| 已验收任务 | 实测 |
|---|---|
| 20 m 原始采集，同时开 Gazebo / RViz | 1,945,476 行，原图约 7.42 GiB |
| 动力学 / 成像进度实时率 | 约 1.000 / 0.995 |
| 采集进程 RSS 合计峰值 | 约 4.09 GiB；共享页可能重复计算 |
| 全设备显存峰值 | 约 6.68 GiB，包含其他应用 |
| 3 m CUDA 初始展开 | 主流程约 18–20 s，加最终哈希约 25–27 s；进程 RSS 约 397 MiB |
| 3 m 相邻条带匹配 | 公开副本复测含哈希约 21.54 s；8 条带连通，142 个窗口接受 |

采集实际占用还包含元数据、诊断和评估文件。20 米原始验收数据约 8.61 GiB；独立重放另需一份空间。详细测量范围、方法和限制见 [阶段 C](docs/STAGE_C.md#6-完整-20-m-采集验收2026-10-02) 与 [阶段 D](docs/STAGE_D.md)。

## 测试与常见问题

完成前面的构建后，推荐用并行入口运行完整回归（本机 WSL 已验收）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export COLCON_DEFAULTS_FILE="$PWD/colcon_defaults.yaml"
python3 tools/run_tests.py > /tmp/ssb_test.log 2>&1
colcon test-result --all
# 两个 Gazebo 插件的实际服务器回归，使用新的输出目录。
python3 tools/test_gazebo_plugins.py --output /tmp/ssb_plugin_regression_NEW \
  > /tmp/ssb_plugin_regression.log 2>&1
```

默认使用 4 个 Python 工作进程，按上次报告的用例耗时分组；CPU 用例与 C++ 测试同时运行，Python CUDA 用例等待 C++ OptiX 测试退出后运行。不需要安装 pytest-xdist 等额外依赖，也不会筛掉用例或改变验收门限。本机完整回归约 **23 秒**，此前普通 `colcon test` 约 56–57 秒；当前 **388 项全部通过、无跳过**，包括新增的 5 项报告完整性检查。

脚本保存各组日志及 XML，核对用例身份、数量、重复项和退出码，再合并到 `build/ssb_tools/pytest.xml`，仍可用 `colcon test-result --all` 查看结果。默认日志目录为 `log/parallel_tests_时间_进程号/`；可用 `--output` 指定新的目录，或用 `--workers 1` 将 Python 用例改为串行排查。原始 `colcon test` 入口也保留。完整 20 米采集/展开复测及上面的 Gazebo 实际服务器回归仍单独执行，不包含在这 388 项中。

原生 Linux 的 GPU 测试应直接加载系统运行库；现有 WSL 包装测试不能当作原生部署验收，见 [部署文档](docs/DEPLOYMENT.md)。

| 现象 | 检查 |
|---|---|
| 缺少 `contact_demo` 或文件哈希不匹配 | 执行上述素材下载与生成步骤，检查配置、标定和世界是否成套；不要随意删包内文件 |
| `Missing runtime library` | WSL 隔离 OptiX 组件未准备好；原生 Linux 应使用原生入口 |
| `missing private Mesa component` | 检查 `SSB_MESA_PREFIX` 与安装内容；默认脚本不会自动改用系统 Mesa |
| `optixInit` / 后端启动失败 | SDK 只用于编译，核对实际驱动运行库；`nvidia-smi` 成功不足以证明 OptiX 正常 |
| 插件或构建身份不匹配 | 更新后重新构建并加载当前 `install/setup.bash`，检查插件搜索路径 |
| 标定光学身份不匹配 | 使用同一资产包的标定；改镜头/安装光学配置后重新标定 |
| RViz 环境纹理不如原图清楚 | RViz 使用轻量预览；成像质量应在原尺度输出中检查 |
| 输出目录已存在 | 换一个新目录，工具拒绝覆盖既有采集和处理结果 |
| 收尾仍显示排空或同步 | 等待完成并查看日志；这段时间不代表继续行驶 |

## 仓库结构与文档

```text
src/ssb_core/      编码器时序、成像几何、OptiX/CUDA、流水线和会话存储
src/ssb_gazebo/    Gazebo 运动/接触插件、GUI 光照与世界
src/ssb_rviz/      RViz 任务面板和原图预览
src/ssb_tools/     场景生成、标定、任务管理、展开、匹配和验收
assets/           已归档的裂缝生成素材与描述
local_data/       本机资产、派生世界、任务配置和日志（不进 Git）
sessions/         原始采集与处理结果（不进 Git）
docs/             设计、部署、验收、后续计划和界面媒体
```

生产重建只允许读取机器人可获得的图像、编码器、公开名义配置与图像估计的标定；仿真位姿、实际轮径、安装真值和缺陷标签只用于生成与独立 `evaluation/`。RViz 中的真实姿态仅供显示，不能作为拼接输入。暂不加入 IMU。

| 文档 | 内容 |
|---|---|
| [设计规范](DESIGN.md) | 几何、时序、光学、可观测性与阶段目标 |
| [素材与资产生成](docs/ASSETS.md) | 官方下载规格、从零生成、重新标定及日志 |
| [部署补充](docs/DEPLOYMENT.md) | WSL 专用运行环境、原生 Linux 启动和测试边界 |
| [阶段 B](docs/STAGE_B.md) | 场景、资产包、标定、采集与画质基线 |
| [阶段 C](docs/STAGE_C.md) | 壁面任务规划、20 米原始采集验收与资源 |
| [阶段 D 与后续计划](docs/STAGE_D.md) | CUDA 展开、图像匹配、全局优化和受控误差场景 |
| [开发规范](docs/DEVELOPMENT_RULES.md) | 邻近项目教训、真值隔离与 WSL I/O 要求 |
| [审核修复记录](docs/REPAIR_PLAN_2026-10-02.md) | 稳定性、运行完整性和照明遮挡验证 |
| [数据保留与清理](docs/DATA_RETENTION.md) | 当前基线、已清理数据与本地报告归档 |
| [历史资料](docs/history/README.md) | 选材、画质实验与早期开发记录 |
