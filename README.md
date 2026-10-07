# 地铁隧道轨道巡检机器人仿真

约 120 kg 轨道车沿钢轨行驶，4096 像素线阵相机与 COB 条光一起连续旋转，编码器逐行触发形成螺旋扫描。Gazebo 模拟轮轨接触，OptiX 加速成像，ROS 2 / RViz 提供任务控制；采后经标定、展开、特征匹配和全局优化复原隧道内壁全图。

**20 米综合误差里程碑已完成**：轮径偏差、扫描轴偏移与双轴倾斜、轨道起伏和假设噪声同时开启；未调参新轨道种子的窗口内/间接缝 P95 为 **0.515/0.587 px**。完整优化图 **57.596 亿像素、零覆盖空洞**，Gazebo 与 RViz 同开时成像实时率约 **0.990**。P95 通过不保证每点≤1 px，完整覆盖也不是逐像素精度证明。范围和证据见 [20米里程碑](docs/MILESTONE_20M.md)；50 米尚未建模/验收。

## 运行效果

### Gazebo：轨道车与旋转扫描

![Gazebo 旋转扫描](docs/media/gazebo_scan.gif)

[高清静态图](docs/media/gazebo.png)

### RViz：任务控制与原图预览

![RViz 采集与控制](docs/media/rviz_capture.gif)

[高清静态图](docs/media/rviz.png)

动图由真实任务的视频生成，展示历史界面与机构运动；它们采用旧光照配置，不作为当前增益 2.4 的画质或性能证据。[媒体来源与哈希](docs/media/README.md)。

## 原始倾斜条带与优化结果

![原始螺旋条带与优化全图，含板缝和裂缝局部](docs/media/reconstruction_comparison.png)

取自最新完整 20 米综合采集的 **8–11 米**顶部概览，以及公开图像选出的板缝/裂缝原尺度局部，不是另一次 3 米验收。左侧摆放原始条带，**不补偿螺旋、畸变或平场**；右侧完成采后处理与优化，无融合、锐化或自动对比度。局部按公开拟合轨迹定位同一结构，裁切原点不同，不用真值对齐。

去除螺旋倾斜主要来自展开；优化的定量基线是名义展开。这批窗口内/间 P95 为 **74.657/74.447 → 0.515/0.587 px**。环缝形状、整图漂移和长细裂缝另有 [专项诊断](docs/MILESTONE_20M_DIAGNOSTICS.md)，照片强度宽度不能当作真实裂缝宽度。

## 场景与误差

| 项目 | 当前设置 |
|---|---|
| 隧道与输出 | 半径 2.75 m，有效 20 m；两端各 2.5 m 缓冲；上方 240° |
| 相机与扫描 | Mono8、4096 像素、90 mm；初始朝下，采集 250° |
| 运动与触发 | 名义 0.2 m/s，**编码器估计** 0.6 m/圈；2500 PPR、AB 四边沿、×128÷15，约 28.444 kHz |
| 光学与壁面 | 增益 2.4、8 µs、0.6% 假设畸变；Concrete034、砂浆板缝、细裂缝 |
| 默认演示 | 80/80 mm 测量轮、名义装配、2 mm 档轨道起伏；噪声关闭 |
| 20 m 综合验收 | 实际/标定轮径 81/80 mm；轴横向 +20 mm、竖向 −20 mm；绕 y +1 mrad、绕 z −1 mrad；轨道起伏、噪声开启 |

轮径影响真实距离、真实螺距和停车点；固定装配误差与接触产生的动态横滚/俯仰分别建模。暂不加 IMU。重建仅使用原图、公开编码器/门控、名义配置和标靶估计结果，**不读取仿真位姿、真实轮径或安装真值**。误差参数、生成与运行命令见 [ERROR_SCENARIOS](docs/ERROR_SCENARIOS.md)。融合可选、默认关闭。

## 安装与快速运行

公共环境：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、CUDA Toolkit 12.8、OptiX SDK 9.1.0、系统 Python 3.12。

| 系统 | 部署入口 |
|---|---|
| WSL2 / WSLg | [WSL 安装](docs/deployment/WSL.md)：Windows NVIDIA 驱动、隔离 OptiX、私有 Mesa；数据放 Linux 文件系统 |
| 原生 Ubuntu | [Linux 安装与启动](docs/deployment/LINUX.md)：系统 NVIDIA/OpenGL/OptiX；不加载 WSL 组件 |

完整验收来自 WSL / RTX 5080；原生 Linux 尚未在另一台主机完成全套验收。先按 [公共部署](docs/DEPLOYMENT.md) 安装依赖、克隆、构建并通过真实射线自检，再从网站生成资产。

```bash
# 仓库根目录，已 source ROS 和 install/setup.bash；下载支持终端代理配置。
python3 tools/download_demo_sources.py --output local_data/stage_b/sources \
  > /tmp/ssb_download.log 2>&1
python3 tools/build_demo_from_sources.py --runtime wsl \
  --sources local_data/stage_b/sources --work local_data/stage_b/build_NEW \
  --output local_data/stage_b/contact_demo_buffered > /tmp/ssb_assets.log 2>&1

# WSL：Gazebo + RViz，点击 Start 才开始任务。
tools/run_mission.sh --gz-gui > /tmp/ssb_mission.log 2>&1
```

首次部署从 [Concrete034](https://ambientcg.com/view?id=Concrete034) 等公开网站下载原图并本机生成，不需要拷贝旧机器资产。手动下载规格与原生生成选项见 [ASSETS](docs/ASSETS.md)。输出目录须为新目录；提交或源码更新后采集前完整构建，包括文档提交。

RViz 用 **Wall coverage** 指定壁面起点/长度，最短 1 米，支持 Start/Pause/Resume/Stop；车辆初始化到派生的超扫起点。**Vehicle travel** 仅指定车体估计行程，不能当作同长度完整壁面。采集保存原图，畸变与平场留到拼接前处理。无界面采集、重建及浏览命令见 [快速运行](docs/QUICKSTART.md)；综合误差使用专门的资产与冻结参数。

## 文档与验证

| 入口 | 内容 |
|---|---|
| [文档索引](docs/README.md) / [设计规范](DESIGN.md) | 当前规范、分章设计与各阶段操作 |
| [里程碑](docs/MILESTONE_20M.md) / [验收协议](docs/EVALUATION.md) | 当前结果、取点、真值隔离与证据边界 |
| [误差场景](docs/ERROR_SCENARIOS.md) / [性能](docs/D3_PERFORMANCE.md) | 误差如何加入、如何复现和阶段耗时 |
| [后续计划](docs/ROADMAP.md) / [数据保留](docs/DATA_RETENTION.md) | 50 米前的容量方案、保留与安全清理 |
| [开发守则](docs/DEVELOPMENT_RULES.md) / [历史归档](docs/history/README.md) | 构建溯源、WSL I/O 与旧失败/旧结果 |

2026-10-07 本机全套回归 **1069 项通过**（983 Python、86 C++），零失败/跳过（10-05 里程碑时为 990 项）；这是已保存的测试记录，不保证未来版本的数量不变。运行 `python3 tools/run_tests.py > /tmp/ssb_test.log 2>&1`，结果查看 `colcon test-result --all`；合入前必须在 GPU 主机上跑全套。GitHub CI 只运行无 GPU 的 `--profile cpu-ci`，不能代替全套。真实 Gazebo/RViz 集成脚本见 [tools/integration](tools/integration/README.md)。采集、独立重成像和几何验收另行执行。

`src/ssb_core` 负责时序/成像/存储，`ssb_gazebo` 负责动力学与 GUI，`ssb_rviz` 负责面板，`ssb_tools` 负责生成、标定和重建。资产、原始会话、构建和日志不进 Git。
