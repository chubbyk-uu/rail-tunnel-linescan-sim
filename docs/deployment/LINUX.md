# 原生 Ubuntu Linux 部署与运行

本页不加载 WSL 组件。公共依赖与构建见 [DEPLOYMENT](../DEPLOYMENT.md)。

## 驱动与 Toolkit

1. 使用原生 Linux NVIDIA 驱动及其配套 OptiX 运行组件，安装 CUDA Toolkit 12.8；按 [NVIDIA Linux CUDA 安装指南](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) 配置驱动与 Toolkit。
2. 安装[公共部署页](../DEPLOYMENT.md)中的依赖及 OptiX SDK。确认 `nvidia-smi`、`nvcc` 和系统的 `libnvoptix.so.1` 正常。
3. 使用系统 OpenGL 驱动和正常桌面显示，不设置 WSL 的 `GALLIUM_DRIVER=d3d12`，不加载私有 WSL Mesa，也不复制上述 WSL 隔离库。

**2026-10-06 起启动脚本、GPU 测试和生产自检统一经 `tools/ssb_runtime.sh` 选择运行环境**：内核版本含 Microsoft 时走 WSL 隔离 OptiX（GUI 另加 d3d12 Mesa），否则走原生，保持系统驱动环境不变；也可用 `SSB_RUNTIME=wsl|native` 显式指定，无法判断时报错。**原生分支已实现，但尚未在原生主机上验收**，因此下文仍保留不依赖脚本的手动步骤；原生路线是否可用，以该机器的后端自检和实际采集结果为准。

## 原生 Linux 启动

先完成 [公共部署页](../DEPLOYMENT.md)的公共依赖、构建、[素材下载及资产生成](../ASSETS.md)与**原生后端自检**。系统 NVIDIA 驱动必须能够提供 CUDA、OpenGL 和 OptiX，使用本机驱动配套的运行库，不加载 WSL 隔离组件。

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

真实步长变更会被插件拒绝。最终检查须确认会话完成且原始块完整；仅仅 `gz sim` 退出不等于采集验收。后续展开和匹配用 [快速运行页](../QUICKSTART.md)的重建命令。

### 原生测试边界

CMake 的 GPU 测试和 `build_demo_from_sources.py --runtime native` 均经 `ssb_runtime.sh`，在原生主机上不加载 WSL 组件。但原生整套 `colcon test` 尚未在原生主机实际跑过，不能宣称已通过。

原生主机可以在正常 ROS/工作区终端中直接运行构建后的 GPU 测试，使用构建目录 PTX：

```bash
SSB_PTX="$PWD/build/ssb_core/ssb_scan.ptx" build/ssb_core/test_render \
  > /tmp/ssb_native_gpu_test.log 2>&1
```

测试成功后还需真实短程采集、哈希和覆盖检查，以及 GUI 联合运行验证。默认资产包已含测量标定，运行任务时不需要在原生主机重跑 WSL 包装的标靶生成；需要重新生成光学标定时，应先改用系统运行库的探针入口，再做独立验收。原生适配自动启动器、完整 GPU 测试和资产重建入口尚未作为本次文档更新的实现内容。
