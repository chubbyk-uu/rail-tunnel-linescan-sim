# 阶段 C：壁面任务与原始采集

更新 2026-10-05。完整 20 m 原图、独立重成像与联合 GUI 采集已完成，当前综合证据见 [里程碑](MILESTONE_20M.md)。本页定义任务语义和名义覆盖；2026-10-02 的旧车体起止、门控与验收数字保留在 [快照](history/STAGE_C_SNAPSHOT_2026-10-05.md)，不用于现行规划。

## 1. 可测输入与原图

暂不加 IMU。公开输入包括原图、曝光/门控、扫描与双测量轮编码器、名义配置和图像估计标定；禁用仿真位姿/TF、真实轮径、实际安装、网格/缺陷真值。采集保存原图，不校正畸变和平场；D1 在拼接前合并校正，不要求采集结束自动生成另一套校正图。

## 2. Task mode 与停车

| 模式 | 含义 |
|---|---|
| Vehicle travel | 起点和车体编码器估计行程；最短 1 m，不能据此声称同长度壁面完整 |
| Wall coverage | 壁面起点/长度；默认 [0,20] m、最短 1 m，自动派生前后超扫，正式 D1 所需的 inspection 预声明 |

点 Start 并通过预检后，车辆初始化到规划起点；改输入框本身不移动车辆。Wall coverage 的车体起点可进入已建模缓冲区。前驱轮速度 PI 控制、双测量轮估计里程停车，扫描随估计里程推进；实际轮径有偏差时真实终点/螺距也偏离，见 [ERROR_SCENARIOS](ERROR_SCENARIOS.md)。

暂停冻结仿真，已有曝光可以继续成像落盘；提前 Stop 保留未完成标记。正式完成需估计距离/速度和轴跟踪满足保持判据，再排空写盘。运动完成、成像完成与离线处理分别报告。

## 3. 覆盖规划

从图像标定的连续有效视场、名义装配、0.6 m 估计螺距、加减速剖面和公开修正界限规划两端超扫。相对尺度模式另按 ±3% 的公开包络扩大支持，绝不使用真实轮径。

目标区、240° 输出域和 0.2 mm 网格在采集前写入 inspection，不能采后缩区或调粗网格凑通过。验收 `valid_x_m` 只是曝光中心的轴向区域，不是二维完整壁面。任务边界由公开 mission 包络和实际已建模范围校验，具体起止以生成的 task.json 为准，不使用旧文固定数值。

## 4. 运行与名义覆盖检查

先按 [部署](DEPLOYMENT.md) 构建并生成资产。WSL 使用新目录：

```bash
bash tools/run_wall_capture.sh sessions/wall_3m_NEW 3 3 \
  local_data/stage_b/contact_demo_buffered > /tmp/ssb_wall.log 2>&1
```

输入配置/世界在 `sessions/wall_3m_NEW_inputs/`，包含私有生成信息，不传给重建。GUI 使用 `tools/run_mission.sh --gz-gui`；原生路径见 [LINUX](deployment/LINUX.md)。综合资产按 [ERROR_SCENARIOS](ERROR_SCENARIOS.md) 开启对应支持开关。

```bash
python3 -m ssb_tools.wall_coverage --session sessions/wall_3m_NEW \
  --calibration local_data/stage_b/contact_demo_buffered/calibration.json \
  --output sessions/wall_3m_NEW/reconstruction/coverage_NEW
```

## 5. 名义覆盖的含义

从双轮计数和标定直径计算轴向位置，扫描编码器/门控计算角度，标定映射/有效列决定每行像素足迹。足迹与行间距分别建模，删除行或坏列不能通过插值桥接。方形像元及径向镜头推算周向足迹是假设，不是独立二维标定或实测 MTF。

检查目标网格的所有中心，报告覆盖次数、全部空洞和边界；按周向分块写覆盖游程，不分配整个稠密图。20 m 为 57,596×100,000 网格，预览不代替全网格。

本检查不证明实际未知姿态、轮径误差下的投影正确，也不证明饱和、清晰度或遮挡。最终完整优化图与真实网格几何需要 [阶段 D](STAGE_D.md) 和 [验收协议](EVALUATION.md)，三者独立报告。任务回归须包含阶段 B 和实际独立重成像，不只检查表行号。
