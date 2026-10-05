# 快速运行：采集、重建与看图

命令默认在仓库根目录。先完成 [部署](DEPLOYMENT.md)、实际 OptiX 自检和 [素材生成](ASSETS.md)，并加载 ROS 与 `install/setup.bash`。本文运行名义默认包；综合误差的资产和冻结参数见 [ERROR_SCENARIOS](ERROR_SCENARIOS.md)。输出目录须不存在，源码更新后先完整构建。

## 1. GUI 任务

WSL 使用：

```bash
tools/run_mission.sh --gz-gui > /tmp/ssb_mission.log 2>&1
```

原生 Linux 使用 [分终端启动](deployment/LINUX.md#联合任务分别打开管理器和-rviz)，不套用 WSL 包装器。RViz 界面为英文：

| 控件 | 含义 |
|---|---|
| Wall coverage | 输入壁面目标起点和长度，默认范围 0–20 m、最短 1 m，自动规划前后超扫 |
| Vehicle travel | 输入车体起点和估计行程，不保证同长度壁面覆盖；正式 D1 要求 Wall coverage 的 inspection 声明 |
| Start | 预检资产/标定和边界，初始化车辆到规划起点，再开始采集 |
| Pause / Resume | 冻结/恢复仿真，计数、扫描相位与会话保持连续 |
| Stop | 结束运动并排空已接受曝光，保留未完成目标标记，不冒充完整任务 |
| 原图预览 | 最近保存原始块的低频缩略图，不是整圈成图，也不作光学校正 |

等待 Capture complete 后按需要关闭观察窗口。成像排空与车辆结束分别计时，排空期间车辆不需要继续前进。

## 2. 无界面采集

WSL：

```bash
bash tools/run_wall_capture.sh sessions/wall_NEW 12 3 \
  local_data/stage_b/contact_demo_buffered > /tmp/ssb_capture.log 2>&1
```

目标是壁面 [12,15] m，不是车体恰好走 3 m。生成配置保存在 `sessions/wall_NEW_inputs/`，含私有真值，不传给生产重建。原生采集见 [LINUX](deployment/LINUX.md#无界面壁面采集)。`wall_coverage` 检查公开名义几何覆盖，不能替代最终优化覆盖或独立真实网格验收。

## 3. 采后校正、展开与匹配优化

采集期间不进行畸变或平场补偿。D1 从原图读取时合并应用测量标定，不要求预先生成另一份完整校正图；原图保持不变。必须使用该包自己的兼容标定。

```bash
python3 -m ssb_tools.initial_unroll --session sessions/wall_NEW \
  --calibration local_data/stage_b/contact_demo_buffered/calibration.json \
  --backend cuda --output sessions/d1_NEW > /tmp/ssb_d1.log 2>&1
python3 -m ssb_tools.public_reconstruction \
  --unroll sessions/d1_NEW \
  --observable sessions/wall_NEW/config/observable_config.json \
  --root sessions/reconstruction_NEW --surface-relief --strict \
  > /tmp/ssb_reconstruction.log 2>&1
```

这组命令采用名义流程的默认匹配密度；不能冒充综合误差冻结方案。严格图像一致性检查也不等于独立几何验收。D1 小型索引仍依赖原图，不删除 `raw/`；移动原图后显式使用 `--raw`，详见 [STAGE_D](STAGE_D.md)。

## 4. 查看原图、名义展开和优化结果

```bash
python3 -m ssb_tools.feature_review \
  --unroll sessions/d1_NEW --trajectory sessions/reconstruction_NEW/fit \
  --observable sessions/wall_NEW/config/observable_config.json \
  --output sessions/review_NEW > /tmp/ssb_review.log 2>&1
python3 -m http.server 8765 --bind 0.0.0.0 --directory sessions/review_NEW
```

打开 `http://localhost:8765/review.html`，WSL 转发不可用时用 `hostname -I` 的地址。左图不补偿螺旋，名义展开用于区分去螺旋与优化效果；局部位置来自公开图像。可用 `feature_review --target-x 8 11` 仅裁切完整拟合的展示范围，不变更验收目标。

## 5. 完整成图和正式验收

特征复核页是有界预览，不等同于完整全图已输出。全分辨率成图、计数与独立 CPU 检查见 [STAGE_D](STAGE_D.md#看图与完整输出)。最终默认未融合；[融合](SEAM_FUSION.md)显式开启才运行。需要正式证据时使用 [冻结协议](EVALUATION.md)，先预声明再采集，不从已评分数据重新挑点或改门限。

## 排障

| 现象 | 检查 |
|---|---|
| 资产缺失/哈希失败 | 从网站生成成套包，保持运行依赖不可变 |
| OptiX/GUI 无法启动 | 对应平台运行库、WSL Mesa、真实射线自检及保存日志 |
| 构建/标定身份不匹配 | 完整重建并 source install；世界、配置和标定须成套 |
| RViz 贴图模糊 | 它是轻量预览；采集质量看原尺度原图 |
| 任务收尾未完成 | 查看保存行增长与成像滞后，等待排空，不强杀写盘进程 |

WSL 大数据放 `/home` 等 Linux 文件系统，日志集中写一个文件，不频繁递归扫描数据目录。
