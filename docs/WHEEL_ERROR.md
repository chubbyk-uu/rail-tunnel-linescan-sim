# 编码器里程停车与轮径误差

更新：2026-10-04。本阶段先实现停车和三档轮径的受控接触验证，随后检查完整原图、独立重成像及公开输入重建。扫描轴固定安装偏差与 20 m 组合验收仍在 [ROADMAP](ROADMAP.md)。本页不把物理控制测试当作 D3 精度验收。

## 控制与完成判据

新任务规划器在接触模式写入 `motion.distance_stop`；`motion.profile` 保留名义运动估算，不再决定新任务结束。两路测量轮的量化计数和标定轮径产生估计里程，编码器轮轴角速度产生估计速度。没有车体世界坐标、真实轮径或真实轨道坡度参与控制。

起步采用 1 s 平滑速度渐入；剩余名义 0.1 m 开始制动，补偿 20 ms 响应延迟，最后按剩余距离降低速度。每个前驱轮读取自身转速，用 PI 力矩控制，限幅 ±8 N·m 并抑制积分饱和。比例控制在起伏轨道上有坡度导致的残余溜车，积分项用于消除该误差；失败后同样保持零目标速度。

估计里程误差 ≤0.1 mm、估计速度 ≤1 mm/s、扫描轴角速度 <0.01 rad/s，并满足已有 0.012 rad 跟踪上限，连续保持 0.5 s 后完成。扫描伺服保持正向；不为消除末端微小超前而倒转曝光。暂停时仿真时间不前进，保持计时冻结。超时标记失败并排空已接受的数据，用户 Stop 保留未完成标记。

会话 `motion` 记录完成依据、目标与实际估计里程及末端估计速度；Stage B 验收独立从归档轮轴角度和角速度复算保持区间。第一条归档位姿在生产端计数零点之后，独立复算显式计入一个计数的零点量化界限。旧配置无此字段时保留时间剖面控制，不修改既有会话和验收记录。

## 受控场景与复现

真实测量轮直径分别 79、80、81 mm，控制与重建的标定直径都保持 80 mm；真实碰撞半径与安装几何必须重新生成，通过物理世界一致性检查。轨道保持默认 2 mm 竖向与 2 mm 差动不平顺，车体姿态由接触产生。真值配置只在生成与 evaluation 中使用。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 先提交并完整构建，再运行作为证据保存的场景。
python3 tools/test_distance_stop.py \
  --output local_data/evaluation/distance_stop_control
# 增加 3 m 壁面目标的完整原图、独立重成像和阶段 B 验收。
bash tools/with_optix_runtime.sh python3 tools/test_distance_stop.py \
  --capture --output sessions/wheel_error_capture
```

每次使用新目录。工具直接调用已安装包与真实 Gazebo 插件；日志按运行保存，采样只写两个连续 CSV，不新增逐行文件。完整采集额外使用名义壁面超扫余量，其估计车体行程大于 3 m；不得与恰好请求车体前进 3 m 的停车试验混淆。

无滑移理想预期：请求估计行程 3 m 时，真实行程分别为 2.9625、3.0000、3.0375 m，真实螺距分别为 0.5925、0.6000、0.6075 m。实际参考取两个测量轮中心的位移；车体基准点随俯仰摆动，不能直接当作轮上位移。

## 当前结果与未通过项

冻结 `20cdd4a` 后完整构建，三档真实轮轨接触与完整采集在 `sessions/wheel_error_20261004/`。采集溯源均为干净源码且 `binary_matches_source=true`；每档 25 项阶段 B 检查通过，独立重成像逐字节相同，且刻意不足的超时不会被当作完成。完整回归 595 项通过，无失败或跳过，本机 25.40 s。

| 真实 / 标定轮径 | 请求估计 3 m 的实际轮上位移 | 实测螺距 | 名义展开接缝 P95 | 优化窗口内 / 窗口间 P95 |
|---|---:|---:|---:|---:|
| 80 / 80 mm | 3.000018 m | 0.599957 m | 44.943 px | 0.774 / 0.850 px |
| 79 / 80 mm | 2.962522 m | 0.592456 m | 73.951 px | 0.450 / 0.420 px |
| 81 / 80 mm | 3.037516 m | 0.607458 m | 79.441 px | 0.763 / **1.015 px，主门限失败** |

前两列来自恰好请求车体估计前进 3 m 的物理试验；后两列来自带首尾余量的 [3,6] m 壁面目标，其车体估计行程为 4.060191 m，每档约 40.8 万行原图。名义展开已补偿螺旋，是定量中间基线；展示用的原始条带仍不补偿螺旋。

生产 D2/D3 三档统一使用 0.2 m 匹配间距、0.02 m 姿态节点及既有约束，每档主进程和 8 个工作进程的实际私有读取违规数均为零。分别有 5,787、5,796、5,670 个冻结规则下的评价样本，计划点无缺测、四边支撑通过。三档均通过 3 px 次级门限，但 81 mm 档未通过 1 px 主门限；图像留出一致性通过不能替代这一结论。

三档均接受 314 个匹配窗口，未发生系数限幅或完全无支撑。81 mm 窗口间混凝土面 P95 为 0.839 px，非混凝土面 59 点的 P95 为 4.082 px；混凝土面有 102 点超过 1 px，加上上述 59 点，共 161/3096 点超过 1 px，将全体 P95 推过门限。材质分组只是隔离评价诊断，不剔除主评价点，也不证明误差已全部归因。仍需检查误差尾部和填缝视差处理；不降低门限或宣称所有轮径场景拼接合格。

原始报告、逐点评价及汇总哈希在 `sessions/wheel_error_20261004/evaluation/` 和各档 `capture/evaluation/wheel_geometry/`。未根据这些真值评分调参。该批为已知种子 20261001、噪声关闭的受控敏感性试验，不称作盲验，也不证明噪声与安装偏差组合已经合格；尚未验证仅靠图像恢复绝对尺度。

## 单独运行重建与评价

```bash
# 已完成 --capture 后，以 81 mm 档为例；三档保持相同参数。
bash tools/with_optix_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  folder=sessions/wheel_error_capture/images_81
  python3 -m ssb_tools.initial_unroll --session "$folder/capture" \
    --calibration local_data/stage_b/contact_demo/calibration.json \
    --backend cuda --output "$folder/unroll"
  python3 -m ssb_tools.public_reconstruction --unroll "$folder/unroll" \
    --observable "$folder/capture/config/observable_config.json" --root "$folder" \
    --spacing-m .2 --attitude-spacing-m .02
  python3 -m ssb_tools.evaluate_global_geometry --session "$folder/capture" \
    --unroll "$folder/unroll" --trajectory "$folder/fit" \
    --output "$folder/capture/evaluation/wheel_geometry"
'
```

评价命令完成不等于主门限通过，须检查报告 `gates.strict_seam.status`、缺测与四边支撑。原图不去重、不删除；D1 为 v2 原图按需采样，未为此生成整幅大数组。
