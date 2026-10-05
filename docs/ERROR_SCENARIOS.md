# 误差场景：定义、配置和复现

更新 2026-10-05。误差已加入，不是尚待实施项。当前正式交付使用轮径＋轴偏移＋双轴倾斜＋轨道起伏＋噪声的一组综合场景，不展开九组参数矩阵。正式结果见 [20米里程碑](MILESTONE_20M.md)，不同符号/幅度或 50 m 未验收。

## 1. 默认演示与综合验收包

| 项目 | 默认 contact_demo_buffered | 最新综合 20 m |
|---|---|---|
| 真实/标定测量轮直径 | 左右均 80/80 mm | 左右均 81/80 mm |
| 扫描轴相对名义轴位置 | 固定偏移 0 | 横向 +20 mm、竖向 −20 mm |
| 固定轴方向 | 名义方向 | 绕 y +1 mrad、绕 z −1 mrad |
| 轨道共同起伏/左右高差 | 2 mm 档/2 mm 档 | 同档，不同未调参种子 |
| 动态姿态 | 轮轨接触产生 | 轮轨接触产生，并与固定安装组合 |
| 传感器噪声 | 关闭 | 假设 shot_read_prnu_v1 开启 |
| 镜头/响应 | +0.6% 边缘畸变、增益 2.4 | 同基线，但使用综合包自己的标定 |

两者有效目标均 [0,20] m、构造区 [−2.5,22.5] m，名义速度 0.2 m/s、编码器估计螺距 0.6 m、原始采 250°、输出 240°。默认启动器不会自动切换综合包。

## 2. 轮径偏差：机器人不知道真实距离

测量轮实际直径 D_true 决定碰撞与实际滚动，标定直径 D_hat 与量化编码器决定估计里程。左右两路分别换算后平均；扫描伺服和停车都跟随估计值，控制不读取 Gazebo 世界位置。理想无滑移且两侧比例相同：

```text
s_true = s_hat × D_true / D_hat
pitch_true = 0.6 m × D_true / D_hat
```

| 实际/标定 | 估计走 3 m 的理想真实距离 | 理想真实螺距 |
|---|---:|---:|
| 79/80 mm | 2.9625 m | 0.5925 m |
| 80/80 mm | 3.0000 m | 0.6000 m |
| 81/80 mm | 3.0375 m | 0.6075 m |

上表不是动态实测值；起伏、接触和量化另有影响。D3 从公开图像估计相对编码器尺度，不能解释为准确恢复真实轮径或外部绝对尺度。配置参数是 `prepare_contact_demo.py --odo-truth-mm LEFT RIGHT --odo-calibration-mm LEFT RIGHT`，详见 [WHEEL_ERROR](WHEEL_ERROR.md) 和 [RELATIVE_SCALE](RELATIVE_SCALE.md)。生成配置分别写 `truth.odo_left_diameter_m/odo_right_diameter_m` 与 `calibration.odo_left_diameter_m/odo_right_diameter_m`，单位为米；不要把 200 mm 承重轮的 `wheel_diameter_m` 当作 80 mm 测量轮。

## 3. 扫描轴误差与车体姿态

`--mount-offset-mm DY DZ` 和 `--mount-tilt-mrad Y Z` 定义固定装配，生成端写入 `truth.mount.dy_m/dz_m`（米）及 `tilt_y_rad/tilt_z_rad`（弧度）。车体姿态 → 固定安装 `Rz(tilt_z) Ry(tilt_y)` → 扫描旋转依次作用，相机与光源共转；固定偏移随车体旋转，不在世界坐标直接叠加。

`--track-chord-mm {0,2,5}` 指 10 m 弦最大矢度档位，`--track-cross-level-mm {0,2,4}` 指左右高差/三角坑约束档位，`--track-seed` 控制谱的空间随机实现；不是时间 Hz 振动输入。由实际摩擦/导向接触产生升沉、横滚、俯仰。当前支架刚性，没有额外独立支架柔性振动；无 IMU 或真实姿态输入。模型细节见 [MOUNT_ERROR](MOUNT_ERROR.md) 和 [机构设计](design/SIMULATION.md)。

## 4. 噪声与光学标定

综合包使用 [sensor_noise_assumed.yaml](../src/ssb_core/config/sensor_noise_assumed.yaml)：20 e⁻/DN、读出 8 e⁻ RMS、暗电流 100 e⁻/s、偏置 4 DN、PRNU 0.5%。这些不是 DALSA 实测规格；未模拟 MTF/离焦、温漂或热像素。噪声在曝光/像素积分后、量化前加一次，按曝光与列身份确定，批大小变化不改变结果。

安装或光学条件改变后，使用新配置、世界和新标定，不能只编辑旧 YAML。独立居中夹具仅标定镜头、暗场和平场，不给重建提供安装外参。噪声综合包再以独立靶影像估计校正，私有种子/PRNU 真值不公开。细节见 [SENSOR_NOISE](SENSOR_NOISE.md)。

## 5. 生成并运行综合场景（WSL）

先按 [ASSETS](ASSETS.md) 从公开原图生成默认包，完整构建并加载 ROS/工作区。下面入口固定注入表中综合量级，`--seed` 更换轨道与噪声实现；输出必须为新目录。正式留出应选择未参与调参的种子，不把复现种子 20270119 当成新盲验。

```bash
python3 tools/prepare_combined_20m.py \
  --demo local_data/stage_b/contact_demo_buffered \
  --sources local_data/stage_b/sources \
  --work local_data/stage_b/COMBINED_WORK_NEW \
  --output local_data/stage_b/COMBINED_BUNDLE_NEW --seed 20270119 \
  --reuse-lining > /tmp/ssb_combined_generation.log 2>&1
```

`--reuse-lining` 校验已有完整 25 m 包并复用衬面，重新生成轨道/机器人和独立标定；去掉该参数从原图重新生成衬面。已有裂缝形状/宽度不变。新包可独立迁移，不能引用准备目录。此入口内部标定仍使用 WSL 包装环境，不宣称原生综合生成已验收。

先做一米或三米功能检查，再运行完整 20 m。联合 GUI：

```bash
tools/run_mission.sh --gz-gui --relative-encoder-scale \
  --demo local_data/stage_b/COMBINED_BUNDLE_NEW \
  > /tmp/ssb_combined_mission.log 2>&1
```

选择 Wall coverage，目标起点 0、长度 20。相对尺度开关同时扩大公开规划支撑，不意味着控制器知道真实轮径。

完整无界面冻结验收：

```bash
SSB_OFFLINE_WORKERS=4 SSB_D2_SPACING_M=.1 SSB_D2_HEIGHT=256 \
SSB_D2_Q_SHIFT_MM=10 SSB_ADAPTIVE_ATTITUDE=1 SSB_SURFACE_RELIEF=1 \
SSB_SLOW_TRANSLATION=1 SSB_RELATIVE_ENCODER_SCALE=1 \
bash tools/run_d3_holdout.sh sessions/COMBINED_NEW \
  local_data/evaluation/COMBINED_NEW 0 20 local_data/stage_b/COMBINED_BUNDLE_NEW \
  > /tmp/ssb_combined_holdout.log 2>&1
```

脚本要求干净已提交源码，完整构建后声明协议，再采集、实际独立重成像、验收原始数据、D1、公开 D2/D3 和独立网格评价。它不是联合 GUI 性能脚本；完整成图/CPU 与裂缝专项还需按 [STAGE_D](STAGE_D.md) / [里程碑诊断](MILESTONE_20M_DIAGNOSTICS.md) 执行，不能仅凭协议脚本结束宣称整图已核验。

## 6. 只改某个参数时

`prepare_contact_demo.py` 可指定轮径、固定安装、起伏和柔性量级，详见其 `--help`。它生成新的派生目录，随后用 `python3 -m ssb_tools.demo_bundle --demo 派生目录 --output 新包目录` 导出全部依赖。不要在含硬链接的旧包原位修改共享资产。改安装应 `--calibrate`，噪声需独立生成和标定；任务开始前检查物理装配与光学身份。

这些是生成阶段参数，目前没有支持在运行中任意改变真实轮径/安装误差的 launch 热更新接口。后续仅组合失败时补针对性诊断，不默认重复全部单项矩阵。
