# 阶段 B：现行场景、操作与验收

更新：2026-10-01。设计要求见 [DESIGN.md](../DESIGN.md)；按时间顺序的开发过程、选材调查和各轮测量见 [history/](history/README.md)。

## 1. 状态

已实现：

- 20 m 隧道（含缓冲共 23 m）、管片、全部已填的板缝、Concrete034 高清背景、矢量裂缝。
- 刚体轮轨接触：前轮驱动，两个后轮编码器控制扫描。
- OptiX 扫描条光、0.6% 镜头畸变、标靶标定与离线校正。
- GUI 预览：同源纹理、车载工作灯、带阴影的条光、可选眩光。
- RViz：轻量环境和轨道车显示、内嵌任务面板、统一原始采集控制（§10）。

有 3 m 短程完整采集和重放验收（§7）。2026-10-01 用户已确认当前画质作为后续 20 m 采集的基线（§7.1）。

未完成：

- 完整 20 m 任务的画面覆盖、资源与性能验收。
- 完整 20 m 采集（阶段 C），拼接与全局优化（阶段 D）。
- 其余设计层面的待办见 DESIGN §14。

## 2. 默认演示

```bash
tools/run_gz_gui.sh                       # 会话自动命名为 sessions/gui_<时间>
tools/run_gz_gui.sh sessions/my_run       # 指定会话目录（不得已存在）
tools/run_gz_gui.sh SESSION path/to/capture.yaml   # 其他演示目录，世界与标定取同目录
```

演示资产是 `local_data/stage_b/contact_demo/`，内含 `capture.yaml`（含私密光学密钥）、`scene.json`、`world/world.sdf`、`calibration.json` 和 `gui.config`，几者成套使用。

运行流程：

1. 启动前用 `ssb_optical_identity` 检查配置与标定的光学身份，不匹配就不启动 Gazebo。可用 `SSB_OPTICAL_CALIBRATION` 指定另一份标定。
2. 服务器和 GUI 分别启动，初始暂停，点击 Play 开始。
3. 先静置 2 s，然后车体从 x=3 m 按缓起停剖面行驶 3 m（0.2 m/s、20 rpm、28.444 kHz）。
4. 关闭 GUI 后，脚本停止服务器、等待数据全部落盘，用 `tools/check_session.py` 检查完整性，只保留原始图像；畸变校正和平场补偿在拼接前另行执行（§6）。

日志在 `local_data/gui_logs/<时间>/`。WSL 下需要私有 Mesa（`SSB_MESA_PREFIX`，默认 `~/opt/agv-mesa-25.2.8/install`），启动器是 `tools/with_mesa_runtime.py`。

**生成新演示**：用现存资产派生一份新的 3 m 接触演示，`--calibrate` 会同时渲染标靶并拟合标定：

```bash
python3 tools/prepare_contact_demo.py --output local_data/stage_b/NEW_DEMO --calibrate
```

默认输入是 `gui_optics_v11/capture.yaml` 和 `gui_strip_shadow_final_v10/world/world.sdf`。脚本会补上接触配置、装配高度和私密密钥，并重新生成轨道。

**可移植资产包**：把演示及其全部依赖导出为一个相对引用的独立目录，可以复制到新克隆的仓库里使用：

```bash
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.demo_bundle \
  --demo local_data/stage_b/contact_demo --output /tmp/subway_demo_bundle
```

复制为新克隆的 `local_data/stage_b/contact_demo` 即可运行。资产包含有生成端真值（HMAC 密钥、畸变），只用于生成采集，不能作为盲重建评估的输入。已验证的副本是 `local_data/stage_b/review_portable_relocated/`（327 个文件，约 1.8 GiB）。该历史副本仍从 −130° 开始，现行 `contact_demo` 从 180° 开始；恢复资产包后须核对起始相位和配置哈希。要保留新相位，应从现行演示重新导出成套资产包。

## 3. 现行场景与成像配置

| 项目 | 现行取值 | 来源 / 资产 |
|---|---|---|
| 隧道 | 内半径 2.75 m，轴线 z=2.015 m，x∈[−1.5,21.5] m，有效区 [0,20] m | `src/ssb_core/config/stage_b.yaml` |
| 管片与板缝 | 环宽 1.2 m，6 块（10°+2×67°+3×72°），错缝 18°；倒角 3 mm、槽宽 10 mm、深 35 mm，缝区总宽 16 mm，1 mm 圆角，全部填灰砂浆；网格 641773 个三角形，角度步长 0.5° | `stage_b_scene.yaml`；`geometry_b2_v4`；砂浆纹理 `filler_v2` |
| 背景 | Concrete034，0.1 mm 生成网格，GPU 按配方生成 1024² 块；亮度 0.8，增益 0.533；4 种保持抹痕方向的变换；0.45 m 内同向重合 ≤25%；painted_plaster_wall 大尺度明暗层 | `stage_b_material_set.yaml`；`c034_DC4` |
| 裂缝 | 60 个实例（细长 48、细短 9、网状 3），一条 10 m 主干（细化后约 11.2 m）；宽度截断正态 μ0.4/σ0.1 mm、范围 0.2–0.6 mm；`cavity_v2` 腔内模型，有效深度 ×0.8，中位约 0.28 mm | `defects_dev_v5`（快照来源 `defects_review_refined_v2`、`defects_review_base_v1`），索引格 10 mm，旧 float 线段格式 |
| 扫描光源 | 20×20 mm COB，2×2 高斯点；轴向 −0.115 m；光斑半高宽 1.2×0.12 m；弱反射补光 0.002 | `scene.json` 的 `lamp` |
| 相机 | 4096 像素、7.04 μm，90 mm 镜头对焦 2.75 m，视场 0.852 m；8 μs 曝光；k1=0.006 枕形畸变 | `capture.yaml` |
| 采样档位 | 背景走解析路径，3 个曝光时刻，2×2 纹理足迹积分；临界几何每时刻 16 条 N 车射线；复杂裂缝每时刻 64 条（可选 32 性能档）；自适应关闭 | `stage_b_optics --integrated --area-samples 16 --area-pattern rooks --time-samples 3` |
| 机器人 | 120 kg，10 个刚体；前驱、后编码器，四个导向轴承；底座 0.3 m、轴高 1.715 m | `capture.yaml` 的 `robot`、`contact`；`world.sdf` |
| 纹理预算 | OptiX GPU 2 GiB、CPU 1 GiB；GUI 1280 MiB（现行 140 块约 746 MiB） | `stage_b_scene.yaml` 的 `resources` |
| GUI 预览 | 同源 Concrete034 分块底色（x=3–6 m 处 1 mm，其余 2 mm），裂缝 4 倍子像素预览；环境散光 0.6；工作灯和条光见 DESIGN §7.5 | `gui_c034_v1`、`gui_strip_shadow_final_v10` |

所有资产目录都在 `local_data/stage_b/`。素材原图在 `local_data/stage_b/sources/`（concrete034、grey_plaster、painted_plaster_wall），各目录的 `downloads.json` 记录来源、许可和哈希。这些原图是手动下载的，没有自动下载命令。

## 4. 重新生成资产

除非另行说明，命令都在仓库根目录执行，需要先 `source install/setup.bash`，或设置 `PYTHONPATH=src/ssb_tools`。输出目录已存在时工具拒绝覆盖，失败的输出会标记 `FAILED`，复现时请换用新目录。以下是现行资产的生成链。

```bash
CFG=src/ssb_core/config/stage_b.yaml
SPEC=src/ssb_tools/config/stage_b_scene.yaml
OUT=local_data/stage_b

# 1. 背景配方（Concrete034）和填缝砂浆纹理
python3 -m ssb_tools.stage_b_runtime_surface prepare-set --set src/ssb_tools/config/stage_b_material_set.yaml \
  --sources $OUT/sources --config $CFG --spec $SPEC --output $OUT/NEW_SURFACE
python3 -m ssb_tools.stage_b_runtime_surface prepare-filler \
  --downloads $OUT/sources/grey_plaster/downloads.json --output $OUT/NEW_FILLER

# 2. 裂缝：基础布局 → 细化 → 有效深度（每步保存输入快照）
python3 -m ssb_tools.stage_b_defects --config $CFG --spec $SPEC \
  --long-catalog assets/cracks/generated/long_crack_candidates_v1.catalog.json \
  --short-catalog assets/cracks/generated/crack_candidates_v1.catalog.json --output $OUT/NEW_BASE
python3 -m ssb_tools.stage_b_defects --refine $OUT/NEW_BASE --spec $SPEC --output $OUT/NEW_REFINED
python3 -m ssb_tools.stage_b_defects --depth $OUT/NEW_REFINED --spec $SPEC --output $OUT/NEW_DEFECTS

# 3. 隧道几何与世界
ros2 run ssb_tools prepare_stage_b_scene --config $CFG --spec $SPEC --output $OUT/NEW_GEOMETRY

# 4. 绑定光学场景，生成采集配置（含私密光学密钥）
ros2 run ssb_tools stage_b_optics --config $CFG --spec $SPEC --geometry $OUT/NEW_GEOMETRY \
  --surface $OUT/NEW_SURFACE/surface.json --defects $OUT/NEW_DEFECTS/defects.json \
  --filler $OUT/NEW_FILLER/filler.json \
  --integrated --area-samples 16 --area-pattern rooks --time-samples 3 --output $OUT/NEW_OPTICS

# 5. GUI 预览：同源分块底色，然后派生环境照明和机器人世界
python3 -m ssb_tools.stage_b_gui --scene $OUT/NEW_OPTICS/scene.json \
  --world $OUT/NEW_GEOMETRY/world.sdf --output $OUT/NEW_GUI
python3 -m ssb_tools.stage_b_gui_world --world $OUT/NEW_GUI/world.sdf \
  --config $OUT/NEW_OPTICS/capture.yaml --spec $SPEC --output $OUT/NEW_GUI_LIGHT --mode lighting

# 6. 接触演示和标定（--world / --config 指向上面的新产物）
python3 tools/prepare_contact_demo.py --world $OUT/NEW_GUI_LIGHT/world.sdf \
  --config $OUT/NEW_OPTICS/capture.yaml --output $OUT/NEW_DEMO --calibrate
```

说明：

- 新生成的裂缝索引用 double 端点格式（`xq64_radius32_le`，20 m 场景约 35 MB，预算 64 MiB）。现行 `defects_dev_v5` 是旧 float 格式，3 m 演示精度足够。
- 重新生成会得到新的资产哈希和新的光学密钥，旧标定随之失效，必须重新标定。
- 重复度可独立测量：`python3 -m ssb_tools.stage_b_repetition --surface SURFACE.json --area 0 21 -6 6 --size 3 --windows 150`。
- `stage_b_tag_cracks` 加 `stage_b_optics --adaptive` 是显式可选的加速档，要求在背景烘焙后加入裂缝保护标记；现行配方模式不使用。

## 5. 采集、重放与验收命令

```bash
# 无 GUI 采集（世界与配置成对）
SSB_WORLD=$PWD/local_data/stage_b/contact_demo/world/world.sdf \
  tools/run_gz.sh sessions/b_capture local_data/stage_b/contact_demo/capture.yaml

# 改变批大小离线重放，检查原始图像和元数据逐字节一致
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_render \
  --config local_data/stage_b/contact_demo/capture.yaml --session sessions/b_replay \
  --poses sessions/b_capture/evaluation/pose_stream.bin --batch-rows 333
ros2 run ssb_tools validate_stage_b_smoke sessions/b_capture --compare sessions/b_replay

# 接触动力学：只跑物理，不初始化 OptiX；然后独立检查
tools/run_contact_dynamics.sh sessions/dyn CONFIG WORLD
python3 -m ssb_tools.validate_contact sessions/dyn_dynamics --config CONFIG

# 两个 Gazebo 插件的真实服务器回归（含阶段 A 默认命令和装配不匹配拒绝）
python3 tools/test_gazebo_plugins.py --output /tmp/ssb_plugin_regression_NEW

# 指定姿态的光学检查与吞吐测量（不是编码器采集验收）
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_probe --config CONFIG --x 4 --theta 0.2
```

`validate_stage_b_smoke` 独立检查以下各项，报告写到 `evaluation/reports/stage_b_smoke.json`：

- 源码与二进制身份、全部资产哈希、编码器与门控时序、有效区无缺行。
- CPU 三角形交点：用 double 源网格独立计算。
- GPU/CPU 纹理预算、重放逐字节一致。

它不是光度、镜头分辨率或外观验收。采集的 `evaluation/` 目录归档场景、背景和缺陷真值，后续拼接不得读取。

## 6. 光学标定与校正

```bash
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.optical_bench --config CAPTURE.yaml --output BENCH --render
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.optical_calibration fit --bench BENCH/bench.json --output CALIBRATION.json
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.optical_calibration apply --session SESSION \
  --calibration CALIBRATION.json --output CORRECTED
```

标靶组成：暗场、均匀亮场、2 cm 间距的条纹，以及留出验证用的错开 7 mm 的条纹和不同反照率、角度的亮场，全部经过同一 OptiX 照明和响应路径。标定只读取标靶图像，不读取渲染射线或命中记录。方法见 DESIGN §9.4。

现行演示的标定结果：

| 指标 | 数值 |
|---|---|
| 留出标靶几何最大残差 | 0.01958 px |
| 亮场列变异系数 | 6.05% → 0.303% |
| 有效输出列 | 98.58%，边缘不外推 |

这些结果对应理想无噪声相机和固定标定距离，不代表真机精度。

## 7. 当前验收结果（2026-10-01）

最终采集 `sessions/review_portable_gui_final`，用的是移动到新目录后的可移植资产包，GUI 开启、光晕关闭：

| 项目 | 结果 |
|---|---|
| 行程与行数 | 实际前进 2.99999951 m，284445 行；有效区 x∈[3.1,5.7] m 无缺行 |
| 成像进度实时率 | 0.99387（统计从首个位姿开始，不含资产加载和 GPU 初始化） |
| 重放 | 333 行批次重放 `sessions/review_portable_replay`，81 个原始文件逐字节一致 |
| 采集检查 | `validate_stage_b_smoke` 22 项全部通过；CPU 独立射线命中最大误差 0.1972 μm |
| 接触检查 | 全部通过；最大纵向滑移速度 0.0776 mm/s，最大扫描跟踪误差 0.010502 rad |
| 离线校正 | 本次历史验收在退出后校正 70 个块；现行采集已改为拼接前单独执行 |
| 资产包 | 327 个文件哈希在移动后全部匹配，Gazebo 日志无资源加载错误 |

其他已验证项（会话数据已清理，数字记录在 [history/STAGE_B_DEVLOG.md](history/STAGE_B_DEVLOG.md)）：

- **20 m 接触动力学**：行程 19.9999995 m，最大滑移速度约 7.8e-5 m/s，扫描跟踪误差 ≤0.0105 rad。
- **标定轮径 ±1%**：实测螺距 0.593980 / 0.605980 m，符合 `0.6 × D_actual/D_calibrated`。
- **去掉摩擦**：车体不前进，估计里程和扫描推进均为零，说明没有隐藏的强制平移。
- **弱反射补光 0.002**：每像素只增加 0 或 1 个灰度级，长裂缝对比度变化可忽略。
- **采样档位**：板缝边缘 N 车 16 条/时刻相对 16×16×3 参考 RMSE 0.85 DN；复杂裂缝 64 条/时刻相对 128 条参考 RMSE 0.54 DN。
- **长距离光学**：0/100/150 m 平移后，命中与 double 源网格误差小于 5 μm，跨块和改批大小重放一致。

### 7.1 画面复核与基线冻结（2026-10-01）

用户确认：“当前画质可以作为基线”。本轮保持现行配置，复核背景、裂缝、填缝和光照；后续 20 m 采集采用这一画质基线。

复核页为 `local_data/stage_b/final_review_20261001/index.html`，输入哈希和验收记录在同目录的 `evaluation/review.json`。页面包含 8 组原像素对照及 2 张 Gazebo 观察截图：5 组来自已归档的 3 m 编码器采集，另有网状裂缝、较宽裂缝和弱补光对照的指定位置抽查。原始与校正图使用同一显示曲线，无自动对比度或锐化；补充抽查不代表完整编码器采集。

| 检查 | 结果 |
|---|---|
| 归档数据与校正文件 | 70 个块、284445 行；原始和校正文件哈希全部匹配 |
| 选取行重新校正 | 与归档校正结果最大差异 0 DN |
| 有效像素比例 | 98.584% |
| 原始饱和像素比例 | 约 0.0000111% |
| 弱补光 0.002 与关闭对照 | 同位姿原始像素差为 0 或 1 DN，平均 0.333 DN |
| 复核工具峰值常驻内存 | 约 301 MiB，按块读取 |

校正数据保留 float32，最大值约 267.52 DN；显示 PNG 裁剪到 0–255，不修改测量值。Gazebo 观察确认了条光旋转时的顶部光斑和底部支架局部照明；这次截图运行关闭了成像，不能作为新的成像实时率测量。性能基线仍是 §7 的 3 m GUI 采集实时率 0.99387。

以下示例使用已完成离线校正的历史验收会话。新任务只保存原始数据；在运行复核工具前，须按 §6 手动执行 `optical_calibration apply`，将结果写到该会话的 `processed/optical/`。该目录已存在时不要重复覆盖。

```bash
SSB_REVIEW_SESSION=sessions/mission/YOUR_SESSION
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.optical_calibration apply \
  --session "$SSB_REVIEW_SESSION" --calibration local_data/stage_b/contact_demo/calibration.json \
  --output "$SSB_REVIEW_SESSION/processed/optical"
```

可重新生成原像素复核材料（输出目录不得已存在）：

```bash
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.stage_b_review \
  --session sessions/review_portable_gui_final \
  --demo local_data/stage_b/contact_demo \
  --output local_data/stage_b/NEW_REVIEW --supplementary-probes
```

`stage_b_review` 是当前资产的指定样例复核工具，内部固定了背景/板缝坐标，以及 crack_000/018/056 等裂缝编号；补充探针还使用 crack_020/047。换缺陷布局、任务区间或 20 m 场景时，须重新选择可见样例并调整坐标，不能将该工具当作通用全隧道成果浏览器。

新生成的页面默认等待人工确认，不继承本轮验收；Gazebo 观察截图需另行添加。复核使用缺陷真值选择样例，仅限 `evaluation/`，不得供盲重建使用。报告冻结演示配置、世界、场景、标定、数据索引及归档光学资产的哈希；不向公开重建配置导出光学密钥。

冻结参数包括 0.2 m/s、0.6 m/圈、28.444 kHz、复杂裂缝每时刻 64 条采样、Concrete034 背景、现行裂缝颜色与深度系数、0.6% 畸变及独立标靶校正。此次局部画面认可不替代完整 20 m 覆盖、约 10 m 长裂缝连续性和精确 0.2/0.6 mm 边界样本的专项验收；§8 的模型局限仍然适用。

为保留版本库中的画质验收基线身份，记录验收时 `contact_demo/` 的 SHA-256（本地资产不纳入 Git；后续扫描初始相位修改另见 §10）：

| 文件 | SHA-256 |
|---|---|
| `capture.yaml` | `b59578a0b0b8c2f45cae50035ad1428ebba1232a4daac6fe002433387c7dbbdc` |
| `scene.json` | `72e8f44a84b01257a0071c6475eed4df6068f685982e0849c4689e0dbdcda6a4` |
| `world/world.sdf` | `8c5e073855b48be73ce230bff52a78394cebabfa5f90993c17a5e772c1f76421` |
| `calibration.json` | `c235d7031d7e2a40c48f6922b8749307e384a44beddb0dc868d9c1f73e736105` |

## 8. 已知局限

- 裂缝是反照率近似，没有几何凹陷、自遮挡或随灯光方向的明暗不对称；有效深度是合成参数。
- 光学网格不含机器人，不能验证镜筒或支架挡光。GUI 能显示的遮挡和光照与 OptiX 采集无关。
- 车载工作灯只在 GUI 中照明，采集中只以 0.002 弱补光近似；没有多次反射，没有绝对光度标定。
- 没有镜头 MTF、离焦和噪声；暗场为零。
- 背景只有一块 0.57 m² 素材，重复只能控制、不能消除。全区 4200 个窗口的高通互相关中位数 0.20，>0.8 的占 0.4%，拼接端需要做峰值唯一性防护。
- 只验证了直轨。性能只在 3 m 短程测过，不外推到完整 20 m。

## 9. 待完成

1. 阶段 C：完整 20 m 采集，包括覆盖图、资源和吞吐报告，以及 20 m 场景的 double 格式裂缝索引。
2. 阶段 D：展开、重叠匹配、全局优化、分块重采样。
3. 按需加入：裂缝几何凹陷参考、机器人遮挡、镜头退化、未填/破损板缝的新模型、弯轨。

## 10. RViz 与任务控制

```bash
tools/run_mission.sh             # RViz 显示，Gazebo 服务器按任务启动
tools/run_mission.sh --gz-gui    # 同时显示 GZ GUI
# 可指定成套的演示与会话根目录：
tools/run_mission.sh --demo local_data/stage_b/contact_demo --output-root sessions/mission
```

现行演示和新生成的接触演示均从正下方 θ=180° 开始，先空转到右下方 θ=240° 后开门，经顶部到左下方 θ=480° 关门；底部 120° 始终不曝光。RViz 待机和初始化姿态读取同一配置角度。旧演示为 −130°，本次仅更改起始相位：场景、材质、灯具和测得标定未变，光学签名仍兼容，历史会话不修改。当前 `capture.yaml` SHA-256 为 `ce08fc594588e8ba14fcdcf5bb1a1986cf4c9b1f3ada923f57944720749c3be7`，同时更新了世界输入归档与其 manifest。最新真实 GZ＋OptiX 回归确认待机为 180°，首行实际曝光角约 240.0015°，进入右下门控前没有生成有效行。下述性能表已使用这一相位，并包含英文面板和原始图像缩略图。

面板设置轨道纵向起点和前进距离，分别显示车体预计终点和全角度覆盖保守估计，界面全部使用英文，提供 Start、Pause、Resume、Stop（快捷键 Ctrl+Alt+S/P/R/E）。当前范围为 0–20 m，最短任务 1 m（按用户要求，避免过短采集）；开始前核对轨道及视场缓冲余量。运行中锁定输入。点击 Start 后，起点在生成任务世界时初始化，相当于将车体直接放到指定起点；只修改输入框不会移动车体；使用现行 0.2 m/s 标称速度，根据距离生成缓起停剖面，保留扫描由后轮编码器驱动的逻辑。行驶距离显示编码器估计值，轮径误差仍会影响扫描。当前终止按运动剖面的计划时长执行；带轮径偏差的里程目标停车控制尚未实现。

有效曝光验收区 `acceptance.valid_x_m` 在生成任务时确定，按扫描初始相位、门控空转段和起停坡段向内收，另外保留 10 mm 名义跟踪余量。现行 3 m 任务起点为 3 m 时，区间为 [3.11,5.79] m；1 m 任务也有非空验收区。这是头部位置的无缺行检查区，不是车体行程或全角度壁面覆盖区，不从成像结果反推，不修改验收器。轮径或安装误差较大时仍可能判失败，须独立分析，不能静默继续缩区。

统一管理器 `ssb_tools.mission_manager` 接收 `/ssb/mission/command`，发布 `/ssb/mission/status`。每条命令带独立 ID；拒绝越界、活动任务中重新开始和重复请求。null、非数值及非有限的起点/距离在进入工作队列前被拒绝，保留原任务状态；队列已满时返回带请求 ID 的拒绝回执。面板发送前检查接收端是否存在，并在 90 s 内等待回执（覆盖 60 s 启动等待）；超时后清除等待状态，按最新任务状态恢复可用按钮并显示提示，不自动重发。管理器连接中断时取消等待，重连后重新按状态启用控件。暂停采用 Gazebo 冻结仿真时间，保留计数和扫描相位；已有曝光继续成像落盘。结束任务先暂停，再通知自己启动的服务器退出并排空队列，保存尾块；提前结束保持 `session.json` 的 `motion.complete=false`，界面显示“Stopped early; raw data saved”。此时完整任务检查脚本拒绝会话是预期行为。

管理器分别报告运行、已暂停、等待落盘、原始采集完成或失败；不在采集或任务退出时运行畸变校正和平场补偿。已保存行数只计入已写盘且回读哈希通过的块，尾块关闭后计入；成像滞后是最近输入位姿与写线程已处理曝光时间之差，不等于待耐久化尾块的时间。最终数据状态和哈希仍以会话清单为准。会话生成失败或后台成像失败会显示错误；每次重新开始创建不同的会话和任务输入目录。

RViz 复用 SDF 的视觉几何、颜色和预览贴图，转换为 Collada 并缓存到 `local_data/rviz_preview/`，每张纹理最长边 512 px；不加载 0.1 mm 光学纹理。环境 MarkerArray 缓存后以 1 Hz 重发，防止新任务时间归零或手动 Reset 清空显示后场景丢失；不重读或重新烘焙资产。车辆各 link 由 10 Hz TF 更新，完整原始图像不经 DDS。显示话题 `/ssb/sim_truth/scene`、`/ssb/sim_truth/joint_states` 和坐标系 `sim_truth/*` 只供观察；位姿来自仿真物理状态，严禁用于盲重建。`/clock` 与 Gazebo 仿真时间一致，RViz 启用 `use_sim_time`，支持新任务时的时间归零。RViz 光照只是观察效果，不模拟采集条光的光度和阴影。

任务控制面板位于右侧，左下方（Displays 下方）的 Live Capture Preview 显示最近已保存原始块的缩略图及行号范围；默认一个块为 4096 行。只读取写完且回读哈希通过后原子重命名的 `.u8`，不读取未完成块或依赖采集结束的索引。预览按面积平均缩小至最多 512×512、保留 Mono8 DN，每秒最多更新一次，同一块复用编码缓存；PNG 与会话/行号信息通过单条低频消息 `/ssb/mission/preview` 发送，消息小于 512 KiB。任务切换清空旧图，按会话身份拒绝延迟的旧图；暂停和结束后显示最后已保存图。预览不做畸变、平场、对比度或锐化校正，原始文件不变；文件访问错误只显示预览错误，不中断采集。

运行日志在 `local_data/mission_logs/`，各任务私有输入、世界和服务器日志在 `local_data/mission_runs/`；任务状态变更与最终真值写入会话的 `evaluation/mission.json`。关闭 RViz 会结束活动任务并等待原始图像保存；任务自然结束后 RViz 保持打开。可选的 GZ GUI 保留最后画面，新任务复用它连接新的服务器。

本轮真实 Gazebo＋OptiX 联调验证：越界拒绝、任意扫描相位暂停/继续、暂停后无新增曝光、活动任务中重新开始被拒、提前结束尾块保存与未完成标记、重新开始的会话隔离。1 m 暂停任务采集 99568 行，333 行批次重放的 31 个原始/元数据文件逐字节一致；最短 1 m 的重新开始任务也正常完成，低于 1 m 的任务被拒绝。缩略图的行号、PNG 尺寸和实际面板显示均通过检查；所有采集位姿时间间隔仍为 1 ms。另测 3 组 3 m 任务，各 284445 行，原始文件与会话清单哈希全部通过，RViz 预览无资源加载错误。

| 界面 | 动力学实时率 | 成像实时率 | 进程 RSS 合计峰值 |
|---|---:|---:|---:|
| GZ GUI | 1.000 | 0.994 | 3.34 GiB |
| RViz | 1.000 | 0.995 | 2.79 GiB |
| GZ GUI＋RViz | 1.000 | 0.995 | 3.78 GiB |

测量记录 `local_data/evaluation/mission_ui_20261001_cleanup/tmp/ssb_ui_preview_acceptance/report.json`；RSS 为测试期间各自进程树的采样合计，包含共享页重复计数，不是独占内存。OptiX 的纹理分配峰值约 2.00 GiB，与 2 GiB 配置预算一致，未测量整机显存峰值。实时率从首个采集位姿开始，排除资产加载和 GPU 初始化，本轮未执行采后校正；暂停测试不用于性能表。上述结果仍是 3 m 短程，完整 20 m 待验收。

黑屏排查：RViz 会在时间回拨及手动 Reset 时清空 Marker；原来仅发布一次场景，清空后不会自动恢复。现行 1 Hz 缓存重发解决这条路径。负向验证仅在测试进程中禁用重发定时器，恢复旧行为：不打开 GZ GUI，Reset 后画面退化为纯背景（各通道标准差为 0），被空场景检查拒绝；记录在 `local_data/evaluation/mission_ui_20261001_cleanup/tmp/ssb_marker_restore_negative/report.json`。在单开 RViz 和 GZ GUI＋RViz 两组实际采集中，分别操作视角、Reset 和任务时间归零，渲染缓冲截图均保留隧道和车辆，日志无渲染资源错误。双界面另外通过实际面板快捷键验证开始、暂停、继续、提前结束和重新开始；新会话不生成 `processed/optical/`。记录在 `local_data/evaluation/mission_ui_20261001_cleanup/tmp/ssb_mission_integration_final/view_regression.json` 和 `local_data/evaluation/mission_ui_20261001_cleanup/tmp/ssb_rviz_diagnose12/`。没有复现用户所述的所有偶发黑屏，不能据此排除 WSL 显示驱动等其他原因，也不能认定 GZ GUI 是必要触发条件。

任务集成回归对 1 m 暂停/继续任务和 3 m 完整任务执行独立重放，并运行 `validate_stage_b_smoke --compare`，包含有效区检查；提前结束仍按未完成任务处理。在下述相同的 Mesa/ROS 环境中，将测试命令换成 `python3 tools/test_mission_panel.py --output /tmp/ssb_panel_NEW` 可单独测试面板回执丢失/重连；该工具使用假的状态发布者和丢弃指令的接收端，不启动 Gazebo，按实际 90 s 超时验证 UI 恢复。

可重跑集成验收（输出目录必须不存在，需先重建）：

```bash
python3 tools/with_mesa_runtime.py bash tools/with_optix_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib"
  export GZ_GUI_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib"
  export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA QT_QPA_PLATFORM=xcb
  python3 tools/test_mission.py --output /tmp/ssb_mission_NEW --performance
' > /tmp/ssb_mission_NEW.log 2>&1
```

仅做动力学调试可加 `--dynamics-only`，面板会明确标注不采图。可用 `SSB_RVIZ_REVIEW_DIR=/tmp/NEW_REVIEW tools/run_mission.sh` 保存启动后的面板和渲染缓冲截图；此开关启用时还接受 `/ssb/mission/review` 上的 String 消息，内容为文件名前缀（字母、数字、下划线或短横线，1–32 字符）。默认关闭，不产生持续截图负载。截图也不代表采集相机图像。

### 10.1 临时测试数据清理（2026-10-01）

本轮已清理临时任务原始图像、重放副本、过期任务输入和日志，以及 560 个未被现行世界引用的 RViz 缓存文件，释放约 86.04 GiB 已分配磁盘空间。保留现行 `contact_demo` 及标定、已接受的画质基线 `sessions/review_portable_gui_final` 和当前 280 个缓存文件。测试报告、必要截图和会话元数据归档到 `local_data/evaluation/mission_ui_20261001_cleanup/`，删除清单见其中 `cleanup.json`；归档不含这些临时任务的原始图像，重放需要重新采集。该本地归档含生成端私有配置，不纳入 Git，也不作为盲重建输入。

### 10.2 任务数据保留

当前采用手动清理，不自动删除任务。每个 3 m 原始任务约 1.3 GB，20 m 约 8–9 GB；离线 float32 校正、重放副本和复核材料另计。任务原始数据在 `sessions/mission/`，对应生成端私有输入在 `local_data/mission_runs/`，运行日志在 `local_data/mission_logs/`。采集前检查可用空间；保留已接受的画质/验收基线和用户仍在检查的会话。临时试验验证完成后保留小型报告、必要配置和截图，再手动删除不再需要的原始数据及匹配的输入目录；运行中的会话不可清理。可用 `du -sh sessions/mission local_data/mission_runs local_data/mission_logs` 查看增长。

### 10.3 任务验收与回执修复回归（2026-10-01）

132 项 Python 测试通过，包含 1/1.2/3/20 m 的名义验收区、多种初始相位、区内缺行拒绝、畸形输入和满队列回执。真实 Gazebo＋OptiX 的 1 m 暂停/继续和 3 m 完整任务，各经独立重放后通过阶段 B 验收器全部 22 项检查。1 m 任务有效区 [2.11,2.79] m 有 68282 行、零缺行；3 m 任务有效区 [3.11,5.79] m 有 267393 行、零缺行，总行数仍为 284445。起停缓冲区内丢行继续如实报告。

实际 RViz 面板验证了接收端缺失、拒绝回执、管理器断连/重连，以及实际等待 90 s 无回执后重新操作；测试在 91.29 s 检查时已恢复按钮，下一条命令成功接收并回执。报告位于 `local_data/evaluation/mission_audit_fix/capture_regression/report.json` 和 `local_data/evaluation/mission_audit_fix/panel_regression_final/report.json`。回归临时原始数据/重放副本及其私有输入已清理约 4.23 GiB，保留小型配置、报告和必要截图。用户观看的历史会话 `sessions/mission/20261001_145804_982cf426` 保持原配置与哈希，其中原有 [3,6] m 验收区不会追溯修改；修正后的区间用于新生成任务。
