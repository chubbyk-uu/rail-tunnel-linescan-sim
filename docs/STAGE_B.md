# 阶段 B：现行场景、操作与验收

更新：2026-10-01。设计要求见 [DESIGN.md](../DESIGN.md)；按时间顺序的开发过程、选材调查和各轮测量见 [history/](history/README.md)。

## 1. 状态

已实现：

- 20 m 隧道（含缓冲共 23 m）、管片、全部已填的板缝、Concrete034 高清背景、矢量裂缝。
- 刚体轮轨接触：前轮驱动，两个后轮编码器控制扫描。
- OptiX 扫描条光、0.6% 镜头畸变、标靶标定与离线校正。
- GUI 预览：同源纹理、车载工作灯、带阴影的条光、可选眩光。

有 3 m 短程完整采集和重放验收（§7）。2026-10-01 用户已确认当前画质作为后续 20 m 采集的基线（§7.1）。

未完成：

- 完整 20 m 任务的画面覆盖、资源与性能验收。
- 完整 20 m 采集（阶段 C），拼接与全局优化（阶段 D）。
- RViz 任务界面。
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
4. 关闭 GUI 后，脚本停止服务器、等待数据全部落盘，用 `tools/check_session.py` 检查完整性，再自动做离线光学校正，结果写到 `SESSION/processed/optical/`。

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

复制为新克隆的 `local_data/stage_b/contact_demo` 即可运行。资产包含有生成端真值（HMAC 密钥、畸变），只用于生成采集，不能作为盲重建评估的输入。已验证的副本是 `local_data/stage_b/review_portable_relocated/`（327 个文件，约 1.8 GiB）。

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
| 离线校正 | 退出后自动校正 70 个块 |
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

可重新生成原像素复核材料（输出目录不得已存在）：

```bash
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.stage_b_review \
  --session sessions/review_portable_gui_final \
  --demo local_data/stage_b/contact_demo \
  --output local_data/stage_b/NEW_REVIEW --supplementary-probes
```

新生成的页面默认等待人工确认，不继承本轮验收；Gazebo 观察截图需另行添加。复核使用缺陷真值选择样例，仅限 `evaluation/`，不得供盲重建使用。报告冻结演示配置、世界、场景、标定、数据索引及归档光学资产的哈希；不向公开重建配置导出光学密钥。

冻结参数包括 0.2 m/s、0.6 m/圈、28.444 kHz、复杂裂缝每时刻 64 条采样、Concrete034 背景、现行裂缝颜色与深度系数、0.6% 畸变及独立标靶校正。此次局部画面认可不替代完整 20 m 覆盖、约 10 m 长裂缝连续性和精确 0.2/0.6 mm 边界样本的专项验收；§8 的模型局限仍然适用。

为保留版本库中的基线身份，记录 `contact_demo/` 的 SHA-256（本地资产不纳入 Git）：

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

1. RViz 显示与任务面板（DESIGN §13）：隧道和轨道车、任务起点与距离、开始/暂停/继续，以及联合界面的资源验收。
2. 阶段 C：完整 20 m 采集，包括覆盖图、资源和吞吐报告，以及 20 m 场景的 double 格式裂缝索引。
3. 阶段 D：展开、重叠匹配、全局优化、分块重采样。
4. 按需加入：裂缝几何凹陷参考、机器人遮挡、镜头退化、未填/破损板缝的新模型、弯轨。
