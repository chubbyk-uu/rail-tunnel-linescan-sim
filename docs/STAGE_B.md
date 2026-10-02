# 阶段 B：现行场景、操作与验收

更新：2026-10-02。审核修复与最新联合回归见 [REPAIR_PLAN_2026-10-02.md](REPAIR_PLAN_2026-10-02.md)。设计要求见 [DESIGN.md](../DESIGN.md)；按时间顺序的开发过程、选材调查和各轮测量见 [history/](history/README.md)。

## 1. 状态

已实现：

- 20 m 隧道（含缓冲共 23 m）、管片、全部已填的板缝、Concrete034 高清背景、矢量裂缝。
- 刚体轮轨接触：前轮驱动，后轮从动承重，车尾两个弹簧压紧的测量轮（80 mm）带编码器控制扫描。
- OptiX 扫描条光、0.6% 镜头畸变、标靶标定与离线校正。
- GUI 预览：同源纹理、车载工作灯、带阴影的条光、可选眩光。
- RViz：轻量环境和轨道车显示、内嵌任务面板、统一原始采集控制（§10）。

有 3 m 短程完整采集和重放验收（§7）。2026-10-01 用户已确认当前画质作为后续 20 m 采集的基线（§7.1）。

未完成：

- 全局拼接优化、最终接缝融合和完整全图输出（阶段 D）。采后畸变/平场处理、CUDA 初始展开及相邻条带匹配已完成，见 [STAGE_D](STAGE_D.md)。

完整 20 m 原始采集、名义覆盖、独立重放及 Gazebo/RViz 联合 GUI 资源与性能验收已完成，见 [STAGE_C §6](STAGE_C.md#6-完整-20-m-采集验收2026-10-02)。本次不替代真实姿态下的覆盖与最终拼接几何质量评估。
- 其余设计层面的待办见 DESIGN §14。

## 2. 默认演示

```bash
tools/run_gz_gui.sh                       # 会话自动命名为 sessions/gui_<时间>
tools/run_gz_gui.sh sessions/my_run       # 指定会话目录（不得已存在）
tools/run_gz_gui.sh SESSION path/to/capture.yaml   # 其他演示目录，世界与标定取同目录
```

演示资产是 `local_data/stage_b/contact_demo/`，现已直接采用可迁移的完整导出包，内含 `capture.yaml`（含私密光学密钥）、`assets/` 中的光学场景及依赖、`world/world.sdf`、`spec.yaml`、`calibration.json`、`gui.config` 和 `bundle.json`，几者成套使用。运行引用全部相对；`bundle.json.dependencies` 的 `source_file` 和配方 `input_channels` 只是历史来源记录。

`capture.yaml` 的公开 `mission` 段声明有效范围、车体半包络、安全余量和最短距离；管理器校验后发布给 RViz。默认 [0,20] m、0.56 m 半包络加 0.09 m 余量、最短 1 m。扩大范围时必须同时生成足够长的隧道/轨道和光学资产；修改面板范围不等于完成长隧道验收。

运行流程：

1. 启动前用 `ssb_optical_identity` 检查配置与标定的光学身份，不匹配就不启动 Gazebo。可用 `SSB_OPTICAL_CALIBRATION` 指定另一份标定。
2. 服务器和 GUI 分别启动，初始暂停，点击 Play 开始。
3. 先静置 2 s，然后车体从 x=3 m 按缓起停剖面行驶 3 m（0.2 m/s、20 rpm、28.444 kHz）。轨道带 2 mm 档竖向起伏和 2 mm 档水平（差动）分量，车轮带 0.2 mm 静压缩量的聚氨酯柔性，测量轮编码（§11）。
4. 关闭 GUI 后，脚本停止服务器、等待数据全部落盘，用 `tools/check_session.py` 检查完整性，只保留原始图像；畸变校正和平场补偿在拼接前另行执行（§6）。

日志在 `local_data/gui_logs/<时间>/`。WSL 下需要私有 Mesa（`SSB_MESA_PREFIX`，默认 `~/opt/agv-mesa-25.2.8/install`），启动器是 `tools/with_mesa_runtime.py`。

**生成新演示**：用现存资产派生一份新的 3 m 接触演示，`--calibrate` 会同时渲染标靶并拟合标定：

```bash
python3 tools/prepare_contact_demo.py --output local_data/stage_b/NEW_DEMO --calibrate
```

默认输入是完整的 `contact_demo` 包，可用 `--demo /path/to/bundle` 指定迁移后的副本；不依赖 v10/v11 中间目录。脚本重新生成机器人和轨道，默认保留来源包的衬面网格，并自动复用匹配的标定；`--calibrate` 用于重新标定。派生目录还需要下述导出步骤才能再次独立迁移。

轨道起伏与车轮柔性（§11）也在这里设定，二者都会写进配置的真值段并决定世界的生成：

```bash
python3 tools/prepare_contact_demo.py --output local_data/stage_b/NEW_DEMO \
  --config local_data/stage_b/contact_demo/capture.yaml \
  --calibration local_data/stage_b/contact_demo/calibration.json \
  --track-chord-mm 2 --wheel-deflection-mm 0.2
```

- `--track-chord-mm {0,2,5}`：竖向不平顺档位（10 m 弦最大矢度），0 为平直轨；`--track-seed` 设定随机种子。
- `--track-cross-level-mm {0,2,4}`：左右差动（水平）档位，水平最大值和 5 m 三角坑都不超过该值；默认 0。会产生横滚和翘轮，见 §11。
- `--odo-truth-mm 左 右`、`--odo-calibration-mm 左 右`：测量轮真实直径与标定直径（默认都为标称 80 mm），用于轮径误差实验。世界按真实直径生成；事后改配置不重建世界会被物理检查拒绝。
- `--wheel-deflection-mm`：聚氨酯轮静压缩量，默认 0.2，不低于 0.15；0 为刚性轮。会在 DART 中自动标定刚度（需在已 `source /opt/ros/jazzy/setup.bash` 的环境中运行）。
- 沿用现行演示的配置和标定时，光学签名不变（签名不含轨道和车轮），可直接复用标定，不必重新渲染标靶。

**可移植资产包**：把演示及其全部依赖导出为一个相对引用的独立目录，可以复制到新克隆的仓库里使用：

```bash
source install/setup.bash
python3 -m ssb_tools.demo_bundle \
  --demo local_data/stage_b/contact_demo --output /tmp/subway_demo_bundle
```

接触模式资产包同时导出 `spec.yaml` 和 `world/physical_manifest.json`，同步高度场改名后的引用并保留图像内容哈希；导出前后都检查物理世界与配置一致。缺少规格或物理清单的旧包需从完整演示重新导出。

复制为新克隆的 `local_data/stage_b/contact_demo` 即可运行。资产包含有生成端真值（HMAC 密钥、畸变），只用于生成采集，不能作为盲重建评估的输入。2026-10-02 默认包已迁移并在隔离原 `stage_b` 目录后通过 344 个文件哈希、运行引用边界、物理世界、标定身份检查，并从迁移副本派生新演示。约 1.70 GiB 运行依赖（1,827,116,136 字节）。原默认目录保留为 `contact_demo_unbundled_20261002`，历史 `review_portable_relocated` 已删除，不作为现行入口。联合短程采集在本轮修复的集成验收中复核。

## 3. 现行场景与成像配置

| 项目 | 现行取值 | 来源 / 资产 |
|---|---|---|
| 隧道 | 内半径 2.75 m，轴线 z=2.015 m，x∈[−1.5,21.5] m，有效区 [0,20] m | `src/ssb_core/config/stage_b.yaml` |
| 管片与板缝 | 环宽 1.2 m，6 块（10°+2×67°+3×72°），错缝 18°；倒角 3 mm、槽宽 10 mm、深 35 mm，缝区总宽 16 mm，1 mm 圆角，全部填灰砂浆；网格 698922 个三角形，角度步长 0.5°，衬面闭合（漏光审计 0 处，见 §8） | `stage_b_scene.yaml`；`geometry_b2_v5`；砂浆纹理 `filler_v2` |
| 背景 | Concrete034，0.1 mm 生成网格，GPU 按配方生成 1024² 块；亮度 0.8，增益 0.533；4 种保持抹痕方向的变换；0.45 m 内同向重合 ≤25%；painted_plaster_wall 大尺度明暗层 | `stage_b_material_set.yaml`；`c034_DC4` |
| 裂缝 | 60 个实例（细长 48、细短 9、网状 3），一条 10 m 主干（细化后约 11.2 m）；宽度截断正态 μ0.4/σ0.1 mm、范围 0.2–0.6 mm；`cavity_v2` 腔内模型，有效深度 ×0.8，中位约 0.28 mm | `defects_dev_v5`（快照来源 `defects_review_refined_v2`、`defects_review_base_v1`），索引格 10 mm，旧 float 线段格式 |
| 扫描光源 | 20×20 mm COB，2×2 高斯点；轴向 −0.115 m；光斑半高宽 1.2×0.12 m；弱反射补光 0.002 | `scene.json` 的 `lamp` |
| 相机 | 4096 像素、7.04 μm，90 mm 镜头对焦 2.75 m，视场 0.852 m；8 μs 曝光；k1=0.006 枕形畸变 | `capture.yaml` |
| 采样档位 | 背景走解析路径，3 个曝光时刻，2×2 纹理足迹积分；临界几何每时刻 16 条 N 车射线；复杂裂缝每时刻 64 条（可选 32 性能档）；自适应关闭 | `stage_b_optics --integrated --area-samples 16 --area-pattern rooks --time-samples 3` |
| 机器人 | 120 kg；前驱、后轮从动，两个测量轮（80 mm，竖直滑轨，预压 30 N）带编码器，四个导向轴承；车底电池/驱动舱（仅外观，尺寸为估计）；底座 0.3 m、轴高 1.715 m | `capture.yaml` 的 `robot`、`contact`；`world.sdf` |
| 轨道起伏 | 现行演示为 2 mm 档北京地铁谱竖向起伏加 2 mm 档水平分量（种子 20261001）；竖向可选平直或 5 mm，水平可选 0 或 4 mm（§11） | `capture.yaml` 的 `truth.track_irregularity`；`world/track/rail_top_*.png` |
| 车轮柔性 | 现行演示为聚氨酯轮柔性，静压缩量 0.2 mm（标定 SDF 刚度 2.86×10⁶ N/m）；可选 0.15/0.4 mm 或刚性轮（§11） | `capture.yaml` 的 `truth.wheel_compliance` |
| 纹理预算 | OptiX GPU 2 GiB、CPU 1 GiB；GUI 1280 MiB（现行 140 块约 746 MiB） | `stage_b_scene.yaml` 的 `resources` |
| GUI 预览 | 同源 Concrete034 分块底色（x=3–6 m 处 1 mm，其余 2 mm），裂缝 4 倍子像素预览；环境散光 0.6；工作灯和条光见 DESIGN §7.5 | `gui_c034_v1`、`gui_strip_shadow_final_v10` |

所有资产目录都在 `local_data/stage_b/`。素材原图在 `local_data/stage_b/sources/`（concrete034、grey_plaster、painted_plaster_wall），各目录的 `downloads.json` 记录来源、许可和哈希。首次部署使用 `tools/download_demo_sources.py` 从公开网站下载并写入来源清单，然后用 `tools/build_demo_from_sources.py` 从零生成完整演示，见 [ASSETS.md](ASSETS.md)。

## 4. 重新生成资产

除非另行说明，命令都在仓库根目录执行，需要先 `source install/setup.bash`。输出目录已存在时工具拒绝覆盖，失败的输出会标记 `FAILED`，复现时请换用新目录。首次部署优先执行 [ASSETS.md](ASSETS.md) 的下载及完整生成入口。以下为逐模块生成示例；裂缝目录先在本机重定位，避免历史绝对路径。

```bash
CFG=src/ssb_core/config/stage_b.yaml
SPEC=src/ssb_tools/config/stage_b_scene.yaml
OUT=local_data/stage_b

# 1. 背景配方（Concrete034）和填缝砂浆纹理
python3 -m ssb_tools.stage_b_runtime_surface prepare-set --set src/ssb_tools/config/stage_b_material_set.yaml \
  --sources $OUT/sources --config $CFG --spec $SPEC --output $OUT/NEW_SURFACE
python3 -m ssb_tools.stage_b_runtime_surface prepare-filler \
  --downloads $OUT/sources/grey_plaster/downloads.json --output $OUT/NEW_FILLER

# 2. 把已入 Git 的裂缝目录绑定到本机 PNG，保留 source_sha256。
python3 - <<'PY_CATALOG'
import json
from pathlib import Path
root = Path.cwd()
out = root/'local_data/stage_b/NEW_CATALOGS'
out.mkdir(parents=True, exist_ok=False)
for name in ('long_crack_candidates_v1', 'crack_candidates_v1'):
    source = root/f'assets/cracks/generated/{name}.catalog.json'
    data = json.loads(source.read_text())
    data['source_file'] = str(root/f'assets/cracks/generated/{name}.png')
    (out/source.name).write_text(json.dumps(data, indent=2)+'\n')
PY_CATALOG
# 裂缝：基础布局 → 细化 → 有效深度（每步保存输入快照）
python3 -m ssb_tools.stage_b_defects --config $CFG --spec $SPEC \
  --long-catalog $OUT/NEW_CATALOGS/long_crack_candidates_v1.catalog.json \
  --short-catalog $OUT/NEW_CATALOGS/crack_candidates_v1.catalog.json --output $OUT/NEW_BASE
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
python3 tools/prepare_contact_demo.py --demo $OUT/NEW_OPTICS --world $OUT/NEW_GUI_LIGHT/world.sdf \
  --config $OUT/NEW_OPTICS/capture.yaml --spec $SPEC \
  --track-chord-mm 2 --track-cross-level-mm 2 \
  --output $OUT/NEW_DEMO --calibrate
```

说明：

- 第 3 步生成网格后自动做漏光审计（`ssb_tools.mesh_audit`，结果写入 `mesh_audit.json`）：全部板缝已填时，只要有一处从隧道内能看到背衬就判失败。
- 第 6 步默认保留来源网格。需要替换时显式传 `--geometry $OUT/NEW_GEOMETRY`，验证四个衬面网格及完整性清单，光学身份与标定保留。完整来源重建还需传 `--spec $SPEC`；公开纹理原图不随 Git 提供，由下载器获取；AI 裂缝 PNG 与目录已在 Git 内。首次部署的完整链使用 [ASSETS.md](ASSETS.md) 的入口，它显式提供所有输入并为裂缝目录绑定当前仓库路径，不依赖旧演示。
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
python3 -m ssb_tools.optical_bench --config CAPTURE.yaml --output BENCH --render
python3 -m ssb_tools.optical_calibration fit --bench BENCH/bench.json --output CALIBRATION.json
python3 -m ssb_tools.optical_calibration apply --session SESSION \
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

最终采集 `sessions/review_portable_gui_final`（原始数据已于第二次清理删除，见 §10.1），用的是移动到新目录后的可移植资产包，GUI 开启、光晕关闭：

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

2026-10-01 修复衬面漏光后，网格由 `geometry_b2_v4` 换为 `geometry_b2_v5`。配置、纹理、裂缝和光学参数不变；差别只在管片、倒圆角、砂浆和交汇补片之间原来微米级缝隙处的采样，未重新做画面复核。

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
python3 -m ssb_tools.optical_calibration apply \
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
- 衬面网格的漏光问题已修复（2026-10-01）。旧网格 `geometry_b2_v4`（已删除）审计出 59409 条漏光边，缝宽在 30 µm 以内，分为四类：管片与环缝倒圆角之间的 T 形接点、砂浆边缘与折线侧壁之间的缝、纵缝与环缝交汇补片的接口，以及少量管片与纵缝倒圆角的接口。漏过的子射线平时打在背衬上；碰上渲染器 2 m 分块接缝时会完全打空，导致整批拒绝。修复内容：
  - 管片按相邻倒圆角的实际顶点三角化；
  - 砂浆按侧壁实际折线定宽，并伸入侧壁后 50 µm；
  - 交汇补片的接缝下铺隐藏衬垫；
  - 渲染器各分块重叠 10 µm。

  `geometry_b2_v5` 审计结果为 0。保留的未填缝 / 破损板缝开发路径（量产不用）仍有约 5900 条漏光边，以后启用前需要重新设计。
- 只验证了直轨。完整 20 m 的联合 GUI 性能已单独验收（STAGE_C §6），不外推到弯轨或 50 m；现行扩展目标最多为 50 m。
- 轨道起伏有竖向和水平（差动）分量，没有轨向不平顺，也没有波磨、焊缝等局部缺陷。编码轮转动没有轴承阻力，翘起后按惯性转动。车轮刚度是假设值，DART 在 1 ms 步长下无法实现 0.1 mm 以下的静压缩量；没有模拟受载滚动半径变化和滚动阻力。

## 9. 待完成

1. 阶段 C 已完成：完整 20 m 原始采集、名义覆盖图、资源/吞吐报告及独立重放；裂缝索引已采用 double 端点格式。后续使用该会话进入阶段 D。
2. 阶段 D：展开、重叠匹配、全局优化、分块重采样。
3. 隧道加长到 50 m：高度场已按段生成（50 m 每根钢轨约 21 段），其余资产需重新生成并复测资源与实时率。
4. 按需加入：裂缝几何凹陷参考、机器人遮挡、镜头退化、未填/破损板缝的新模型、弯轨、轨向不平顺。

## 10. RViz 与任务控制

```bash
tools/run_mission.sh             # RViz 显示，Gazebo 服务器按任务启动
tools/run_mission.sh --gz-gui    # 同时显示 GZ GUI
# 可指定成套的演示与会话根目录：
tools/run_mission.sh --demo local_data/stage_b/contact_demo --output-root sessions/mission
```

现行演示和新生成的接触演示均从正下方 θ=180° 开始，先空转到右下方 θ=240° 后开门，经顶部到左下方 θ=480° 关门；底部 120° 始终不曝光。RViz 待机和初始化姿态读取同一配置角度。旧演示为 −130°，本次仅更改起始相位：场景、材质、灯具和测得标定未变，光学签名仍兼容，历史会话不修改。当前 `capture.yaml` SHA-256 为 `ce08fc594588e8ba14fcdcf5bb1a1986cf4c9b1f3ada923f57944720749c3be7`，同时更新了世界输入归档与其 manifest。最新真实 GZ＋OptiX 回归确认待机为 180°，首行实际曝光角约 240.0015°，进入右下门控前没有生成有效行。下述性能表已使用这一相位，并包含英文面板和原始图像缩略图。

面板默认 `Vehicle travel` 模式设置轨道纵向起点和前进距离，分别显示车体预计终点和全角度覆盖保守估计，界面全部使用英文，提供 Start、Pause、Resume、Stop（快捷键 Ctrl+Alt+S/P/R/E）。当前范围为 0–20 m，最短任务 1 m（按用户要求，避免过短采集）；开始前核对轨道及视场缓冲余量。运行中锁定输入。点击 Start 后，起点在生成任务世界时初始化，相当于将车体直接放到指定起点；只修改输入框不会移动车体；使用现行 0.2 m/s 标称速度，根据距离生成缓起停剖面，保留扫描由左右测量轮编码器平均里程驱动的逻辑。行驶距离显示编码器估计值，轮径误差仍会影响扫描。当前终止按运动剖面的计划时长执行；带轮径偏差的里程目标停车控制尚未实现。 另有 `Wall coverage` 模式，输入壁面目标起点和长度，车体起点/终点按标定视场和螺距派生，可能进入端部缓冲区；输入的壁面起点不等于车体初始化位置。目标和行程分别显示，见 STAGE_C §2。

有效曝光验收区 `acceptance.valid_x_m` 在生成任务时确定，按扫描初始相位、门控空转段和起停坡段向内收，另外保留 10 mm 名义跟踪余量。现行 3 m 任务起点为 3 m 时，区间为 [3.11,5.79] m；1 m 任务也有非空验收区。这是头部位置的无缺行检查区，不是车体行程或全角度壁面覆盖区，不从成像结果反推，不修改验收器。轮径或安装误差较大时仍可能判失败，须独立分析，不能静默继续缩区。

统一管理器 `ssb_tools.mission_manager` 接收 `/ssb/mission/command`，发布 `/ssb/mission/status`。每条命令带独立 ID；拒绝越界、活动任务中重新开始和重复请求。null、非数值及非有限的起点/距离在进入工作队列前被拒绝，保留原任务状态；队列已满时返回带请求 ID 的拒绝回执。面板发送前检查接收端是否存在，并在 90 s 内等待回执（覆盖 60 s 启动等待）；超时后清除等待状态，按最新任务状态恢复可用按钮并显示提示，不自动重发。管理器连接中断时取消等待，重连后重新按状态启用控件。暂停采用 Gazebo 冻结仿真时间，保留计数和扫描相位；已有曝光继续成像落盘。结束任务先暂停，再通知自己启动的服务器退出并排空队列，保存尾块；提前结束保持 `session.json` 的 `motion.complete=false`，界面显示“Stopped early; raw data saved”。此时完整任务检查脚本拒绝会话是预期行为。

管理器分别报告运行、已暂停、等待落盘、原始采集完成或失败；不在采集或任务退出时运行畸变校正和平场补偿。已保存行数只计入已写盘且回读哈希通过的块，尾块关闭后计入；成像滞后是最近输入位姿与写线程已处理曝光时间之差，不等于待耐久化尾块的时间。最终数据状态和哈希仍以会话清单为准。会话生成失败或后台成像失败会显示错误；每次重新开始创建不同的会话和任务输入目录。

RViz 复用 SDF 的视觉几何、颜色和预览贴图，转换为 Collada 并缓存到 `local_data/rviz_preview/`，每张纹理最长边 512 px；不加载 0.1 mm 光学纹理。环境 MarkerArray 缓存后以 1 Hz 重发，防止新任务时间归零或手动 Reset 清空显示后场景丢失；不重读或重新烘焙资产。车辆各 link 由 30 Hz TF 更新（与 RViz 帧率一致；原 10 Hz 时车辆运动明显跳动），完整原始图像不经 DDS。显示话题 `/ssb/sim_truth/scene`、`/ssb/sim_truth/joint_states` 和坐标系 `sim_truth/*` 只供观察；位姿来自仿真物理状态，严禁用于盲重建。`/clock` 与 Gazebo 仿真时间一致，RViz 启用 `use_sim_time`，支持新任务时的时间归零。RViz 光照只是观察效果，不模拟采集条光的光度和阴影。

任务控制面板位于右侧，左下方（Displays 下方）的 Live Capture Preview 显示最近已保存原始块的缩略图及行号范围；默认一个块为 4096 行。只读取写完且回读哈希通过后原子重命名的 `.u8`，不读取未完成块或依赖采集结束的索引。预览按面积平均缩小至最多 512×512、保留 Mono8 DN，每秒最多更新一次，同一块复用编码缓存；PNG 与会话/行号信息通过单条低频消息 `/ssb/mission/preview` 发送，消息小于 512 KiB。任务切换清空旧图，按会话身份拒绝延迟的旧图；暂停和结束后显示最后已保存图。预览不做畸变、平场、对比度或锐化校正，原始文件不变；文件访问错误只显示预览错误，不中断采集。

运行日志在 `local_data/mission_logs/`，各任务私有输入、世界和服务器日志在 `local_data/mission_runs/`；任务状态变更与最终真值写入会话的 `evaluation/mission.json`。关闭 RViz 会结束活动任务并等待原始图像保存；任务自然结束后 RViz 保持打开。可选的 GZ GUI 保留最后画面，新任务复用它连接新的服务器。

本轮真实 Gazebo＋OptiX 联调验证：越界拒绝、任意扫描相位暂停/继续、暂停后无新增曝光、活动任务中重新开始被拒、提前结束尾块保存与未完成标记、重新开始的会话隔离。1 m 暂停任务采集 99568 行，333 行批次重放的 31 个原始/元数据文件逐字节一致；最短 1 m 的重新开始任务也正常完成，低于 1 m 的任务被拒绝。缩略图的行号、PNG 尺寸和实际面板显示均通过检查；所有采集位姿时间间隔仍为 1 ms。另测 3 组 3 m 任务，各 284445 行，原始文件与会话清单哈希全部通过，RViz 预览无资源加载错误。

| 界面 | 动力学实时率 | 成像实时率 | 进程 RSS 合计峰值 |
|---|---:|---:|---:|
| GZ GUI | 1.000 | 0.994 | 3.34 GiB |
| RViz | 1.000 | 0.995 | 2.79 GiB |
| GZ GUI＋RViz | 1.000 | 0.995 | 3.78 GiB |
| GZ GUI＋RViz（2026-10-01 晚，测量轮、2+2 mm 轨道、v5 衬面、30 Hz TF） | 1.000 | 0.994 | 未测 |

测量记录 `local_data/evaluation/mission_ui_20261001_cleanup/tmp/ssb_ui_preview_acceptance/report.json`；RSS 为测试期间各自进程树的采样合计，包含共享页重复计数，不是独占内存。OptiX 的纹理分配峰值约 2.00 GiB，与 2 GiB 配置预算一致，未测量整机显存峰值。实时率从首个采集位姿开始，排除资产加载和 GPU 初始化，本轮未执行采后校正；暂停测试不用于性能表。上述表格仍记录 3 m 短程结果；另行完成的完整 20 m 联合 GUI 验收见 STAGE_C §6。

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

本轮已清理临时任务原始图像、重放副本、过期任务输入和日志，以及 560 个未被现行世界引用的 RViz 缓存文件，释放约 86.04 GiB 已分配磁盘空间。保留现行 `contact_demo` 及标定、已接受的画质基线 `sessions/review_portable_gui_final` 和当前 280 个缓存文件（该基线会话后来在第二次清理中删除，见下）。测试报告、必要截图和会话元数据归档到 `local_data/evaluation/mission_ui_20261001_cleanup/`，删除清单见其中 `cleanup.json`；归档不含这些临时任务的原始图像，重放需要重新采集。该本地归档含生成端私有配置，不纳入 Git，也不作为盲重建输入。

**第二次清理（2026-10-01，用户确认）**：删除旧车型和旧衬面网格时期的数据，共约 15.6 GB：
- 最终采集 `review_portable_gui_final` 及其重放、动力学记录，可移植资产包 `review_portable_relocated`；
- 竖向起伏试验 `irr_trials_v2` 及其资产；
- 默认演示验收的重放副本；
- 旧车型的任务会话 `sessions/mission` 和 `local_data/mission_runs`；
- 任务回执回归采集 `mission_audit_fix/capture_regression`；
- 旧衬面网格 `geometry_b2_v4`。

画质基线以复核页 `final_review_20261001`（含原像素对照图）为准。现存：阶段 A 基线 `sessions/gz_a`、默认演示验收 `sessions/default_acceptance`、测量轮试验 `sessions/measuring_trials`，以及现行演示的全部输入。

### 10.2 任务数据保留

2026-10-02 再次清理重复展开、重放和界面测试数据，释放约 70.02 GiB；保留 20 m 原始基线、最终 CPU/CUDA 展开和 D2。原有任务演示与修复测试目录已归档后删除，现行保留清单见 [DATA_RETENTION](DATA_RETENTION.md)。

当前采用手动清理，不自动删除任务。每个 3 m 原始任务约 1.3 GB，20 m 约 8–9 GB；离线 float32 校正、重放副本和复核材料另计。任务原始数据在 `sessions/mission/`，对应生成端私有输入在 `local_data/mission_runs/`，运行日志在 `local_data/mission_logs/`。采集前检查可用空间；保留已接受的画质/验收基线和用户仍在检查的会话。临时试验验证完成后保留小型报告、必要配置和截图，再手动删除不再需要的原始数据及匹配的输入目录；运行中的会话不可清理。可用 `du -sh sessions/mission local_data/mission_runs local_data/mission_logs` 查看增长。

### 10.3 任务验收与回执修复回归（2026-10-01）

132 项 Python 测试通过，包含 1/1.2/3/20 m 的名义验收区、多种初始相位、区内缺行拒绝、畸形输入和满队列回执。真实 Gazebo＋OptiX 的 1 m 暂停/继续和 3 m 完整任务，各经独立重放后通过阶段 B 验收器全部 22 项检查。1 m 任务有效区 [2.11,2.79] m 有 68282 行、零缺行；3 m 任务有效区 [3.11,5.79] m 有 267393 行、零缺行，总行数仍为 284445。起停缓冲区内丢行继续如实报告。

实际 RViz 面板验证了接收端缺失、拒绝回执、管理器断连/重连，以及实际等待 90 s 无回执后重新操作；测试在 91.29 s 检查时已恢复按钮，下一条命令成功接收并回执。报告位于 `local_data/evaluation/mission_audit_fix/panel_regression_final/report.json`（`capture_regression` 已于第二次清理删除）。回归临时原始数据/重放副本及其私有输入已清理约 4.23 GiB，保留小型配置、报告和必要截图。用户观看的历史会话 `sessions/mission/20261001_145804_982cf426`（已于第二次清理删除）保持原配置与哈希，其中原有 [3,6] m 验收区不会追溯修改；修正后的区间用于新生成任务。

## 11. 轨道不平顺与车轮柔性

设计依据见 DESIGN §5.1（车轮柔性）和 §6.1（轨道不平顺）。实现位置：

- 剖面与高度场：`ssb_tools.rail_irregularity`；
- 轨道生成：`stage_b_track.replace_track`；
- 车轮弹簧与标定：`stage_b_robot.wheel_compliance`、`ssb_tools.wheel_stiffness`；
- 一致性检查：物理清单与检查 `ssb_tools.physical_world`（采集前、采集后），插件 `world_check.hpp`（启动时），验收器 `validate_contact` 和 `validate_stage_b`（用会话里的物理快照）。

接触采集必须在配置旁提供 `spec.yaml`。会话 `evaluation/physical/` 保存世界、规格、配置、物理清单和高度场，完成时为其中每个文件记录哈希。快照检查只读取归档文件：缺图像或规格即失败，不回退到原演示目录。剖面 NPZ/统计 JSON 若存在也一并归档。

阶段 B 接触验收使用同一归档世界核对车辆起点和物理参数，并把归档 SDF 哈希与采集输入哈希比较；原世界路径仅保留为来源标签。缺少完整快照或文件哈希保护的旧接触会话需要重新采集，不自动补齐或降级验收。此独立性仅针对物理输入；光学验收仍需要原光学资产。

物理清单现为 `ssb.physical_manifest.v2`，还包含两侧钢轨碰撞盒的尺寸、形状及模型/连杆/碰撞的完整姿态。平直轨承载面顶面必须为 z=0；起伏轨限位盒顶面按剖面最低点向下留出 3 mm，底面保持 z=−38 mm，侧面保留轮缘限位。启动器和插件分别检查 SDF 与已加载实体。旧 v1 世界需重新生成；若物理世界本身已正确，可用 `python3 -m ssb_tools.physical_world write --config ... --world ...` 更新清单，再运行 `check`，不可修改历史会话来补验收。

**生成内容**：`world/track/` 下有 `rail_profile.npz`（5 mm 网格剖面）、`rail_irregularity.json`（档位、种子、均方根、10 m 弦最大值、最大坡度、各段高度场哈希）和 `rail_top_{left,right}_XX.png`（每根钢轨每段一张 16 位高度场）。剖面文件存左右两根钢轨，记录中另有水平最大值、5 m 和 0.7 m 基长扭曲最大值。它们都是仿真真值，重建不得读取。

**DART 车轮刚度标定**（车体 98 kg、轴座 0.5 kg、车轮 5 kg，1 ms，阻尼比 0.2）：

| 目标静压缩量 | SDF 刚度 | 按 载荷/压缩量 的名义刚度 |
|---|---|---|
| 0.15 mm | 4.92×10⁶ N/m | 1.60×10⁶ N/m |
| 0.20 mm | 2.86×10⁶ N/m | 1.20×10⁶ N/m |
| 0.40 mm | 1.00×10⁶ N/m | 0.60×10⁶ N/m |

整车实测静压缩量为 0.1501 / 0.2000 / 0.4000 mm。标定在生成演示时自动进行，结果和迭代记录写进配置。

**动力学实测**（3 m 短程，只算动力学，起步 1.5 s 后统计）：

| 工况 | 车体升沉范围 | 俯仰范围 | 俯仰与参考偏差 | 升沉与参考偏差 | 编码器 − 后轴行程 | 车轮载荷 / 静载 |
|---|---|---|---|---|---|---|
| 平直，0.2 mm | 0 | 0.07 mrad | 0.03 mrad | 0.00 mm | +0.03 mm | 0.94–1.06 |
| 2 mm，0.2 mm | 1.9 mm | 3.2 mrad | 0.13 mrad | 0.04 mm | −0.31 mm | 0.69–1.16 |
| 5 mm，0.2 mm | 4.8 mm | 6.9 mrad | 0.38 mrad | 0.14 mm | −0.52 mm | 0.69–1.16 |
| 5 mm，0.15 mm | 4.8 mm | 7.0 mrad | 0.34 mrad | 0.13 mm | −0.46 mm | 0.62–1.18 |
| 5 mm，0.4 mm | 4.8 mm | 6.8 mrad | 0.45 mrad | 0.18 mm | −0.51 mm | 0.81–1.11 |

- 车轮始终不卸载，所有工况都通过 `validate_contact` 的全部检查。
- 车体响应对车轮刚度几乎不敏感：激励频率（几 Hz）远低于悬挂固有频率，车体基本按几何关系跟随轨面。
- 参考值取刚性圆盘在剖面上的准静态位置（半径 0.1 m 的车轮会跨过短波谷底）。
- 起步阶段：车体以平直姿态放到轨道上，静置时轮轨静摩擦锁住一点预应力，开动后释放。稳定段判据从 1.5 s 起算；但首行曝光在约 1.05 s（扫描头进入门控时），1.05–1.5 s 的误差单独报告（`startup`），不作通过判据。刚性轮时这段偏差最大约 1.3 mrad；现行 0.2 mm 柔性下，升沉 16 µm、俯仰 6 µrad、横滚 13 µrad（默认演示）。
- 编码器读的是轮轨接触点的走行，与后轴实际行程相差约 0.02%。
- 19 m 行程（5 mm 档、0.2 mm）稳定：俯仰范围 12.2 mrad，编码器多计 0.45 mm，滑移 99% 分位 0.74 mm/s，车轮载荷为静载的 41%–124%。

**验收判据的变化**：车体参考点在轨面上方 0.3 m，俯仰时会相对车轮前后摆动，所以行程、滑移和倒退改在编码轮中心计算（现为测量轮中心，含滑块位移）。新增或调整的判据：

| 判据 | 要求 |
|---|---|
| 无倒退 | 总倒退量小于 0.5 mm，指令车速超过 5% 后不得倒退 |
| 小纵向滑移 | 99% 分位小于 1 mm/s，最大值小于 5 mm/s |
| 支撑高度 | 与参考值相差不超过 0.3 mm（平直轨仍为 3 mm） |
| 车体跟随轨面（有起伏时） | 起步后俯仰和横滚偏差都小于 1 mrad；有水平分量时，支撑高度和本项只取四轮都受载的时段 |
| 车轮始终受载（有柔性、无水平分量时） | 悬挂压缩量始终大于零；有水平分量时改为报告各轮卸载比例和预测比例 |
| 静压缩量（有柔性时） | 与目标值相差不超过 2% |
| 物理世界与真值一致 | 在会话的物理快照上重做全部物理检查：两轨齐全并覆盖全长、位置姿态尺寸、文件哈希、逐段剖面（该段量化容差）、重叠区、车轮半径、弹簧 |
| 测量轮贴轨（接触模式） | 测量轮中心与钢轨剖面上刚性圆盘的高度差小于 0.1 mm，滑块离行程限位至少 1 mm |

```bash
python3 -m ssb_tools.validate_contact SESSION_dynamics --config CAPTURE.yaml --world WORLD.sdf
```

**带成像的完整采集**（5 mm 档、0.2 mm，3 m，无 GUI，代码定稿并重新构建后采集）：284443 行，成像实时率 0.977，动力学实时率 0.982，渲染 12.6 s。阶段 B 验收器 23 项全部通过，包括运行程序与源码一致、新增的"钢轨剖面与真值一致"、有效区无缺行和 333 行批次重放逐字节一致；接触验收全部通过。会话为 `sessions/irr_trials_v2/final_chord5`，演示资产在 `local_data/stage_b/irregularity_trials_v2/`。GUI 下的实时率尚未在起伏轨道上单独测量。

**单步断触**：刚性轮在高度场上偶有 1 ms 的断触，竖向速度出现 9.8 mm/s（重力加速度乘以 1 ms）的跳变，与起伏幅值和高度场分辨率无关。4WIDS 也观察到同类现象。加上车轮柔性后不再出现。

**水平（差动）分量与测量轮**。编码器原先装在两个后轮上。在有水平分量的轨道上，刚性车体加硬聚氨酯轮会翘轮，翘起的编码轮在减速和停车时空转：竖向 2 + 水平 4 mm 的 3 m 工况多计 27.5 mm，19 m 工况停车时翘起的轮一直转，多计约 0.28 m，扫描头也跟着转。用户决定改为车尾两个弹簧压紧的测量轮（DESIGN §5.2），后轮只承重。

测量轮实测（0.2 mm 柔性，只算动力学；`sessions/measuring_trials`，资产在 `local_data/stage_b/measuring_trials`）。"轴距扭曲"是车辆实际走过区段的 0.7 m 基长最大扭曲；"测量轮贴轨误差"是测量轮中心与刚性圆盘在钢轨剖面上的高度差：

| 工况 | 行程 | 轴距扭曲 | 横滚范围 | 承重轮卸载时间（最多的一轮） | 编码器 − 行程 | 滑移最大 | 测量轮滑块行程 | 测量轮贴轨误差 |
|---|---|---|---|---|---|---|---|---|
| 平直 | 3 m | 0 | 0 | 0 | +0.02 mm | 0.51 mm/s | 0.16–0.19 mm | 0.2 µm |
| 竖向 2 mm（默认） | 3 m | 0 | 0 | 0 | −0.13 mm | 0.62 mm/s | ±0.5 mm | 0.9 µm |
| 竖向 2 + 水平 2 mm | 3 m | 0.59 mm | 0.91 mrad | 0 | −0.13 mm | 0.63 mm/s | −0.4–0.7 mm | 1.1 µm |
| 竖向 2 + 水平 4 mm | 3 m | 1.18 mm | 1.86 mrad | 57% | −0.13 mm | 0.65 mm/s | −0.9–0.8 mm | 67 µm（一次，0.2 s） |
| 竖向 5 + 水平 4 mm | 3 m | 1.18 mm | 1.84 mrad | 59% | −0.20 mm | 0.67 mm/s | −1.6–1.3 mm | 67 µm（瞬时） |
| 竖向 2 + 水平 2 mm | 19 m | 1.35 mm | 1.53 mrad | 16% | +0.08 mm | 0.62 mm/s | ±1.1 mm | 70 µm（瞬时） |
| 竖向 5 + 水平 4 mm | 19 m | 2.69 mm | 3.08 mrad | 31% | +0.28 mm | 0.96 mm/s | −3.0–2.4 mm | 73 µm（瞬时） |

- 全部工况通过接触验收。承重轮照样翘轮、车体照样对角摇摆，但里程不再受影响：19 m 误差不超过 0.3 mm（0.0015%），扫描跟踪误差与平直轨相同。
- 测量轮贴轨误差平时约 1 µm；车体对角翻转瞬间测量轮受冲击，接触求解出现最大约 70 µm 的瞬时穿透，0.2 s 内恢复。判据为 0.1 mm，同时要求滑块离行程限位至少 1 mm。
- 车体姿态只在四轮都受载的时段与平面参考比较（19 m 时占 43%–73%），俯仰、横滚偏差不超过 0.12 mrad（5+4 mm 的 3 m 工况为 0.56 mrad），升沉偏差不超过 0.19 mm。

**测量轮碰撞体**用与轮半径相同的球。轨头是圆弧形轨冠，测量轮与轨面本来就是近似点接触。圆柱碰撞体在 DART（ODE 高度场）上不稳：轻载的 80 mm 圆柱在车将停稳时恰好落在高度场网格线上，曾陷入轨面 5.6 mm。球与高度场的接触没有这个问题。

**高度场长度修正**：gz-physics/DART 把 N 个采样铺在 size×(N−1)/N 的长度上，未补偿时每段两端错位约 11 mm。生成器已按 N/(N−1) 放大 SDF 尺寸（DESIGN §6.1）。本节前面的竖向起伏实测和下面的旧默认演示验收都是在这两项修正和测量轮之前做的。

**默认演示（2026-10-01 起）**：`local_data/stage_b/contact_demo` 为 2 mm 档竖向起伏加 2 mm 档水平分量，0.2 mm 车轮柔性，测量轮编码，衬面网格 `geometry_b2_v5`。它沿用原演示的光学设置、密钥和标定（光学签名不变），用 `--geometry` 换入新网格。

水平取 2 mm，与竖向档位同一依据（GB/T 50299 新线验收：高低、水平 ≤2 mm）；4 mm 是运营线综合维修限值，代表状态偏差的线路。3 m 演示路段轴距扭曲 0.59 mm，承重轮不卸载；19 m 行程会有约 16% 的时间一轮卸载，里程不受影响（见上表）。

验收会话 `sessions/default_acceptance`：
- 采集：284443 行，动力学实时率 1.00、成像实时率 0.993（无 GUI）。
- 阶段 B 验收器 24 项全部通过，含二进制与源码一致、333 行批次重放逐字节一致、物理世界与真值一致（会话快照）。
- 接触验收 15 项全部通过：编码器与行程相差 −0.11 mm，车体升沉 1.98 mm、俯仰 2.75 mrad、横滚 0.89 mrad，测量轮贴轨误差 67 µm（对角翻转瞬间），18 段钢轨高度场的物理检查 10 项通过，重叠区差 0.05 µm。
- 起步段（首行曝光 1.054 s 至 1.5 s）单独报告：升沉 16 µm、俯仰 6 µrad、横滚 13 µrad。

平直轨演示可用 `--track-chord-mm 0 --track-cross-level-mm 0 --wheel-deflection-mm 0` 重新生成。

**实时率**：加测量轮后物理每步约 0.95 ms，接近 1 ms 步长，采集实时率降到 0.91。原因是钢轨高度场太密：高度场必须是 2ⁿ+1 的正方形，513 点、12 m 一段时，73 mm 宽的轨头上也有 513 行，每个车轮每步要检查几千个格子。改为 129 点、3 m 一段（沿轨仍约 23 mm）后，每步降到 0.49 ms。同一路段离线重放，v5 网格只比 v4 慢约 2%。再降采样收益很小：在默认演示世界中（不含插件），129 点每步 0.36 ms，65 点 0.34 ms，33 点 0.34 ms，平直轨（无高度场）0.16 ms。剩余开销来自车轮与高度场的接触处理，与采样密度无关；65 点以下段数成倍增加，所以保持 129 点。
