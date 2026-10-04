# 阶段 B：现行场景与采集操作

更新：2026-10-04。本页只保留当前配置、操作和限制。历史画质实验、轨道/车轮参数扫描和任务修复过程见 [归档快照](history/STAGE_B_SNAPSHOT_2026-10-03.md)，已完成的工程修复见 [修复记录](history/REPAIR_PLAN_2026-10-02.md)。

## 1. 状态

已实现 20 m 隧道、Concrete034 背景、填缝管片与细裂缝、刚体轮轨接触、双测量轮编码器、旋转 COB 条光、0.6% 畸变、独立图像标定、Gazebo/RViz 任务控制。用户已接受响应增益 2.4；当前采集 250°、输出上方 240°，每侧 5° 保护区。

完整 20 m 原始采集和独立重成像已完成。采后 CUDA 展开、图像匹配及全局轨迹优化也已实现，见 [阶段 D](STAGE_D.md)。新的优化整幅图与融合仍待完成；接缝及四边支撑通过不等于整幅所有像素几何达标，现行结果集中在 [验收文档](EVALUATION.md)。

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
3. 先静置 2 s，然后车体从 x=3 m 按缓起停剖面行驶 3 m（0.2 m/s、20 rpm、28.444 kHz）。轨道带 2 mm 档竖向起伏和 2 mm 档水平（差动）分量，车轮带 0.2 mm 静压缩量的聚氨酯柔性，测量轮编码（§9）。
4. 关闭 GUI 后，脚本停止服务器、等待数据全部落盘，用 `tools/check_session.py` 检查完整性，只保留原始图像；畸变校正和平场补偿在拼接前另行执行（§6）。

日志在 `local_data/gui_logs/<时间>/`。WSL 下需要私有 Mesa（`SSB_MESA_PREFIX`，默认 `~/opt/agv-mesa-25.2.8/install`），启动器是 `tools/with_mesa_runtime.py`。

**生成新演示**：用现存资产派生一份新的 3 m 接触演示，`--calibrate` 会同时渲染标靶并拟合标定：

```bash
python3 tools/prepare_contact_demo.py --output local_data/stage_b/NEW_DEMO --calibrate
```

默认输入是完整的 `contact_demo` 包，可用 `--demo /path/to/bundle` 指定迁移后的副本；不依赖 v10/v11 中间目录。脚本重新生成机器人和轨道，默认保留来源包的衬面网格，并自动复用匹配的标定；`--calibrate` 用于重新标定。派生目录还需要下述导出步骤才能再次独立迁移。

轨道起伏与车轮柔性（§9）也在这里设定，二者都会写进配置的真值段并决定世界的生成：

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
| 管片与板缝 | 环宽 1.2 m，6 块（10°+2×67°+3×72°），错缝 18°；倒角 3 mm、槽宽 10 mm、深 35 mm，缝区总宽 16 mm，1 mm 圆角，全部填灰砂浆；网格 698922 个三角形，角度步长 0.5°，衬面闭合（历史漏光审计 0 处，过程见归档快照） | `stage_b_scene.yaml`；`geometry_b2_v5`；砂浆纹理 `filler_v2` |
| 背景 | Concrete034，0.1 mm 生成网格，GPU 按配方生成 1024² 块；亮度 0.8，增益 0.533；4 种保持抹痕方向的变换；0.45 m 内同向重合 ≤25%；painted_plaster_wall 大尺度明暗层 | `stage_b_material_set.yaml`；`c034_DC4` |
| 裂缝 | 60 个实例（细长 48、细短 9、网状 3），一条 10 m 主干（细化后约 11.2 m）；宽度截断正态 μ0.4/σ0.1 mm、范围 0.2–0.6 mm；`cavity_v2` 腔内模型，有效深度 ×0.8，中位约 0.28 mm | `defects_dev_v5`（快照来源 `defects_review_refined_v2`、`defects_review_base_v1`），索引格 10 mm，旧 float 线段格式 |
| 扫描光源 | 20×20 mm COB，2×2 高斯点；轴向 −0.115 m；光斑半高宽 1.2×0.12 m；弱反射补光 0.002 | `scene.json` 的 `lamp` |
| 相机 | 4096 像素、7.04 μm，90 mm 镜头对焦 2.75 m，视场 0.852 m；8 μs 曝光；k1=0.006 枕形畸变 | `capture.yaml` |
| 门控 / 响应 | 初始 180°，−125°～+125° 采集；响应增益 2.4 | 成套 capture.yaml / scene.json |
| 采样档位 | 背景走解析路径，3 个曝光时刻，2×2 纹理足迹积分；临界几何每时刻 16 条 N 车射线；复杂裂缝每时刻 64 条（可选 32 性能档）；自适应关闭 | `stage_b_optics --integrated --area-samples 16 --area-pattern rooks --time-samples 3` |
| 机器人 | 120 kg；前驱、后轮从动，两个测量轮（80 mm，竖直滑轨，预压 30 N）带编码器，四个导向轴承；车底电池/驱动舱（仅外观，尺寸为估计）；底座 0.3 m、轴高 1.715 m | `capture.yaml` 的 `robot`、`contact`；`world.sdf` |
| 轨道起伏 | 现行演示为 2 mm 档北京地铁谱竖向起伏加 2 mm 档水平分量（种子 20261001）；竖向可选平直或 5 mm，水平可选 0 或 4 mm（§9） | `capture.yaml` 的 `truth.track_irregularity`；`world/track/rail_top_*.png` |
| 车轮柔性 | 现行演示为聚氨酯轮柔性，静压缩量 0.2 mm（标定 SDF 刚度 2.86×10⁶ N/m）；可选 0.15/0.4 mm 或刚性轮（§9） | `capture.yaml` 的 `truth.wheel_compliance` |
| 纹理预算 | OptiX GPU 2 GiB、CPU 1 GiB；GUI 1280 MiB（现行 140 块约 746 MiB） | `stage_b_scene.yaml` 的 `resources` |
| GUI 预览 | 同源 Concrete034 分块底色（x=3–6 m 处 1 mm，其余 2 mm），裂缝 4 倍子像素预览；环境散光 0.6；工作灯和条光见 DESIGN §7.5 | `gui_c034_v1`、`gui_strip_shadow_final_v10` |

来源列中的 `c034_DC4`、`defects_dev_v5`、`gui_*` 是历史生成标签，不是新部署依赖。实际运行依赖封装在 contact_demo/assets 与 world 内，以 bundle.json 和成套配置为准。素材原图在 `local_data/stage_b/sources/`（concrete034、grey_plaster、painted_plaster_wall），各目录的 `downloads.json` 记录来源、许可和哈希。首次部署使用 `tools/download_demo_sources.py` 从公开网站下载并写入来源清单，然后用 `tools/build_demo_from_sources.py` 从零生成完整演示，见 [ASSETS.md](ASSETS.md)。

## 4. 下载与生成资产

首次部署按 [ASSETS](ASSETS.md) 从官方下载原图并生成完整包，不依赖旧机器或历史中间目录。主背景为 Concrete034，历史 Wall 04 / Concrete030 选材实验不属于默认生成链。逐模块旧生成命令保留在归档快照，当前统一入口为 `tools/build_demo_from_sources.py`。

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

`validate_stage_b_smoke` 独立检查以下各项，首次报告写到 `evaluation/reports/stage_b_smoke.json`，同时保存 `stage_b_smoke.identity.json`（报告及两份会话摘要的 SHA-256）：

- 源码与二进制身份、全部资产哈希、编码器与门控时序、有效区无缺行。
- CPU 三角形交点：用 double 源网格独立计算。
- GPU/CPU 纹理预算、重放逐字节一致。

已有报告或哈希记录会被拒绝覆盖。复查不修改历史证据：

```bash
ros2 run ssb_tools validate_stage_b_smoke SESSION --compare REIMAGE --read-only
# 需要另存本次检查时，指定新的 evaluation 报告路径；同样不会覆盖。
ros2 run ssb_tools validate_stage_b_smoke SESSION --compare REIMAGE \
  --output SESSION/evaluation/reports/recheck_NEW.json
```

新冻结协议 v5 核验首次报告哈希；旧报告没有该记录时不能补写后声称历史字节身份已恢复。重成像原图必须实际独立渲染，核验后才能去重；已链接副本的比较不是新的独立证据。

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
| 留出标靶几何最大残差 | 0.02351 px |
| 亮场列变异系数 | 6.055% → 0.3526% |
| 有效输出列 | 98.58%，边缘不外推 |

这些结果对应理想无噪声相机和固定标定距离，不代表真机精度。

## 7. 已知局限

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

## 8. RViz 与任务控制

```bash
tools/run_mission.sh             # RViz 显示，Gazebo 服务器按任务启动
tools/run_mission.sh --gz-gui    # 同时显示 GZ GUI
# 可指定成套的演示与会话根目录：
tools/run_mission.sh --demo local_data/stage_b/contact_demo --output-root sessions/mission
```

现行默认头部从正下方 θ=180° 开始，先空转到 θ=235°（等价于 −125°）开门，经过顶部到 θ=485°（+125°）关门。门控 250°，底部 110° 不曝光；最终成果仍排除底部 120°。起始相位、标定和签名以该包配置为准，不沿用归档的配置哈希或首行角度。

面板为英文，提供 Start、Pause、Resume、Stop 和原始行图缩略图。Vehicle travel 输入车体起点及行程；Wall coverage 输入壁面目标并自动派生超扫。范围由公开配置声明，默认 0–20 m、最短 1 m。修改输入框不移动车辆，开始后在生成世界的规划起点初始化；运行中锁定输入。暂停冻结仿真，已有曝光可继续成像落盘；提前结束保留未完成标记。新接触任务按双测量轮估计里程减速停车，连续确认静止后完成；超时或提前结束不声明运动完成。历史配置仍按时间剖面结束。实现与受控验证见 [WHEEL_ERROR](WHEEL_ERROR.md)。

采集不自动生成 processed/optical，畸变和平场在拼接前处理。RViz 只作轻量环境预览，真值显示坐标系不进入重建。丢回执超时、Reset 后静态 Marker 重发及联合 GUI 验证过程见归档快照；尚不能排除所有 WSL 显示驱动引起的偶发黑屏。

需要实际任务集成回归时，先构建，输出使用新目录：

```bash
python3 tools/with_mesa_runtime.py bash tools/with_optix_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib"
  export GZ_GUI_PLUGIN_PATH="$PWD/install/ssb_gazebo/lib"
  python3 tools/test_mission.py --output /tmp/ssb_mission_NEW
' > /tmp/ssb_mission_regression.log 2>&1
```

## 9. 轨道不平顺与车轮柔性

默认竖向和水平（差动）分量均为 2 mm 档，车轮等效静压缩量 0.2 mm。波长谱的频率是空间频率（cycle/m），不是 Hz；车速把空间起伏转换为时间激励。实际车体横滚、俯仰和升沉由刚体接触求解，不把指定姿态曲线直接当成测量输入。

钢轨顶部采用分段原生高度场，同一全局剖面采样并保持重叠一致；轮轨碰撞不用三角网格。车尾双弹簧压紧测量轮持续测里程，承重后轮仍从动。轮材等效柔性与 DART 弹簧标定是仿真假设，没有轴承/滚动阻力、受载滚动半径变化或真实材料黏弹模型。

接触配置旁提供 spec.yaml，采集前检查物理清单；完成会话的 evaluation/physical 保存世界、规格、配置和高度场及哈希。验收只读归档快照，不回退到现行资产。物理世界与真实轮径必须一致，剖面/轨道标签不能进入拼接。

模型细节以 [DESIGN §6](../DESIGN.md#6-隧道轨道与缺陷场景) 和生成配置为准。历史刚度、起伏扫描、接触判据和高度场性能调查保留在 [归档快照](history/STAGE_B_SNAPSHOT_2026-10-03.md#11-轨道不平顺与车轮柔性)，不外推到弯轨或 50 m。后续误差场景见 [ROADMAP](ROADMAP.md)，保留数据见 [DATA_RETENTION](DATA_RETENTION.md)。
