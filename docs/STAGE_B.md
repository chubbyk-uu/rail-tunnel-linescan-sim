# 阶段 B 开发记录

阶段 B 已接入实际纹理光学采集，尚未完成本阶段全部验收。20 m 场景已生成；短程采集和离线重放用于检查管线，不能替代完整 20 m 采集及后续拼接优化。

2026-09-29 外观复核发现：GUI 偏暗、GUI 与采集背景细节不足、裂缝折线与均匀暗线感、板缝深槽暗边过强。修正计划见 [STAGE_B_QUALITY_PLAN.md](STAGE_B_QUALITY_PLAN.md)，设计基线更新至 [v0.7](../DESIGN.md)。2026-09-30 初选 Wall 04 原生 16K 单一主背景；用户查看 B1 样块后反馈仍模糊，要求排查并下载 Concrete030 8K 对照。分级排查和新对照见 [纹理清晰度调查](STAGE_B_TEXTURE_AUDIT.md)。2026-10-01 B1 定稿：只用 Concrete034 作主背景，0.1 mm 生成网格加 2×2 纹理足迹积分，并做重复控制（同方向重合约束、4 种方向、大尺度明暗层），用户认为重复感可以接受；GUI 成像实时率 0.994（`sessions/b_c034_DC4_g10_v1`）。下述“当前实现”的 2K/1 mm 资产、0.35 环境光及未填槽均是当前实现记录，不是新版质量验收结果。

### B1 高清背景样块（2026-09-30，Wall 04 阶段；已被 Concrete034 取代，定稿见纹理清晰度调查）

B1 的 Concrete034 背景已由用户复核；B2 板缝、裂缝及后续审核修正已实现。现行裂缝模型为 `cavity_v2`，有效深度整体乘 0.8。GUI 默认采集入口已切换到 `optics_b2_v12`；墙面预览仍使用历史低清材质，高清 GUI 材质和缺陷预览属于待实施的 B3。以下旧参数和旧性能数据按历史版本解读。

- 经环境代理下载 Wall 04 的底色、NormalGL 和粗糙度，均为 16384²、16 位原生 PNG，总下载约 2.626 GB，约 71.8 s；底色/法线原文件带 alpha，准备时使用 RGB 内容，不改变物理映射尺度。逐通道 libvips 顺序解码、128 行浮点处理、磁盘映射输出，源打包 2 GiB，准备主进程 RSS 峰值约 519 MiB、耗时约 144.6 s。默认准备工作集上限 4 GiB，解码子进程另有 64 MiB libvips 缓存限制。
- 新 `stage_b_runtime_surface` 输出 `ssb.surface_runtime.v1`：原生源、固定裁片布局及混合掩码。`SurfaceRecipe` 提供 CPU 参考，`CudaSurfaceRecipe` 在 GPU 生成当前曝光需要的 1024 核心块，接入原有有界缓存，保持反照率16/粗糙度8/双分量法线16的格式。源及配方单独占约 2.002 GiB 显存，局部块另计，不与 2 GiB 生成块预算混算。源码位于 `ssb_core/src/surface_recipe.cpp` 和 `ssb_core/src/optix/surface_recipe.cu`。
- 背景网格仍覆盖 23 m 全圆周，虚拟纹理间距约 0.2 mm、113×85 个逻辑块（2026-09-30 起改为等于源纹素间距约0.1953 mm、115×87 块，见[原生对齐生成网格](STAGE_B_TEXTURE_AUDIT.md#原生对齐生成网格2026-09-30)）；**没有物化保存这 9605 个高清块**。布局约 0.8 m 裁片、0.2 m 重叠，5 m 复用距离内拒绝原生裁片中心小于 0.08 m 的重用；这是单来源布局的初始约束，不是已通过全局特征重复度验收。20 mm 引导图用于拼接布局及混合遮罩；底色/法线/粗糙度的细节从原生16K源采样。后续排查发现遮罩羽化尺度约13 mm，局部混合过宽，须另行修正，不能因通道数值一致就认为视觉质量通过。
- 六个首尾/中部块经 C++ CPU 与独立 NumPy float64 对照，GPU 各通道最多差 1 个量化单位；部分末块、周期冗余正确，未混合单片底色对原生插值误差约 0.027/65535。GPU 单块生成约 0.59–1.84 ms（本次短测）。合成夹具另验证八种方向、法线变换、源哈希拒绝、逐出重建及不同分批结果。
- 0.2 m/s、20 rpm、28.444444 kHz、8 μs 曝光，以平滑曲面和相同条光生成三档底色样块，每档4096×5927像素，周向约1.2 m、每行轴向约0.852 m；该运动使图像保持螺旋行排列。原亮度/×0.8/×0.65 的平均灰度约190.2/152.2/123.6；后两档无饱和。亮度变化用对已打包×0.8底色的等效、截断前增益实现，未做网页增强。源材质自带的痕迹是背景，不含人工裂缝宽度真值。
- 同一16K源、同一布局和同一照明的1 mm间距对照显示明显软化；与三源2K旧场景不同，不能称为逐像素相同内容的2K对照。实际样块改为333行批次后，图像与交点文件逐字节一致。完整无人工缺陷曲面短探测约5.7–6.0万行/秒；统计从初始化后开始，不含Gazebo/GUI/完整编码器会话，不当作全链RTF验收。

结果 `local_data/stage_b/b1_wall04_16k/b1_results.json`，原生源哈希 `source/downloads.json`，块对照 `recipe_validation/`，原始PGM与参数 `optical_samples/`，查看页 `review/index.html`。未锐化/去噪，100%原始行与完整PNG可查看。准备命令：

```bash
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.stage_b_runtime_surface fetch \
  --resolution 16k --output local_data/stage_b/b1_wall04_16k/source
PYTHONPATH=src/ssb_tools python3 -m ssb_tools.stage_b_runtime_surface prepare \
  --downloads local_data/stage_b/b1_wall04_16k/source/downloads.json \
  --config src/ssb_core/config/stage_b.yaml \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --brightness 0.8 --texel-m 0.0002 \
  --output local_data/stage_b/b1_wall04_16k/runtime_surface
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_recipe_probe \
  --surface local_data/stage_b/b1_wall04_16k/runtime_surface/surface.json \
  --tiles 0,112,9492,9604,4999,5000 \
  --output local_data/stage_b/b1_wall04_16k/recipe_validation
```

以上输出已存在时拒绝覆盖；复现实验使用新目录。光学样块的准备和运行脚本在该结果目录中，分别为 `prepare_samples.py` / `run_samples.py`。B1不代表全隧道视觉随机性、人工缺陷自然程度或完整20 m性能通过，后续仍按计划推进。

## 当前实现

- 轨道车约 120 kg，四个轮轴、理想直线导向；相机光心位于半径 2.75 m 的隧道轴上。底盘保留双侧驱动盒、轮毂电机、双横梁和电子仓，上部以用户 2026-09-29 补充的新版真机图片为准：近直双立柱、紧凑 U 形架，沿旋转轴依次为光源、相机、滑环。两侧地面相机及其立柱不建模。轮径仍估计为 200 mm，场景 x∈[-1.5,21.5] m、目标区 [0,20] m。DART 轮轨接触叠加导向约束曾引起位置跳变，因此当前采用轮轴速度控制和理想滚动关系，不验收真实轮轨接触动力学。
- 内壁、板缝、裂缝不设置动力学碰撞体。OptiX 独立使用实际三角网格追踪光学交点与遮挡。现行板缝为 3 mm 倒角、10 mm 槽宽、全部填砂浆，带圆角和连通交叉；现行网格 641773 个三角形；管片角度步长 0.5°，弦高小于 0.027 mm。板缝后方有连续不透光背衬，防止不同分段曲面在接缝交会处产生漏光。
- 相机 4096 像素、90 mm 镜头，对焦距离 2.75 m；有效投影距离 93.045112782 mm、轴向视场 0.852259271 m。旋转编码器 2500 线、四边沿计数、×128÷15，每圈前进 0.6 m。当前车速 0.2 m/s、转速 20 rpm、触发行频 28.444444 kHz，每圈 3 s。50 kHz 保留为相机暂定上限。轴向像素间距 0.208071 mm、周向行距 0.202485 mm，降速不改变空间间距；8 µs 曝光内周向运动约 0.046077 mm。底部 120° 停止采集，顶部 240° 由编码器逐行触发；超限时速度与旋转同步降低。
- （历史开发背景）三款 CC0 背景 Wall 04、Concrete030、Wall 03 在线性光域统一色调，初版采用 ×0.8 亮度；正式背景已改为 Concrete034，见 `stage_b_material_set.yaml`。原亮度、×0.8、×0.65 的比较仍保留，亮度属于待确认的工程参数。Concrete030 的 2 m 映射宽度为工程假设；Wall 04 的 3.2 m、Wall 03 的 4 m 有官方页面尺寸依据。
- 多源裁片按物理尺度铺排，采用旋转/镜像、重叠区域最小割和统一混合权重，底色、粗糙度、法线共享布局，法线随变换正确转向。5 m 纵向复用约束目前针对同一来源的相近裁片中心，不等价于所有视觉特征都不重复；整体相似度验收尚未完成。
- 背景分块烘焙后添加稀疏矢量裂缝，物理宽度独立于生图素材像素宽度。初版 60 个实例，细长 48、细短 9、网状 3，符合 80%/15%/5%；细长、细短内部少量分叉。主体宽度服从截断正态分布，均值 0.4 mm、标准差 0.1 mm，范围 0.2–0.6 mm，沿路径平滑变化并在自由尖端收口。第一条主干原始标定弧长为 10 m，细化后约 11.22 m，分叉长度不计入主干。密度与其余长度分布是可调工程参数。
- 裂缝走势来自生图素材骨架，保存原连通拓扑，拒绝断裂主干，不使用短片段重复拼出 10 m。B2 起路径经细化、腔内和边缘带按外观假设着色（见下方“B2 裂缝”）；已有合成有效深度用于腔内反照率，几何凹陷和内部自遮挡仍未实现。含有效深度的稀疏索引约 27.9 MiB，与背景分辨率独立。
- 光源采用用户确认的 20×20 mm COB、80×80 mm 散热器，附风扇和特制凸透镜；尺寸不是 30 cm 灯条。OptiX 用 2×2 高斯点近似等效有限发光面，考虑距离衰减、粗糙漫反射及遮挡；同一光源位置函数用于正常着色与板缝可见性检查。透镜以壁面半高宽 1.2 m×0.12 m 的等效光束表示，不模拟折射；COB 面积与实测透镜出瞳不等价，绝对照度、增益、光度曲线待标定。
- 当前采用 `--integrated --area-samples 16 --area-pattern rooks --time-samples 3`：只有单线段包含整个空间/曝光足迹时才使用稳定解析积分；分叉、多线段、折点和有限端部回退到每时刻 32 条 N 车射线（`crack_area_samples`，默认 32），共 96 条，按点级几何并集求覆盖。背景照明仍使用三个曝光时刻和 2×2 纹理足迹积分。关键几何边缘、板缝遮挡变化保留每时刻 16 条 N 车射线。自适应另为显式可选项，当前关闭。局部裂缝测试采用相位偏移合成的 64×64/16 射线参考；常规 16×16/16 参考在静止、平行于像素轴的边缘处存在较粗的空间量化。

相机安装以 **等效投影中心位于旋转轴** 为理论基线。`camera_optical` 为头部原点，`camera_sensor` 位于后方约 93.045 mm，`camera_front_glass` 位于前方暂估 35 mm；镜筒跨过旋转轴，机壳细白线标示内部传感器平面。后方距离按当前薄透镜模型的成像常数推导，前玻璃和壳体外形是工程估计，不能解释为实际镜头内部光学结构。该外观修正保持原 OptiX 投影中心、视场、采样和灯光参数。

固定 U 架底边中心距旋转轴 215 mm（下移 10 mm），斜角连接板与底部电机支座相应衔接；托盘全周最小几何间隙约 18.12 mm。两根立柱中部新增交错斜撑：按车体 +x 前进、+y 为左，前立柱连前横梁左侧，后立柱连后横梁右侧；锚点落在结构横梁上，避开电子仓盖。斜撑暂取直径 24 mm，仅作为底座刚体的固定结构，未引入弹性刚度或结构强度验算。

当前橙白车模采用约 0.56 m 长 U 形架（此前为 0.72 m）、顶部间距 0.62 m 的近直立柱；光源、相机随共同转轴旋转，右侧滑环区分固定外壳和转子。U 形架斜角底边、镜头、散热片、风扇与管夹为可视几何，仍保持 6 个动力学刚体和 2 个底盘碰撞体，不新增纹理贴图或细节碰撞体。散热器平面与 COB 尺寸已确认；透镜直径、组件高度、支架和传动外形尚按照片比例估计。

光源位于相机轴向 −115 mm，切向/径向偏移及内倾角均为 0。按 852.259 mm 相机视场、1200 mm 光斑计算，最大允许中心间距 173.870 mm；当前较紧一端仍有 58.870 mm 覆盖余量。软件半高宽不代表光斑内照度完全均匀，后续可用平场校正及实测配光标定。GUI 现用附着在扫描头上的 Ogre2 projector 显示条光：轴向/周向半高宽 1.2 m×0.12 m，沿用 OptiX 的八次超高斯包络，单张 1280×128 RGBA 纹理约 0.63 MiB。投影体只覆盖内壁附近 20 cm 径向范围，不增加固定点光源。Ogre2 将其实现为矩形发光贴花体，因此只用于检查光斑位置与同步扫掠，不用于验证逆平方照度、机器人遮挡或镜头折射；实际成像使用 OptiX 光照。环境散光调为 0.35 以便观察光斑。

## 资源与精度优先级

按当前要求，先满足车速 0.2 m/s、GUI 成像实时率 ≥0.6，再在此范围内提高精度，不跳过编码器触发的行。物理曝光时间、空间采样间距与计算消耗的墙钟时间分开处理。

`src/ssb_tools/config/stage_b_scene.yaml` 的 GPU 纹理预算为 **2 GiB**，CPU 纹理预算为 **1 GiB**。缓存采用 LRU、按需分配；每块 1024² 核心、四周 4 像素冗余，实际格式是反照率 16 位、粗糙度 8 位、缺陷保护标记 8 位、双分量法线各 16 位，共 8 字节/像素，约 8.13 MiB/块。当前不含 mip 链。两项预算最多容纳约 252/126 块，不包含网格、驱动、CUDA 上下文、图像队列等其他开销。

光学配置中的预算覆盖烘焙时记录的旧缓存预算，因此提高缓存无需重烘焙背景。GPU 根据实际批次射线覆盖范围加载；覆盖超过预算时拆批并保留每行。CPU 在读入新块前逐出旧块，避免临时越过纹理预算。记录缓存峰值、命中次数、加载耗时以及进程 VmHWM，不能仅用纹理分配量代替进程总内存/显存。

素材准备工作集仍为 512 MiB，写盘队列 128 MiB，最多 4 个待处理批次，每批 1024 行。当前 2K 开发背景按 1 mm 烘焙，391 块、约 3.10 GiB，准备峰值约 486 MiB；源纹理的物理细节约 0.98–1.95 mm。它用于检查纹理、光学与采集流程，**不能声称具备 0.2 mm 背景细节**。0.2–0.6 mm 裂缝则由独立物理矢量和像素覆盖积分呈现。最终背景必须使用原生高清来源和有界解码/采样流程，插值放大不能补出真实细节。

## 准备与运行

构建后执行以下流程。准备输出已存在时拒绝覆盖；失败输出标记 `FAILED`。下载来源、色调配方、几何、纹理块、裂缝与最终光学配置都保存哈希。

```bash
source install/setup.bash
ros2 run ssb_tools stage_b_materials fetch \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --output local_data/stage_b/material_sources_2k
ros2 run ssb_tools stage_b_materials preview \
  --sources local_data/stage_b/material_sources_2k/sources.json \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --output local_data/stage_b/colour_preview
ros2 run ssb_tools stage_b_surface \
  --config src/ssb_core/config/stage_b.yaml \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --sources local_data/stage_b/material_sources_2k/sources.json \
  --recipe local_data/stage_b/colour_preview/colour_recipe.json \
  --texel-m 0.001 --brightness 0.8 --output local_data/stage_b/surface
ros2 run ssb_tools stage_b_defects \
  --config src/ssb_core/config/stage_b.yaml \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --long-catalog assets/cracks/generated/long_crack_candidates_v1.catalog.json \
  --short-catalog assets/cracks/generated/crack_candidates_v1.catalog.json \
  --output local_data/stage_b/defects
ros2 run ssb_tools prepare_stage_b_scene \
  --config src/ssb_core/config/stage_b.yaml \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --surface local_data/stage_b/surface/surface.json \
  --output local_data/stage_b/geometry
ros2 run ssb_tools stage_b_optics \
  --config src/ssb_core/config/stage_b.yaml \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --geometry local_data/stage_b/geometry \
  --surface local_data/stage_b/surface/surface.json \
  --defects local_data/stage_b/defects/defects.json \
  --integrated --area-samples 16 --area-pattern rooks --time-samples 3 \
  --output local_data/stage_b/optical
SSB_WORLD=$PWD/local_data/stage_b/geometry/world.sdf \
  tools/run_gz.sh sessions/b_capture local_data/stage_b/optical/capture.yaml

# 改变批大小离线重放，检查原始图像和元数据逐字节一致。
bash tools/with_optix_runtime.sh install/ssb_core/lib/ssb_core/ssb_render \
  --config local_data/stage_b/optical/capture.yaml --session sessions/b_replay \
  --poses sessions/b_capture/evaluation/pose_stream.bin --batch-rows 333
ros2 run ssb_tools validate_stage_b_smoke sessions/b_capture --compare sessions/b_replay
```

`stage_b.yaml` 自身仅是几何/时序基线；必须使用 `stage_b_optics` 输出的配置启用纹理光学后端。Gazebo 预览直接读取哈希校验后的实际烘焙底色，以约 2 mm/像素为每个管片生成独立贴图，采用完整 OBJ 法线；旧版缺少法线会导致 Ogre2 材质构建失败。已停用 20 mm quilting 引导图作为 GUI 底色。GUI 贴图 RGBA8+mip 保守估计约 499 MiB，预算 768 MiB，准备实测峰值约 211 MiB。GUI 预览不用于线阵成像，矢量裂缝目前仍只在 OptiX 采集层呈现。

本机现行入口是 `tools/run_gz_gui.sh`，默认加载 `crack_review_fix_v1/world.sdf` 与 `optics_b2_v12/capture.yaml`：使用优化后的 B2 几何，车体从 x=3 m 起步，3 m 短程采集，光学层为 Concrete034、修正后的裂缝和 0.8 有效深度。初始暂停，点击 Play 开始采集；结束后后台完成落盘和摘要。GUI 墙面预览仍为旧低清材质，不能用它判断采集背景清晰度；高清 GUI 预览属于 B3。相邻 4WIDS_agv 的私有 Mesa 启动器可由 `SSB_MESA_WRAPPER` 指定。其他输出通过 `tools/run_gz_gui.sh SESSION CONFIG WORLD` 指定，旧 `geometry_light_v1`/`optical_light_v1` 仅作历史对照。

`ssb_probe` 用于指定姿态的光学检查和吞吐测量，不代表编码器采集验收。`stage_b_tag_cracks` 可在背景烘焙后加入保守保护标记，再以 `stage_b_optics --adaptive` 显式开启自适应加速；当前解析覆盖档不需要这一步。

采集的评估目录归档场景、背景、裂缝真值，供验收使用；重建输入仅保留可观测信号、相机参数和后端身份哈希，后续拼接不得读取缺陷真值。

## GUI 验证

Concrete034 背景（`sessions/b_c034_DC4_g10_v1`）：运动计划与下方 2K 背景演示相同。共 284445 行，**GUI 成像实时率 0.994**，动力学结束后 0.1 s 完成落盘；21 项独立检查及 333 行批次重放全部通过。优化过程和逐项耗时见[纹理清晰度调查](STAGE_B_TEXTURE_AUDIT.md#实时率优化裂缝索引网格2026-10-01)。Gazebo GUI 的墙面预览仍是旧的 2K 贴图（属 B3），高清背景只进入 OptiX 采集。

2K 开发背景（历史）：

最新光斑展示与采集：`sessions/b_light_demo_v2` 的 16.3 s 计划中车体 x=3→6 m，共前进 3 m、扫描头旋转 5 圈，保存 284445 行，形成 5 段 4096×56889 的逻辑条带；GUI 成像实时率约 **0.880**。20 项独立检查及与 333 行批次离线重放的对比通过，报告 `sessions/b_light_demo_v2/evaluation/reports/stage_b_smoke.json`。行位置验收区为 [3.1,5.7] m，不是全角度覆盖承诺；原始条带尚未拼接。查看页位于 `local_data/stage_b/light_demo_v2/review/`。该会话仍用 2K/1 mm 背景，视觉质量未通过用户复核；这些性能与完整性结论不能外推到计划中的高清材质和新缺陷光学模型。

新版紧凑扫描架 + 20×20 mm COB 等效光源复测：0.2 m/s、28.444 kHz、1280×720 GUI 下，9.4 s 仿真保存 171147 行，成像实时率 **0.775**、动力学实时率约 1，停止后约 2.72 s 全部落盘。59 项测试结果通过；20 项独立检查和 333 行批次重放通过，有效区域无缺行。报告 `sessions/b_robot_v4_gui/evaluation/reports/stage_b_smoke.json`，模型展示 `local_data/stage_b/robot_review_v4/`。仍为短程结果，不代表完整 20 m 稳态性能或实物光度标定。

橙白车模、纯环境散光的复测：9.4 s 仿真保存 171147 行，GUI 成像实时率 0.841、动力学实时率约 1，停止后约 1.78 s 全部落盘。20 项独立检查和 333 行批次重放通过，报告位于 `sessions/b_robot_v3_gui2/evaluation/reports/stage_b_smoke.json`。车模展示目录为 `local_data/stage_b/robot_review_v3/`。这仍是短程性能结果。

前次灰白车模加载全部管片预览、GUI 为 1280×720，0.2 m/s 下运行 9.4 s、多圈扫描保存 171147 行，最终验证成像进度实时率 **0.782**、动力学实时率约 1，之前同设置短测约 0.73。完整计算和落盘包括停止后的滞后；此值是短程验证，不能承诺整个 20 m 相同。20 项独立检查及 333 行批次重放均通过，报告见 `sessions/b_gui_final_v2/evaluation/reports/stage_b_smoke.json`。

用 Gazebo `/gui/screenshot` 服务确认真实渲染图，而非仅检查 PNG 存在。入口验证截图在 `local_data/stage_b/gui_preview_check_v1/`；修复前后与最终 GUI 采集截图在 `local_data/stage_b/sampling_comparison_v1/`。实际进程映射确认私有 Mesa 的 D3D12/NVIDIA 路径，OptiX 使用 RTX 5080。

0.2 m/s 下，长裂缝、分叉和网状裂缝局部与 16×16/16 参考比较，最大分别差 1、3、3 个灰度值；板缝区域最大差 14，局部 99 分位为 6，仍需加强板缝边缘收敛。报告 `sampling_comparison_v1/quality_speed020_v1/comparison.json`。独立均匀底色夹具对 0.2–0.6 mm 裂缝测试 16 个亚像素相位，静止/运动的对比度积分等效宽度最坏误差约 6.2 µm；这不是镜头 MTF 或实物宽度测量验收。报告 `sampling_comparison_v1/width_fixture_speed020_v1/comparison.json`。

## 早期 50 kHz、高采样实测（历史基线）

RTX 5080 上，高精度档的 4096 行名义运动探测约 483 行/秒；一次含完整门控进入/退出的短程 Gazebo 采集保存 56883 行，约 595 行/秒，动力学实时率约 1，成像进度实时率 0.0167。图像计算在动力学结束后继续约 94 s，全量落盘。两次门控事件与独立参考相符，有效测试区 [7.85,8.20] m 的 49778 行无缺失；区外 6 条初始化丢行有明确记录。GPU/CPU 纹理分配峰值各 221524992 字节（约 211 MiB）。这些数值不是完整 20 m 负载结论，也不包含其他显存分配。

`b_precision_gate_smoke_v1` 与改变批大小至 333 行的 `b_precision_gate_replay_v1` 通过 20 项会话检查，24 个原始图像/元数据/评估数据文件逐字节一致；64 个独立 CPU 参考射线的最坏交点误差约 0.468 µm。报告位于 `sessions/b_precision_gate_smoke_v1/evaluation/reports/stage_b_smoke.json`。

名义车速/转速下，裂缝局部的 8 档与 16 档参考比较，最大差 2 个灰度值；99% 的局部像素一致。板缝边缘探测仍有最大 14 个灰度值差异，不能据裂缝局部收敛就认定全部场景已收敛，后续需加强边缘积分检查。自适应 8 档在同一段约 210° 的扫描中约 2275 行/秒，99% 以上像素一致，但极少数像素最大差 8，暂保留为可选加速档。最终质量优先的默认仍关闭自适应。

比较结果保存在 `local_data/stage_b/optical_quality_nominal_convergence_v1.json`、`optical_quality_joint_convergence_v1.json` 与 `optical_adaptive_quality_comparison_v1.json`。这些是当前工作点调整前的历史基线；纹理载入耗时较小，主要成本是光学积分，扩大缓存不能直接消除这一成本。

## 验证与剩余工作

自动检查包括 0.2–0.6 mm 裂缝宽度积分、曝光积分、条光足迹、分块重采样一致性、法线变换、接缝不漏光、缓存逐出/拆批不改变图像、资产被修改时拒绝采集，以及已有阶段 A 时序回归。短程会话验证器独立检查编码器/门控、源代码与二进制身份、全部资产哈希、CPU 三角形交点、CPU/GPU 纹理预算及离线重放一致性；它不是完整光度或镜头分辨率验收。

仍需完成原生高清背景及整体重复纹理检查、填充/损伤板缝和手孔、镜头模糊/离焦/噪声与光度标定、标定尺寸下裂缝自然程度审查、完整 20 m 负载测量。本阶段的短程采集不代表已输出 20 m 隧道全图；完整采集覆盖、拼接和全局优化继续按阶段 C/D 推进。

GUI 投影语义参考本机 Ogre2Projector 实现及 [Gazebo Rendering 源码](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/ogre2/src/Ogre2Projector.cc)。

## B2 板缝（2026-10-01）

- **几何**：`stage_b_scene` 生成 `panels.obj`（含倒角）、`joints.obj`（槽侧壁和槽底）、`filler.obj`（砂浆）和 `gap.obj`（接触缝后的止水垫），共约30万个三角面，预算为60万。环缝按下一环的分块分段，纵缝按环分段，每段的状态与缺失区段记入 `geometry.joints`。
- **材质**：0 号为墙面和倒角；1 号为槽内混凝土（反照率 0.12、低对比细节、坐标错开）；2 号为砂浆（`prepare-filler`，grey_plaster 去掉 10 mm 以上色调、对比度 0.35、均值 0.15，最初为 0.17）；3 号为止水垫（0.02）。全部为外观假设。
- **入口**：`stage_b_runtime_surface prepare-filler`；`stage_b_optics --filler`，有 `filler.obj` 时必须提供。验收工具会校验砂浆纹理的哈希。
- **尺寸**：倒角 3 mm、槽宽 10 mm，从墙面看缝区总宽 16 mm（用户确认）；最初的 5 mm 倒角、15 mm 槽宽版本总宽 25 mm，偏宽。三种宽度的对比图在 `local_data/stage_b/b2_run/joints_widths_1to1.png`。
- **结果**：`sessions/b_b2_joints_v2`，GUI 成像实时率 0.982（加缝前 0.994），21 项检查及 333 行批次重放全部通过。
- **定稿（`geometry_b2_v3`，`sessions/b_b2_joints_v3`）**：全部已填；倒角两侧 1 mm 圆角、内缘轻微起伏；T 形交叉用高度场连通，交界沿用环缝砂浆边的折线以保证不漏光，砂浆面在交叉内平滑过渡。约 78 万个三角面（预算 80 万）。GUI 成像实时率 0.981，21 项检查及重放全部通过。图在 `local_data/stage_b/b2_run/joints_final_1to1.png`、`b2_probe/junction_zoom3x_v4.png`。崩角样块（`chip_sample/`）不自然，代码未保留。前后对比图在 `local_data/stage_b/b2_run/joints_before_after_1to1.png`；三种状态的探针图、暗边拆解和灯光换边图在 `local_data/stage_b/b2_probe/`。

## B2 裂缝

- **路径与宽度**：`stage_b_defects --refine`（参数在 `stage_b_scene.yaml` 的 `cracks.refine`）。按 0.5 mm 重采样，σ=5 mm 高斯平滑去掉源骨架约 4.8 mm 像素的阶梯；叠加 0.5–20 mm 自仿射摆动（10 mm 波长幅值 0.5 mm，H=0.75），在分叉共用端点 10 mm 内渐隐，连通性不变；宽度沿程双尺度对数起伏（σ 0.18/0.08），限制在 0.2–0.6 mm。均为合成细节，已记入缺陷真值。现行开发缺陷集为 `local_data/stage_b/defects_dev_v5`；审核后的宽度、端点和主路径真值修正见下方记录。
- **覆盖积分（历史）**：旧版以线段带状解析覆盖率最大值近似并集，曾与 8×8×3 参考比较得到 RMSE 1.28 DN。审核发现这不能正确处理分叉、交叉和端部，现已改为单段解析与局部 64 N 车/3 时刻采样；该历史 RMSE 不作为现行验收。
- **着色（最初）**（场景 `crack_optics`，平底）：腔内反照率为所在墙面的 0.2 倍，带 ±35% 类碎屑变化；两侧 0.25 mm 边缘带压暗 15%。原尺寸复核时像墨线：10 m 长裂缝沿程每列最暗处都是背景的 18–23%，几乎没有起伏，边缘也很硬。没有 `crack_optics` 的旧场景仍用固定反照率 0.035。
- **着色（历史 cavity_v1，V 形截面）**：`stage_b_defects --depth` 为每个顶点生成合成的有效可见深度 D=深宽比×局部宽度，深宽比沿程取对数正态（中位 0.9、σ 0.5、波长 5–40 mm），每米约 6 段、长 3–15 mm 的浅段（深宽比 0.15，代表灰尘填塞）；深度写入 `depths.bin` 并记入真值。渲染按朗伯槽口公式计算腔内反照率：ρf/(1−ρ(1−f))，f=w/(w+2D)；横截面为 V 形（半宽 1、2/3、1/3 的三层嵌套带各占三分之一），中心最暗、两侧渐浅。开发缺陷集 `defects_dev_v3`：有效深度中位 0.35 mm（5–95% 为 0.12–0.88 mm），墙面反照率 0.3 时中心腔内为墙面的 0.26–0.68 倍。同位置重渲染后，长裂缝沿程每列最暗处是背景的 0.29–0.56 倍（中位 0.44）；对比图在 `local_data/stage_b/b2_run/cracks_1to1/*_old_vs_cav1.png`。均为外观假设；没有几何凹陷、真实自遮挡和随灯光方向的明暗不对称。
- **宽度**：由图像暗度积分估计的宽度比几何真值平均宽 0.114 mm，其中约 0.094 mm 来自边缘带；因背景纹理噪声，逐窗口相关性较低。几何宽度真值与图像暗线宽度分开报告。
- **结果**：平底版为 `sessions/b_b2_cracks_v1`；空腔版为 `sessions/b_b2_cavity_v1`（`optics_b2_v8`），GUI 成像实时率 0.995，渲染 6.67 s，21 项检查及 333 行批次重放全部通过。平底场景经重构后重放逐字节不变。
- **细缝台阶**：0.23 mm 细缝放大后呈横竖台阶。检查路径几何，各段走向均匀分布，接近坐标轴方向的只占 11–12%，与随机情况一致，所以台阶不在几何里；它来自亚像素宽斜线的方形像素积分，现在没有镜头模糊。降低对比后已不明显。是否加入镜头点扩散另行决定，因为这会影响整幅图的清晰度。

## 统一性能优化

离线基准：`b_b2_cracks_v1` 的位姿流、1024 行批次、无 GUI（`local_data/stage_b/b2_run/bench.sh`）。

| 步骤 | 渲染 s | 内核 s | 生成块 s | 足迹 s |
|---|---:|---:|---:|---:|
| 起点 | 12.24 | 7.00 | 3.91 | 0.99 |
| 足迹块范围改为按周期直接算区间 | 11.38 | — | — | 0.41 |
| 交叉处三角面 78 万→64 万（`geometry_b2_v4`，3 倍放大图无可见差异） | 11.68 | 7.01 | 3.92 | 0.40 |
| 块生成：行/列分离的坐标计算、每批一次启动 | 8.04 | 6.81 | 0.51 | 0.40 |
| 完整射线像素改为 N 车 16 条/时刻（`optics_b2_v7`） | 5.93 | 4.72 | 0.51 | 0.40 |

- 块生成前后，40 个探针块和整次重放都逐字节相同。块生成从每块 0.94 ms 降到 0.09 ms；原先耗时主要来自每次调用的固定开销和逐纹素的双精度坐标计算。
- 完整射线像素只占 0.85%，但每个要追 192 条主射线，再加每条 4 条阴影射线，此前占内核时间约 40%。以 16×16×3 网格为参考，在这 990 万个像素上比较：旧 8×8 网格 RMSE 1.12 DN，p99 为 6 DN，超过 4 DN 的占 1.5%；N 车 16 条/时刻 RMSE 0.85 DN，p99 为 3 DN，超过 4 DN 的占 0.10%。射线减到四分之一，误差反而更小，因为板缝边缘近似平行于像素轴，网格每轴只有 8 档，N 车的 3 个时刻合起来有 48 档。只把临界边距离改为按像素足迹计算的试验反而更慢：回退像素沿缝成片，同一线程束仍要等其中最慢的像素，因此未采用。
- 同位姿逐像素对比：只有完整射线像素有变化（占 0.38%），其余像素逐字节不变。对比图在 `local_data/stage_b/b2_run/opt_grid8_vs_rooks16_1to1.png`（左旧、中新、右为差值×8）。
- 全链路：`sessions/b_b2_opt_v1`，GUI 成像实时率 0.995，渲染累计 6.61 s（墙钟 16.38 s），队列峰值 1 批（此前满 4 批），21 项检查及 333 行批次重放全部通过。实时率受 Gazebo 实时步进限制，上限约为 1；渲染余量约 2.5 倍。

## B2 审核修正与裂缝浅化

现行资产为 `defects_dev_v5`、`geometry_b2_v4`、`optics_b2_v12`（v11 加 `crack_area_samples: 32`）；Concrete034 配方 `c034_DC4` 保持不变。修正后的 348799 个非尖端主体顶点宽度均在 0.2–0.6 mm；所有主路径顶点均属于实际细化路径，旧版最大约 3 mm 的偏离已消除。另修正很短的图边两端平滑区互相干扰的问题，恢复其源图共用端点；250 个端点过渡顶点有调整，主体路径保持原位置。源配置、基础规范及每次细化/深度规范保存哈希快照，完整索引范围一起校验，渲染也拒绝损坏快照或不匹配的范围。含深度索引约 27.9 MiB，C++ 预算检查包含深度数组。

生产单精度解析积分改为归一化、分段及因式分解，避免接近退化投影方向的消减误差。同一组 200 万名义运动构造案例的最坏裂缝覆盖率误差从 0.02683 降至约 2.23×10⁻⁷；这是数学检查，不是镜头 MTF 证明。新增 GPU 局部检查覆盖交叉、圆端、折点及收尖，静止/名义运动与相位偏移构成的 64×64×16 射线参考比较；保留独立 75% 交叉并集及有限圆端面积检查。背景仍为 2×2 足迹积分，复杂裂缝局部为每时刻 64 条 N 车射线；49 项 Python 测试与全部 C++/GPU 测试通过。

按用户要求，`cracks.depth.scale: 0.8` 让合成有效深度整体浅 20%，当前中位 0.280 mm、5–95% 为约 0.095–0.714 mm。对同一修正后的几何、深度场、相机及照明只改变这一比例，512×4096 样块有 22009 个像素变亮，无变暗像素，变化像素平均增加 4.41 DN，最大增加 11 DN；其余像素逐字节相同。对比图与原始数据在 `local_data/stage_b/crack_review_fix_v1/depth_comparison.png`、`depth_before/`、`depth_after/`。这只改善槽口外观，不新增几何凹陷或真实自遮挡。

（64 条/时刻时的记录）修正后短程首次 GUI 性能实测仍约 0.995，保存 284445 行；局部提高采样后渲染累计约 12.8 s，队列峰值 4 批，满足 0.2 m/s、GUI 成像 RTF≥0.6。该性能值不外推到完整 20 m。整仓库指纹包含文档，因此定稿后重新构建与复验；最终采集为 `sessions/b_b2_crack_fix_gui_v2`，333 行批次重放为 `sessions/b_b2_crack_fix_replay_v2`，最终身份、行完整性、资源和逐字节一致性以 `evaluation/reports/stage_b_smoke.json` 为准，必须包括 `binary_matches_source: pass`，不绕过来源校验。

**复核（32 条/时刻）**：统计同一位姿流，约 233 万个裂缝像素中 99.6% 走点级并集路径，只有约 1 万个走单段解析积分，因为细化路径有 0.5 mm 尺度摆动，像素足迹内几乎总含多个线段。曾试过同一路径连续段仍按最大值解析：与并集相比最大偏差 49–88 DN，不采用。改为降低并集路径的采样数，以每时刻 128 条为参考：64 条 RMSE 0.54 DN、最大 5 DN，离线渲染 11.1 s；32 条 RMSE 0.84 DN、p99 2 DN、最大 9 DN，8.1 s；16 条 RMSE 1.85 DN、最大 20 DN，6.8 s。32 条与板缝边缘已接受的档位（0.85 DN）相当，定为默认（`sampling.crack_area_samples`，生成元按 N 车格点自动计算）。局部参考测试中交叉中心像素的容差由 ±2 放宽到 ±4 DN：仍明确区分 75% 并集（约 39）与最大值近似（约 69）。全链路 `sessions/b_b2_crack32_v1`（`optics_b2_v12`）：GUI 成像实时率 0.995，渲染累计 9.07 s（主内核 7.79 s，墙钟 16.38 s），渲染队列在裂缝密集段达到 4 批上限；21 项检查及 333 行批次重放 `b_b2_crack32_v1_replay` 全部通过。

旧版缺陷布局若仍指向已修改的原配置，派生操作会明确拒绝。重新准备基础布局即可得到不可变快照，然后按顺序细化和加深度；不要手改旧哈希，也不要覆盖历史输出：

```bash
python3 -m ssb_tools.stage_b_defects --config src/ssb_core/config/stage_b.yaml \
  --spec src/ssb_tools/config/stage_b_scene.yaml \
  --long-catalog assets/cracks/generated/long_crack_candidates_v1.catalog.json \
  --short-catalog assets/cracks/generated/crack_candidates_v1.catalog.json --output NEW_BASE
python3 -m ssb_tools.stage_b_defects --refine NEW_BASE \
  --spec src/ssb_tools/config/stage_b_scene.yaml --output NEW_REFINED
python3 -m ssb_tools.stage_b_defects --depth NEW_REFINED \
  --spec src/ssb_tools/config/stage_b_scene.yaml --output NEW_DEPTH
```

需先 `source install/setup.bash` 或设置 `PYTHONPATH=src/ssb_tools`。GUI 默认采集已使用新版光学场景和 3 m 短程世界，高清 GUI 背景/缺陷预览仍待 B3；完整 20 m 采集及拼接优化仍待后续阶段。
- 未做的项：曝光端点改用平面外推（省约 0.65 s）、同一时刻的 4 个纹理取样合并光源计算（省约 0.4 s）。收益有限，暂不实施。

## 后续第1项：采样档位兼容性

旧光学场景缺少 `sampling.crack_area_samples` 时保留64条/时刻；生成工具增加
`--crack-area-samples {32,64}`，默认64高精度档，32为性能档。两档均保留点级并集。
默认GUI配置改为 `optics_b3_quality64/capture.yaml`，它仅将v12的采样数显式改为64，
其他光学资产、深度和照明不变。旧资产不覆盖。49项Python测试、3组C++/GPU测试通过；
并集参考测试分别覆盖显式32和缺省64。历史32档性能记录仍是当时实测，不代表当前默认档。

## 后续第2项：Concrete034 GUI 分块预览

`python -m ssb_tools.stage_b_gui --scene local_data/stage_b/optics_b3_quality64/scene.json
--world local_data/stage_b/crack_review_fix_v1/world.sdf --output local_data/stage_b/gui_c034_v1`
从已验收配方生成预览，原生底色先面积滤波，再按同一裁片变换、混合遮罩及宏观亮度采样。
将原始 `panels.obj` 的三角形分组并重映射UV，逐个保留其顶点、法线和面；不再用光滑圆柱
覆盖倒角。原板缝、砂浆和槽底网格保留。裂缝采用同一线段/宽度/有效深度数据做4倍子像素
胶囊覆盖预览，不人为加宽；不宣称与OptiX曝光积分和光度逐像素一致。

x=3..6 m附近采用1 mm纹素，其余2 mm，原生采集网格仍为0.1 mm。GUI预算独立设为
1280 MiB（RGBA8含完整mip链的预估），生成前检查总量和单纹理8K尺寸上限；实际贴图为中性灰RGB PNG，避免单通道纹理在不同渲染后端的通道解释差异。
这是有界常驻分块加Ogre mipmap，不是动态流送。细裂缝远看会自然变淡，微米级量测仍看采集图。
产物清单保存输入及各块哈希和三角形计数；输出目录拒绝覆盖。默认入口切换到该世界。

本次产物140块，RGBA8含mip预估745.8 MiB，混凝土471652个三角形全部保留。
52项Python测试通过，Gz实际加载并截图核查。全填缝场景的 `gap.obj` 无三角形，
GUI不再提交这个空网格（消除Ogre加载错误）；光学资产不改。
