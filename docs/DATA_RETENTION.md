# 数据保留与清理

更新：2026-10-03。大型本机资产和会话不进 Git。清理只针对已完成的重复计算、重放及界面测试；生成、采集、重建中的文件不能删除。

## 当前保留

| 数据 | 位置 | 用途 |
|---|---|---|
| 离线提速后冻结复核 | `sessions/perf_ring_{high,low}_20261003/`、`local_data/evaluation/perf_tier2_20261003/` | `d60e15d` 上用同一轨道种子与区段重采；协议 v4、公开运行审计和独立评分均通过，与原冻结结果的接缝分数一致到小数点后 6 位；每组约 3.8 GB |
| 环缝高/低占比新轨道留出 | `sessions/ring_{high,low}_seed20261005_20261003/`、同名 `local_data/evaluation/` 目录 | 冻结 `b206f6d`、未用于调参的轨道种子；保留原图、独立重成像、D1/D2/D3、阶段 B 与协议核验、公开输入复现 |
| 环缝修复完整 20 m 复算 | `sessions/ring_phase_20m_corrected_20261003/`、`local_data/evaluation/ring_phase_20261003/corrected_20m_*` | 使用第二批原图/D1，窗口间 P95 0.691 px；新匹配、轨迹、公开复现和真实网格评分，不是新 20 m 采集；没有新整幅图 |
| 环缝开发对照与原内点评价 | `sessions/ring_phase_dev_20261003/`、`local_data/evaluation/ring_phase_20261003/` | 原 15.396 px 开发失败、新 0.690 px、141 个原接受窗口的带符号网格诊断、冻结计划和汇总哈希；原失败不覆盖 |
| 未调参轨道生成资产 | `local_data/stage_b/ring_holdout_seed20261005/` | 仅改变轨道起伏种子，共享不可变光学资产；源配置、世界和标定不变；含生成端私有配置，不分享为公开重建输入 |
| 双圈共同域协议新采 | `sessions/d3_common_target_3m_20261003/`、`local_data/evaluation/d3_common_target_3m_20261003/` | `81fe051` 冻结复采，公共输入复现、采集验收与协议检查通过；已规划接缝通过，目标外及不可规划位置单列。保留原图和独立重成像 |
| 共同域 v1 失败反例 | `sessions/d3_common_domain_3m_20261003/`、`local_data/evaluation/d3_common_domain_3m_20261003/` | 目标末端 9 点缺测的旧规则失败及公开支撑诊断；不能用新协议覆盖旧报告 |
| 新巡航余量 20 m 反例 | `sessions/d3_noise_20m_support_20261003/`、`local_data/evaluation/d3_noise_20m_support_20261003/` | 已规划共同域无缺测，但窗口间 P95 4.232 px 仍失败；原协议与补充 v2 评价分别保留，新批次尚未生成优化整幅输出 |
| README 未补偿螺旋对比 | `sessions/d3_noise_20m_v2_20261003/feature_review_rawleft/`、`local_data/readme_export_rawleft_20m_20261003/` | 旧采集原图和重建不变，`284e265` 生成原始左图、优化右图，逐像素与哈希核验通过；当前 8769 特征复核页 |
| 双圈支撑回归证据 | `local_data/evaluation/double_support_fix_20261003/` | 日志归档、完整 XML、真实空隙/旧实现变异检查、导出像素校验及清理清单 |
| 默认完整演示及标定 | `local_data/stage_b/contact_demo/` | Gazebo/RViz 默认入口；当前采集 250°、输出 240°、增益 2.4，带 2 mm 档轨道起伏。旧光学场景及 `*_baseline_240_20261003` 配置/标定保留，新旧会话不能混用标定 |
| 三种公开素材原图 | `local_data/stage_b/sources/` | 从网站原图重新生成资产 |
| 原生成链及旧演示配置 | `local_data/stage_b/` 下 c034、几何、缺陷、GUI、砂浆及 unbundled 目录 | 历史评价仍记录原资产身份和路径，暂不清理 |
| 历史画质基线复核页 | `local_data/stage_b/final_review_20261001/` | 增益调整前已确认的背景、裂缝、填缝与照明局部图 |
| 现行增益画质确认 | `sessions/stage_d3_3m_review_20261003/gain_comparison/` | 2026-10-03 已确认响应增益 2.4；包含原图 DN 投影及采后校正对照 |
| D3 正式 3 m 整幅对比 | `sessions/d3_holdout_rebuilt_20261003/full_mosaic/`、`local_data/evaluation/d3_global_mosaic_20261003/` | 名义/优化全图的 `mosaic_u16/count` 大数组已于 2026-10-03 删除（哈希留在 `full_mosaic/provenance.json`，可重新生成），保留覆盖游程、预览和报告；整幅回读、固定 CPU 核对和运行日志。没有新增原始图副本；原始留出采集继续保留 |
| 3 m 噪声验证 | `sessions/d3_noise_assumed_20261003/`、`local_data/evaluation/d3_noise_assumed_20261003/` | 原图、独立重放、D1/D2/D3、两幅全图的核验记录（大数组已删，可按溯源重新生成）；重放原图为独立校验后的硬链接 |
| 噪声档资产与标定 | `local_data/stage_b/noise_assumed_20261003/`、`local_data/evaluation/sensor_noise_implementation_20261003/` | 独立标定和生成参数；只读素材与默认演示硬链接，不能原位改写共享文件。495 项测试及默认关闭后的兼容性证据 |
| 无噪声兼容重放 | `sessions/noise_off_compatibility_20261003/` | 98 个二进制文件与旧正式会话相同；重复原图校验后链接原始块，保留元数据和溯源 |
| 新 20 m 带噪声完整输出 | `sessions/d3_noise_20m_v2_20261003/`、`local_data/evaluation/d3_noise_20m_v2_20261003/` | 原图、独立重成像、D1/D2/D3、两幅全图的报告、预览与哈希（约 32.3 GiB 大数组已删，可按溯源重新生成）、逐像素核验及复核页；整体接缝因末圈 15 个双圈点缺测仍失败 |
| 首次 20 m 规模失败记录 | `sessions/d3_noise_20m_20261003/`、`local_data/evaluation/d3_noise_20m_20261003/` | D3 来源检查发现 39 个边缘坐标不一致；保留原图和失败链条，不称为优化验收通过；重复原图已按哈希去重 |
| 20 m README 对比源 | `local_data/readme_export_20m_final_20261003/`、`docs/media/` | 候选来源、逐像素等同核验和导出溯源，已入 Git 的只有公开 PNG 与媒体清单 |
| 20 m 原始采集与诊断 | `sessions/stage_c_20m_acceptance_20261002/`，不含已删除的 `wall_replay/` | 后续 D1/D2/D3、原始采集和独立评价基线 |
| 20 m 任务私有输入 | `local_data/mission_runs/20261002_113730_eae97f12/` | 生成和独立评价；不是重建输入 |
| v1 CPU 展开（瘦身） | `sessions/stage_d1_3m_final_20261002/` | 报告、溯源、预览及 8765 复核页；大数组已删，v2 等价记录见 STAGE_D |
| v1 CUDA 展开（瘦身） | `sessions/stage_d1_3m_cuda_final_20261002/` | 报告与溯源；后续匹配输入改用 `sessions/stage_d1_3m_v2_20261003/` |
| 最终 D2 与公开输入证明 | `sessions/stage_d2_public_verified_20261002/` | 对应点、带真值的独立评价及公开数据隔离证明，8766 复核页 |
| D2 偏置来源对照 | `sessions/bias_study_20261003/`、`local_data/bias_study_20261003/` | 平法线/仅反照率变体原始采集、v2 展开与匹配、带符号网格评价（约 18 GiB）；平法线 v2 展开为 D3 实现检查基线；原光照重放仅保留逐字节一致记录 |
| D1 v2 验收 | `sessions/stage_d1_3m_v2_20261003/` | v2 展开（25 MB）、D2、CUDA/CPU 公开复现及 D2 公开复现；`public_cuda` 内原图为硬链接 |
| 20 m D1/D2 v2 | `sessions/stage_d_20m_v2_20261003/` | 完整 20 m 展开（137 MB）、D2 对应点及带符号网格评价；D3 的 20 m 输入 |
| 3 m D3 历史诊断与对比 | `sessions/stage_d3_3m_review_20261003/`、`sessions/stage_d3_3m_20261003/` | `robust/`、`features_final/` 与 `frozen_v2_*` 是历史对照；公开链接仍保留。被替代的 `frozen_material_review`、`frozen_flat_review`、`frozen_material_features` 已归档清理，不替代最新采集 |
| D3 平轨与前轮冻结复核 | `sessions/stage_d3_3m_frozen_20261003/`、`local_data/evaluation/stage_d3_frozen_20261003/` | 保留辅助平轨反例及带起伏历史结果，记录光度偏差可能使真实几何略变差；不作为主要交付场景 |
| 最新带起伏 3 m 复核 | `sessions/d3_guard_acceptance_20261003/`、`local_data/evaluation/guard_fix_20261003/acceptance/` | 完整原图、公开观测、生成端真值、D1/D2/D3、全原图饱和检查和公开搬迁复现；后续优化全分辨率输出的主要输入。复现原图为硬链接 |
| 最新板缝/裂缝复核页 | `sessions/stage_d3_3m_review_20261003/guarded_final/` | 当前 8767 复核页，原始螺旋条带、名义展开与优化硬接缝；位置只从公开图像选择 |
| D3 测试与输入边界证据 | `local_data/evaluation/stage_d3_3m_20261003/` | 冻结代码、406 项回归、假匹配回归与公开搬迁完全一致记录；仅独立证据，重建不读取 |
| D3 原始条带及特征复核证据 | `local_data/evaluation/stage_d3_features_20261003/` | `final_report.json` 为当前页面的生产输入审计、代码身份与产物哈希；原始位姿和缺陷真值未用于位置选择。只保留正式页面，三个 `feature_probe*` 临时生成目录已清理 |
| 早期采集基线 | `sessions/gz_a/`、`sessions/default_acceptance/` 及其 dynamics | 阶段 A/B 基线，不保留重复的重放原图 |
| 双测量轮试验 | `sessions/measuring_trials/` | 轨道起伏、横滚/俯仰及里程基线 |
| README 源视频 | `local_data/readme_media_20261002/` | 可重新生成已入 Git 的 GIF/PNG；任务原始行图另已清理 |

部分文件采用硬链接，按目录分别执行 `du` 后再相加会重复计算。后续评估保留数据时既看运行依赖，也看评价端依赖，不能只因默认演示已独立打包就删除全部旧生成链。

D1 v2 不保留原始列浮点缓存，因此其后续匹配仍依赖 `native_source.json` 所列的原图块。迁移 D1 时需同时保留这些块，并通过 D2 的 `--raw` 指定迁移后的原图目录；不能因展开产物已生成就删除原始采集。

## 2026-10-03 删除

角向保护与饱和修复后，新增清理 **13 个目录**，释放约 **3.17 GiB**：

- 被冻结后新重采替代的 `sessions/d3_guard_gain_probe_20261003/` 开发会话及 `local_data/stage_b/d3_guard_gain_dev_20261003/` 开发演示/标定工作目录。
- 三份已由 `frozen_v2_*` 和最新 `guarded_final/` 替代的重复页面。
- 未采用的横移/升沉、仅升沉、部分节点布局试验；保留仍被历史复现引用的 `matches02` 及现行输入，不清理整棵残差研究目录。
- 三个已结束的旧测试工作目录 `/tmp/ssb_d3_{evaluation_suite,step2_suite,step2_final_suite}`；最新 444 项测试日志 `/tmp/ssb_guard_full_suite/` 保留。

删除前将 255 个必要报告、配置、测试日志/XML 和标定靶图片存成单个约 18.2 MiB 的压缩包，并逐条核对包内 SHA-256；不复制出大量小文件。原图、大数组和大块逐行诊断未收入此开发归档，需要时应重新生成，不能将旧路径当成仍可运行的会话。清单与归档：

```text
local_data/evaluation/cleanup_guard_20261003/
  cleanup.json                         删除目录、尺寸、归档哈希与保留范围
  reports_configs_calibration.tar.gz   单个开发/旧测试证据归档
  preserved_check.json                 默认演示全部哈希与保留目录检查
```

原 20 m 采集、已确认画质、材质源图、旧冻结对照和最新带起伏原图/产物未删除。清理后默认演示所有运行文件的哈希重新核对通过。默认包中 `capture_update.calibration_source` 是被归档的开发来源标签；运行只读取默认目录内已复制的新标定，不依赖已删除开发目录。

D1 改为 v2（按需读原图）后，删除 4 份 v1 展开中可重新生成的大数组，共 21 个文件、约 **47.6 GiB**：`sensor_flat.npy`、`sensor_valid_bits.npy`、`mosaic.npy`、`coverage.npy`、`source_band.npy`，以及 D2 公开证明目录中 `public_d1/sensor_flat.npy` 这个硬链接。涉及 `stage_d1_3m_final_20261002`、`stage_d1_3m_cuda_final_20261002`、`bias_study_20261003/{flatnormal,albedo}_d1`。

- 删除前用 v2 重新展开两个变体，重跑 D2，结果与原 D2 逐字节一致；v1 与 v2 的逐项等价记录在 STAGE_D。
- 保留每个目录的报告、溯源、投影表、映射、预览和复核页，8765/8766 复核页仍可访问。旧 D2 溯源中引用的 v1 数组哈希不能再从原文件核对，由 v1/v2 等价记录代替。
- 清单（路径、字节数、来自 D1 溯源的 SHA-256、原因）：`local_data/evaluation/cleanup_20261003/cleanup.json`。删除后可用空间约 736 GiB。

## 2026-10-02 删除

完成以下清理，共 58 个明确指定的目录：

- 三份早期 D1 展开与公开复测副本，保留最终 CPU/CUDA 版本。
- 两份被最终公开隔离验证替代的早期 D2 结果。
- 默认演示和 20 m 验收的重放原图副本；保留原始采集和已通过的重放报告。
- `repair_joint_3m_20261002`、`repair_mission_suite_20261002` 修复试验，以及被完整 20 m 基线替代的 `wall_target_3m_20261002` 试验及输入。
- 7 份已完成的 RViz/README 录制任务原图和不再使用的任务私有输入；保留视频、完成记录及验证元数据。
- 从公开原图重新生成演示的 smoke 工作目录和测试演示包；保留原图、配方代码和生成验证报告。
- 条光阴影各轮已结束的开发测试目录；保留相关记录、截图及当前代码。

按已分配磁盘块及硬链接统计，删除释放约 **70.02 GiB**。报告/配置归档约 **215 MiB**；清理后本机文件系统可用空间约 **730 GiB**。这些是 WSL Linux 文件系统内的空间，不代表 Windows 上的 VHDX 文件已缩小，也不代表 Windows 磁盘立即释放相同容量。

## 本地归档

```text
local_data/evaluation/cleanup_20261002_public/
  cleanup.json                  完整删除/保留清单、尺寸、时间及执行结果
  reports_and_configs.tar.gz    单个归档文件：报告、配置、元数据清单和必要图片
```

归档前逐文件核对原始内容与 tar 中的 SHA-256，共 9,826 个文件；不包含删除目录中的原始行图、大数组、二进制位姿流或网格。归档采用一个压缩包，避免 WSL 再复制出大量小文件。归档含生成端私有配置，仅保存在本机，不纳入 Git。

历史报告中的输入/输出路径是当时的记录，清理后不能据此声称那些大数据仍存在。重新验证需按原图、保留配置和当前工具生成新的会话，不修改旧报告或降低哈希验收门限。README 媒体清单已标明原始采集行图的清理状态，视频及最终媒体仍保留。

## 清理后检查

检查现行演示全部资产哈希、物理装配和标定身份，20 m 原图/公开元数据，最终 CPU/CUDA 展开和 D2 公开输入及匹配结果的哈希。复核服务使用的目录保留原位；没有终止正在运行的复核服务。检查结果写入本地 `preserved_data_check.log`。

后续继续手动清理；在完成 20 m 全图前保留本页的原始基线和最终展开/匹配输入。D1 v2 存储优化已完成；50 m 扩展前需按新布局检查句柄、内存、磁盘和耗时预算，不能依靠反复删除独立证据来腾空间。


## 2026-10-03 留出区段验证与画质对照

保留 `sessions/d3_holdout_12_15_20261003/` 的 3 m 原始采集、公开重建及硬链接复现，约 1.7 GiB；协议、冻结检查与两组网格评价在 `local_data/evaluation/d3_holdout_20261003/`。开发区段原图和历史 20 m 原图继续保留。新增复核页 `sessions/stage_d3_3m_review_20261003/{holdout_12_15,gain_comparison}/`；2026-10-03 用户已确认增益 2.4 为新画质基线。独立确认记录保存在 `local_data/evaluation/d3_holdout_20261003/visual_acceptance.json`；不改写确认之前的生成报告或其哈希链。

只删除本轮被成功完整回归替代的 `/tmp/ssb_review_repairs_suite/` 测试工作目录（约 522 MiB），保留最终 `/tmp/ssb_review_repairs_suite_v2/`。删除前将两轮日志、XML 和摘要批量保存到私有评价目录的单个 `test_reports.tar.gz`，逐文件核对内容哈希；没有删除原图、独立验证证据或正在服务的复核页面。


## 2026-10-03 留出采集构建溯源修复

现行正式 3 m 证据为 `sessions/d3_holdout_rebuilt_20261003/`，配套协议与评分在 `local_data/evaluation/d3_holdout_rebuilt_20261003/`。旧 `d3_holdout_12_15_20261003` 保留为历史原图/数值结果与新增门限的反向验证，不把其二进制身份 false 改为 true。

新旧两次采集各自完成独立重成像；验收后仅对重复原图作哈希核对和硬链接去重，174 块释放约 2.71 GiB，内容/路径/元数据/独立重成像报告均保留。明细为 `replay_deduplication.json`。删除被最终回归替代的 `/tmp/ssb_holdout_provenance_suite/`，最终 470 项测试目录保留；两轮日志/XML/摘要已核验并批量归档为单个 `test_reports.tar.gz`。

## 2026-10-03 新 20 m 数据去重

两个 20 m 批次均独立重成像并通过原图/表文件比较，之后才按实际 SHA-256 和尺寸共享不可变原图块。两个重成像各 495 块、第二批采集与第一批相同的 494 块改为硬链接，共释放 **23.18 GiB**（24,894,095,360 字节）。不删除失败批次、原始曝光、元数据、独立评价或重建输入，不把第二批不同的一个块强行当成相同；内容与所有路径/哈希记录保留。明细见两份 `replay_deduplication.json`。

删除三个已结束回归目录中的 12 个临时夹具目录，以及两份被正式导出替代的临时 README 图片目录。日志、XML、摘要仍保留，三个测试归档逐文件核验通过；正式 501 项测试报告及 README 导出溯源不删。清单 `local_data/evaluation/d3_noise_20m_v2_20261003/cleanup.json`。

## 2026-10-03 双圈支撑与展示修复

旧 20 m 失败数据与全分辨率图保留；README 和 8769 的主要对比左图更新为未补偿螺旋的原始条带，名义展开作为中间状态。新巡航余量 20 m 批次的真实窗口间精度失败、共同域 v1 的 3 m 边界失败、冻结后 v2 的 3 m 复核分别归档，不因新规则而改写历史报告。评价真值与密钥不加入 Git 或复核网页。

两份完成的测试套件清理了 8 个临时夹具目录，逻辑文件约 1.085 GB；没有删除采集会话、原图或独立评价。删除前逐文件核验日志压缩包，最终测试 XML 另行归档。`final_test_counts.json` 区分 441 个 Python、73 个 C++ 独立用例与 5 个 CTest 套件，colcon 总计 519、独立用例 514，全通过无跳过。清单见 `fixture_cleanup.json`。清理在 Linux 文件系统内一次批量完成，没有生成逐图块小文件或对正式原图重复拷贝。

## 2026-10-03 环缝稳健性修复

高/低占比新种子采集、开发反例与完整 20 m 复算均按上表保留，旧 20 m 4.232 px 失败记录仍在原目录；没有删除或更改其哈希。重建没有新复制完整原图，公开输入副本使用已核验内容的硬链接；轨道变体仅重新生成物理轨面，共享纹理，避免重复保存大型光学资产。

两次完成的回归夹具批量清理 8 个目录、逻辑文件约 1.087 GB；先核验 `regression_logs.tar.gz` 内 31 个日志/XML/摘要，73 个 C++ 用例 XML 单独归档。最后的源资产保护回归为 536 项、无失败/跳过，报告另存 `final_regression_logs.tar.gz`；最终另清理 4 个夹具目录、约 0.544 GB，记录在 `final_fixture_cleanup.json`，合计约 1.63 GB。20 m 运行日志另存单个 `runtime_logs.tar.gz`。清理只删已结束测试的可再生成数据，不删正式原图、独立重成像或评价文件。

## 2026-10-03 离线提速后清理

清理记录：`local_data/evaluation/cleanup_20261003_offline/cleanup.json`。合计释放约 **72.4 GB**，磁盘可用空间从 667 GB 增加到 740 GB。

- **无损去重（30.2 GB，1,801 个原图块）**：以下副本经 SHA-256 核对与原件相同后，改为指向原件的硬链接，路径和文件哈希都不变：
  - `d3_noise_20m_support` 的重成像原图块；
  - `d3_noise_20m_v2` 公开复现目录中的原图块；
  - 6 组 3 m 会话的重成像原图块；
  - `perf_ring_{high,low}` 的采集原图块（与 `ring_{high,low}_seed20261005` 的采集逐字节相同）。

  独立重成像的逐字节一致性早在去重前已由各自的阶段 B 报告确认。去重后，同一会话再做重成像比对必然一致，不能再算作新的独立证据。
- **删除可再生成的整幅数组（44.9 GB）**：删除了 `d3_noise_20m_v2`、`d3_holdout_rebuilt`、`d3_noise_assumed` 三处 `full_mosaic/{nominal,optimized}/mosaic_{u16,count}.npy`，删除前逐个核对过与溯源记录的哈希一致。覆盖游程、预览图、报告和溯源都保留；原图、D1、D3 仍在，可用 `ssb_tools.global_mosaic` 重新生成，并按记录的哈希核验。相关复核页上指向这些大数组的链接已失效。
- **删除旧 colcon 日志（0.49 GB，349 项）**：只保留 `log/COLCON_IGNORE`。正式测试日志另有归档。

**操作失误记录**：抽查去重结果时，`validate_stage_b --compare` 把重新生成的报告写回了 `sessions/ring_high_seed20261005_20261003` 和 `sessions/d3_noise_20m_support_20261003` 的 `capture/evaluation/reports/stage_b_smoke.json`，原文件被覆盖。原报告没有单独留存哈希，因此无法证明两者逐字节相同。不过，重新生成时使用的检验代码（`validate_stage_b`、`validate_stage_a`、`ref_geometry`、`session`）自这两次采集以来没有改动，输入文件内容也相同；24 项检查名称和结果与 `local_data/evaluation/*/stage_b.log` 中的原始运行日志逐项一致，全部通过。以后抽查请先把会话复制到临时目录，或改为只读检查。
