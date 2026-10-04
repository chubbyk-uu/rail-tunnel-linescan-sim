# 数据保留与清理

更新：2026-10-04。大型本机资产和会话不进 Git。清理只针对已完成的重复计算、重放及界面测试；生成、采集、重建中的文件不能删除。

## 当前保留

| 数据 | 位置 | 用途 |
|---|---|---|
| 里程停车与三档轮径试验 | `sessions/wheel_error_20261004/`、`local_data/evaluation/wheel_error_implementation_20261004/` | 保留三档真实接触、原图、独立重成像、D1/D2/D3、25 项采集验收、公开输入审计及逐点几何；81 mm 主门限失败原样保留。开发失败诊断和 595 项测试日志集中归档；正式原图未去重 |
| 当前 README 对比源 | `sessions/audit_guard_high_20261003/feature_review_readme_20261004/`、`local_data/readme_export_final_20261004/` | 最新验收会话的公开图像复核与 1048×1092 组合图；只读原始输入，不生成整幅大数组；图片/哈希说明在 docs/media |
| 审计与报告保护后复采 | `sessions/audit_guard_high_20261003/`、同名 `local_data/evaluation/`；`local_data/evaluation/audit_guards_tests_20261003/` | 冻结 `6795971`，已知高环缝区段；阶段 B 24 项、协议 v5 的 14 项及实际审计通过。独立重成像原图本轮未去重；保留报告哈希、5,787 个逐点比较及 581 项回归日志归档 |
| 离线提速后冻结复核 | `sessions/perf_ring_{high,low}_20261003/`、`local_data/evaluation/perf_tier2_20261003/` | `d60e15d` 上用同一轨道种子与区段重采；协议 v4、公开运行审计和独立评分均通过，与原冻结结果的接缝分数一致到小数点后 6 位；每组约 3.8 GB |
| 环缝高/低占比新轨道留出 | `sessions/ring_{high,low}_seed20261005_20261003/`、同名 `local_data/evaluation/` 目录 | 冻结 `b206f6d`、未用于调参的轨道种子；保留原图、独立重成像、D1/D2/D3、阶段 B 与协议核验、公开输入复现 |
| 环缝修复完整 20 m 复算 | `sessions/ring_phase_20m_corrected_20261003/`、`local_data/evaluation/ring_phase_20261003/corrected_20m_*` | 使用第二批原图/D1，窗口间 P95 0.691 px；新匹配、轨迹、公开复现和真实网格评分，不是新 20 m 采集；没有新整幅图 |
| 环缝开发对照与原内点评价 | `sessions/ring_phase_dev_20261003/`、`local_data/evaluation/ring_phase_20261003/` | 原 15.396 px 开发失败、新 0.690 px、141 个原接受窗口的带符号网格诊断、冻结计划和汇总哈希；原失败不覆盖 |
| 未调参轨道生成资产 | `local_data/stage_b/ring_holdout_seed20261005/` | 仅改变轨道起伏种子，共享不可变光学资产；源配置、世界和标定不变；含生成端私有配置，不分享为公开重建输入 |
| 双圈共同域协议新采 | `sessions/d3_common_target_3m_20261003/`、`local_data/evaluation/d3_common_target_3m_20261003/` | `81fe051` 冻结复采，公共输入复现、采集验收与协议检查通过；已规划接缝通过，目标外及不可规划位置单列。保留原图和独立重成像 |
| 共同域 v1 失败反例 | `sessions/d3_common_domain_3m_20261003/`、`local_data/evaluation/d3_common_domain_3m_20261003/` | 目标末端 9 点缺测的旧规则失败及公开支撑诊断；不能用新协议覆盖旧报告。2026-10-03 起只保留报告、溯源与日志，原图及大数组已删，不能再复现 |
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
| 首次 20 m 规模失败记录 | `sessions/d3_noise_20m_20261003/`、`local_data/evaluation/d3_noise_20m_20261003/` | D3 来源检查发现 39 个边缘坐标不一致；不称为优化验收通过。2026-10-03 起本目录只保留报告、溯源与日志；同一次采集的原图仍在 `d3_noise_20m_v2_20261003/capture/raw` |
| 20 m README 对比源 | `local_data/readme_export_20m_final_20261003/`、`docs/media/` | 候选来源、逐像素等同核验和导出溯源，已入 Git 的只有公开 PNG 与媒体清单 |
| 20 m 原始采集与诊断 | `sessions/stage_c_20m_acceptance_20261002/`，不含已删除的 `wall_replay/` | 后续 D1/D2/D3、原始采集和独立评价基线 |
| 20 m 任务私有输入 | `local_data/mission_runs/20261002_113730_eae97f12/` | 生成和独立评价；不是重建输入 |
| v1 CPU 展开（瘦身） | `sessions/stage_d1_3m_final_20261002/` | 报告、溯源、预览及 8765 复核页；大数组已删，v2 等价记录见 STAGE_D |
| v1 CUDA 展开（瘦身） | `sessions/stage_d1_3m_cuda_final_20261002/` | 报告与溯源；后续匹配输入改用 `sessions/stage_d1_3m_v2_20261003/` |
| 最终 D2 与公开输入证明 | `sessions/stage_d2_public_verified_20261002/` | 对应点、带真值的独立评价及公开数据隔离证明，8766 复核页 |
| D2 偏置来源对照 | `sessions/bias_study_20261003/`、`local_data/bias_study_20261003/` | 平法线变体原始采集、两种变体的 v2 展开与匹配、带符号网格评价；平法线 v2 展开为 D3 实现检查基线；仅反照率变体的原图与大数组已于 2026-10-03 删除，只保留报告和溯源；原光照重放仅保留逐字节一致记录 |
| D1 v2 验收 | `sessions/stage_d1_3m_v2_20261003/` | v2 展开（25 MB）、D2、CUDA/CPU 公开复现及 D2 公开复现；`public_cuda` 内原图为硬链接 |
| 20 m D1/D2 v2 | `sessions/stage_d_20m_v2_20261003/` | 完整 20 m 展开（137 MB）、D2 对应点及带符号网格评价；D3 的 20 m 输入 |
| 3 m D3 历史诊断与对比 | `sessions/stage_d3_3m_review_20261003/`、`sessions/stage_d3_3m_20261003/` | `robust/`、`features_final/` 与 `frozen_v2_*` 是历史对照；公开链接仍保留。被替代的 `frozen_material_review`、`frozen_flat_review`、`frozen_material_features` 已归档清理，不替代最新采集 |
| D3 平轨与前轮冻结复核 | `sessions/stage_d3_3m_frozen_20261003/`、`local_data/evaluation/stage_d3_frozen_20261003/` | 保留辅助平轨反例及带起伏历史结果，记录光度偏差可能使真实几何略变差；不作为主要交付场景 |
| 历史带起伏 3 m 复核 | `sessions/d3_guard_acceptance_20261003/`、`local_data/evaluation/guard_fix_20261003/acceptance/` | 完整原图、公开观测、生成端真值、D1/D2/D3、全原图饱和检查和公开搬迁复现；后续优化全分辨率输出的主要输入。复现原图为硬链接 |
| 历史板缝/裂缝复核页 | `sessions/stage_d3_3m_review_20261003/guarded_final/` | 当前 8767 复核页，原始螺旋条带、名义展开与优化硬接缝；位置只从公开图像选择 |
| D3 测试与输入边界证据 | `local_data/evaluation/stage_d3_3m_20261003/` | 冻结代码、406 项回归、假匹配回归与公开搬迁完全一致记录；仅独立证据，重建不读取 |
| D3 原始条带及特征复核证据 | `local_data/evaluation/stage_d3_features_20261003/` | `final_report.json` 为当前页面的生产输入审计、代码身份与产物哈希；原始位姿和缺陷真值未用于位置选择。只保留正式页面，三个 `feature_probe*` 临时生成目录已清理 |
| 早期采集基线 | `sessions/gz_a/`、`sessions/default_acceptance/` 及其 dynamics | 阶段 A/B 基线，不保留重复的重放原图 |
| 双测量轮试验 | `sessions/measuring_trials/` | 轨道起伏、横滚/俯仰及里程基线 |
| README 源视频 | `local_data/readme_media_20261002/` | 可重新生成已入 Git 的 GIF/PNG；任务原始行图另已清理 |

部分文件采用硬链接，按目录分别执行 `du` 后再相加会重复计算。后续评估保留数据时既看运行依赖，也看评价端依赖，不能只因默认演示已独立打包就删除全部旧生成链。

D1 v2 不保留原始列浮点缓存，因此其后续匹配仍依赖 `native_source.json` 所列的原图块。迁移 D1 时需同时保留这些块，并通过 D2 的 `--raw` 指定迁移后的原图目录；不能因展开产物已生成就删除原始采集。

## 清理原则与历史缺口

历次删除、去重、归档哈希及操作失误原样保存在 [清理快照](history/DATA_RETENTION_SNAPSHOT_2026-10-03.md)。清理前确认不再被运行或评价依赖；优先少量压缩归档和顺序大块 I/O，正式原图不因小型 D1 已生成就删除。

硬链接按目录相加会重复计空间；完成独立渲染核验后才可去重，链接副本重新比对不能当作新独立验证。两份旧报告（ring_high_seed20261005、d3_noise_20m_support 的 stage_b_smoke.json）曾被覆盖，原字节身份没有恢复；现行验收器拒绝覆盖，复查使用只读模式。

部分旧全图大数组和三处历史反例的原图/大表已删除，报告与预览不能称可运行的完整会话。请按表和快照判断保留范围，不按旧文中的路径推断文件仍存在。


2026-10-04 文档整理只清理本轮已完成测试的 4 个临时夹具目录和初次展示导出草稿；最终公开图像复核、干净提交导出、原始采集与独立重成像全部保留。测试/XML/构建及媒体核对记录先集中打包、校验后清理，清单见 `local_data/evaluation/readme_restructure_20261004/summary.json`，没有修改历史验收报告。

2026-10-04 轮径试验清理：验证集中归档的 387 个文件哈希后，删除本轮三个开发目录及 12 个测试临时夹具目录，逻辑占用约 5.66 GB。开发原图和大时序表已删除，归档不再是完整可运行会话；保留首次失败报告、修正复核报告与物理诊断。最终三档全部原图、独立重成像和重建产物仍保留，清单和归档哈希见 `local_data/evaluation/wheel_error_implementation_20261004/cleanup.json`。
