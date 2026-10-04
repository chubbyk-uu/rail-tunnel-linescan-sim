# 数据保留与清理

更新：2026-10-04。大型本机资产和会话不进 Git。清理只针对已完成的重复计算、重放及界面测试；生成、采集、重建中的文件不能删除。

组合装配验收调整：按用户要求取消九组单项矩阵，删除 `mount_seed20261103/` 中从未采集的八套单项导出包及对应生成目录，释放约 13.62 GiB。生成配置、世界清单、标定夹具图片/结果和日志集中保存在 `local_data/evaluation/mount_cancelled_matrix_20261004/generation_and_calibration.tar.gz`（约 2.81 MiB），读取校验通过；`cleanup.json` 记录范围与归档哈希。该归档仅保留来源及标定证据，不能直接运行。名义包、新组合包以及所有已经采集的开发原图、独立重成像和评价均保留。

最新 review 数据固化：本轮三档 3 m 和新采 20 m 的原图、独立重成像、曝光/编码器、D1、匹配、轨迹和共享深度已从 `/tmp` 直接迁入 `sessions/review_fixes_distance_20261004/` 与 `sessions/review_fixes_mission_20m_20261004/`。连同旧 20 m 整图的覆盖游程、预览和报告，共约 29.45 GiB；3,200 个文件路径、2,405 个独立 inode 的迁移前后哈希一致，原有硬链接关系保留，没有创建新硬链接或软链接。旧 `/tmp` 采集根目录已不存在，四批长期路径的实际读取及 CUDA/CPU 深度探针验证通过。原始报告、清单及首次验收字节不改写；路径映射在 `local_data/evaluation/review_fixes_20261004/relocation.json`，读取复核在 `relocation_readers.json`。

此前按用户指定删除 16 个测试临时夹具目录，释放约 2.10 GiB，48 个日志/XML 等文件和 155 项归档哈希均核验保留，记录为同目录 `cleanup_tests_20261004.json`。数据迁移后，按用户指示清理本轮剩余 `/tmp/ssb_fix_*` 数据及一个归档辅助脚本，共 44 个顶层条目；其中旧 20 m 可重建的 `mosaic_u16.npy`、`mosaic_count.npy` 已删除。删除前再次核验长期目录的 3,200 个文件路径、2,405 个独立文件哈希和原 155 项证据；其余 790 个诊断/日志文件集中压缩并逐项校验，归档约 41.45 MiB，净释放约 16.25 GiB。记录为 `cleanup_tmp_20261004.json`，补充归档为 `tmp_retained_before_cleanup_20261004.tar.gz`。其他 `/tmp` 数据未处理；长期保留的预览/覆盖目录不包含两幅像素数组，整图回读需要按原图和参数重新生成。

历史共享深度修复收尾：50 个正式证据哈希、两个导出包各 344 个文件均核对通过，217 项开发与测试记录逐项校验后集中归档（约 61 MB）。删除当轮开发目录、导出前中间世界和 16 个测试临时夹具目录，按 inode/硬链接计数预计净释放约 2.13 GiB；保留在正式会话中的原图硬链接不算释放空间。正式采集与独立重成像未删除或去重，首次缺测候选与失败测试日志仍在归档。

## 首尾支撑修复后的保留数据（2026-10-04）

| 用途 | 路径 | 范围 |
|---|---|---|
| 新种子 3 m 正式验收与展示 | `sessions/boundary_crop81_holdout_20261004/`、同名 `local_data/evaluation/` | 81 / 80 mm，独立原图与重成像、公开重建、取点 v4／报告 v5、16 项协议及图片来源全部保留 |
| 新种子 20 m 正式验收与完整全图 | `sessions/boundary_crop20_holdout_20261004/`、同名 `local_data/evaluation/` | 80 / 80 mm、假设噪声；原图、独立重成像、重建、整图数组、完整覆盖与 CPU 探针、SIGTERM 中断及恢复记录保留 |
| 三档受控回归 | `sessions/boundary_wheel_20261004/`、`sessions/boundary_crop_wheel_20261004/`、`local_data/evaluation/boundary_crop_wheel_20261004/` | 前者保存三档物理与采集／独立重成像，后者读取同一原图作修复后的展开与公开优化；不是三批新的盲验 |
| 首轮余量补修失败 | `sessions/boundary81_holdout_20261004/`、`sessions/boundary20_holdout_20261004/`、同名 evaluation | 81 mm 缺测失败及 20 m 重建前停止；保留反例，不覆盖或改称通过 |
| 审计与边界修复证据 | `local_data/evaluation/boundary_repairs_20261004/` | 723 项测试、反向回注、源记录诊断、汇总与日志／程序归档；`cleanup_tests.json` 只删除本轮临时测试夹具 |

新增正式原图、独立重成像和全图数组保留用于复核，当前不删除。大块二进制顺序读写，无逐行／逐像素小文件；不批量改写历史验收报告。下文旧 review 的失败结论保留其当时规则与数据身份。

## 当前保留

| 数据 | 位置 | 用途 |
|---|---|---|
| review 修复后的三档 3 m 新采 | `sessions/review_fixes_distance_20261004/images_{80,79,81}/` | 保留各批 `capture/`、独立 `reimage/`、`capture_dynamics/`、`unroll/`、`production/` 和独立几何报告；已知种子开发回归，已测 P95≤1 px，但新严格门限各有 828 个缺测诊断点而失败 |
| review 修复后的 20 m 新采 | `sessions/review_fixes_mission_20m_20261004/` | 原图在 `sessions/20261004_164412_ba34f8df/`，独立重成像为 `wall_replay/`，保留 `unroll/`、`production/` 和独立评分；2,074,077 行，采集及图像一致性通过，严格接缝因 27 个缺测诊断点失败 |
| review 修复与迁移证据 | `local_data/evaluation/review_fixes_20261004/` | 691 项测试、故障注入、逐点评分、源码快照、迁移前后完整文件哈希和读取验证；`mosaic_retained/` 保留旧 20 m 整图报告、预览、完整覆盖游程，像素数组已清理、可重新生成；补充归档保留剩余临时诊断与日志 |
| 共享径向深度的两个独立留出 | `sessions/relief81_holdout_20261004/`、`sessions/relief81_cached_holdout_20261004/`、同名 `local_data/evaluation/`；`local_data/stage_b/relief81_seed20261013_bundle/`、`relief81_seed20261017_bundle/` | 分别冻结 `a16958f`、`857f5c7`，新种子 20261013、20261017，[8,11] m，真实 81 / 标定 80 mm。每次 25 项阶段 B、16 项协议 v7 通过；完整原图与实际独立重成像分别保留，未去重 |
| 共享深度与等价缓存回归 | `sessions/relief_final_20261004/`、`sessions/relief_cached_20261004/`、同名 `local_data/evaluation/` | 原 79/80/81 mm 及 20 m 高环缝噪声原图回归，主门限和四边支撑通过；缓存前后匹配、轨迹、深度数组、取点计划及逐点评价一致。汇总 `relief_cached_20261004/summary.json` 绑定 50 个证据哈希；不是新 20 m 轮径误差采集 |
| 共享深度根因及本轮开发归档 | `local_data/evaluation/relief_root_20261004/`；`local_data/evaluation/relief_cached_20261004/development_and_tests.tar.gz`、`archive_hashes.json`、`cleanup.json` | 保留真实网格/理想圆柱隔离诊断、缺测候选、原型图像、性能对照、失败测试及最终 661 项测试日志。归档排除重复原图、D1 投影和临时测试夹具，不是完整可运行会话；逐项校验后清理本轮开发目录和导出前中间世界 |
| 可选局部姿态加密的新轮径留出 | `sessions/wheel81_holdout_final_20261004/`、同名 `local_data/evaluation/`；`local_data/stage_b/wheel81_seed20261009_bundle/` | 冻结 `05717c7`、新种子 20261009、[12,15] m、真实 81 / 标定 80 mm。25 项采集、15 项协议及公开输入审计通过；接缝 0.492 / 0.670 px。完整原图与真正独立重成像分别保留，未去重；资产包为相对资源引用 |
| 密集取点的轮径反例与原模型对照 | `sessions/wheel_adaptive_regression_20261004/`、`local_data/evaluation/wheel_adaptive_20261004/` | 保留公开 D2/D3、0.1 m 取点的两种模型评分、逐点和材质诊断、36 个输入身份及汇总。旧 80/81 mm 在窗口间仍失败；原始输入继续在 `wheel_error_20261004/`，此处不额外复制独立原图 |
| 局部加密开发及完整回归归档 | `local_data/evaluation/wheel_adaptive_20261004/development_and_tests.tar.gz`、`archive_hashes.json`、`cleanup.json` | 436 个归档项逐一验证，约 430 MB；含候选/拒绝模型、精简日志、629 项测试 XML、首次 CLI 参数缺口的证据及配置。排除原图和重复 D1 投影，不能冒充完整可运行采集；首批重复原图与最终重采的独立原图摘要分别核对相同后删除 |
| 里程停车与三档轮径试验 | `sessions/wheel_error_20261004/`、`local_data/evaluation/wheel_error_implementation_20261004/` | 保留三档真实接触、原图、独立重成像、D1/D2/D3、25 项采集验收、公开输入审计及逐点几何；81 mm 主门限失败原样保留。开发失败诊断和 595 项测试日志集中归档；正式原图未去重 |
| 当前 README 对比源 | `sessions/audit_guard_high_20261003/feature_review_readme_20261004/`、`local_data/readme_export_final_20261004/` | 2026-10-03 历史验收会话的公开图像复核与 1048×1092 组合图，尚未换成共享深度版本；只读原始输入，不生成整幅大数组；图片/哈希说明在 docs/media |
| 审计与报告保护后复采 | `sessions/audit_guard_high_20261003/`、同名 `local_data/evaluation/`；`local_data/evaluation/audit_guards_tests_20261003/` | 冻结 `6795971`，已知高环缝区段；阶段 B 24 项、协议 v5 的 14 项及实际审计通过。独立重成像原图本轮未去重；保留报告哈希、5,787 个逐点比较及 581 项回归日志归档 |
| 离线提速后冻结复核 | `sessions/perf_ring_{high,low}_20261003/`、`local_data/evaluation/perf_tier2_20261003/` | `d60e15d` 上用同一轨道种子与区段重采；协议 v4、公开运行审计和独立评分均通过，与原冻结结果的接缝分数一致到小数点后 6 位；每组约 3.8 GB |
| 环缝高/低占比新轨道留出 | `sessions/ring_{high,low}_seed20261005_20261003/`、同名 `local_data/evaluation/` 目录 | 冻结 `b206f6d`、未用于调参的轨道种子；保留原图、独立重成像、D1/D2/D3、阶段 B 与协议核验、公开输入复现 |
| 环缝修复完整 20 m 复算 | `sessions/ring_phase_20m_corrected_20261003/`、`local_data/evaluation/ring_phase_20261003/corrected_20m_*` | 使用第二批原图/D1，窗口间 P95 0.691 px；新匹配、轨迹、公开复现和真实网格评分，不是新 20 m 采集；该环缝修复当时未生成整幅图 |
| 环缝开发对照与原内点评价 | `sessions/ring_phase_dev_20261003/`、`local_data/evaluation/ring_phase_20261003/` | 原 15.396 px 开发失败、新 0.690 px、141 个原接受窗口的带符号网格诊断、冻结计划和汇总哈希；原失败不覆盖 |
| 未调参轨道生成资产 | `local_data/stage_b/ring_holdout_seed20261005/` | 仅改变轨道起伏种子，共享不可变光学资产；源配置、世界和标定不变；含生成端私有配置，不分享为公开重建输入 |
| 双圈共同域协议新采 | `sessions/d3_common_target_3m_20261003/`、`local_data/evaluation/d3_common_target_3m_20261003/` | `81fe051` 冻结复采，公共输入复现、采集验收与协议检查通过；已规划接缝通过，目标外及不可规划位置单列。保留原图和独立重成像 |
| 共同域 v1 失败反例 | `sessions/d3_common_domain_3m_20261003/`、`local_data/evaluation/d3_common_domain_3m_20261003/` | 目标末端 9 点缺测的旧规则失败及公开支撑诊断；不能用新协议覆盖旧报告。2026-10-03 起只保留报告、溯源与日志，原图及大数组已删，不能再复现 |
| 新巡航余量 20 m 历史反例 | `sessions/d3_noise_20m_support_20261003/`、`local_data/evaluation/d3_noise_20m_support_20261003/` | 原模型已规划共同域无缺测、窗口间 P95 4.232 px 失败报告保留；同一原图的环缝与共享深度修复分别另存，不覆盖旧报告；该原模型没有整幅输出，后续共享深度整图已核验并保留预览与覆盖证明 |
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

本轮迁移后的历史 `native_source.json` 和审计报告仍记录当时的 `/tmp` 路径，不能直接使用旧路径。重建/评价传入长期 `--unroll`、`--trajectory`、`--session` 和显式 `--raw`；历史审计产物身份按 `relocation.json` 映射核对，迁移不重新生成首次验收。匹配重跑使用新输出目录；标定的字节相同副本在 `local_data/evaluation/review_fixes_20261004/relocated_inputs/calibration.json`。例如重新评分新采 20 m：

通用审计复核已支持显式迁移清单：`python3 -m ssb_tools.holdout_protocol audit-public --root sessions/review_fixes_mission_20m_20261004/production --relocation local_data/evaluation/review_fixes_20261004/relocation.json --output local_data/evaluation/relocated_audit_NEW.json`。逐项绑定原报告的哈希、根目录迁移关系和现存产物，拒绝重名、越界或哈希变更。这个通过仅代表原审计产物内容身份，不表示几何通过或新的冻结采集；不改写旧报告。

```bash
python3 -m ssb_tools.evaluate_global_geometry \
  --session sessions/review_fixes_mission_20m_20261004/sessions/20261004_164412_ba34f8df \
  --unroll sessions/review_fixes_mission_20m_20261004/unroll \
  --trajectory sessions/review_fixes_mission_20m_20261004/production/fit \
  --raw sessions/review_fixes_mission_20m_20261004/sessions/20261004_164412_ba34f8df/raw \
  --output local_data/evaluation/review_fixes_20m_recheck_NEW --strict \
  > /tmp/ssb_relocated_20m_recheck.log 2>&1
```

上述迁移的旧 review 数据严格接缝门限仍失败，命令应退出非零；迁移不会改变几何结果，不表示新的冻结或盲验。

## 清理原则与历史缺口

历次删除、去重、归档哈希及操作失误原样保存在 [清理快照](history/DATA_RETENTION_SNAPSHOT_2026-10-03.md)。清理前确认不再被运行或评价依赖；优先少量压缩归档和顺序大块 I/O，正式原图不因小型 D1 已生成就删除。

硬链接按目录相加会重复计空间；完成独立渲染核验后才可去重，链接副本重新比对不能当作新独立验证。两份旧报告（ring_high_seed20261005、d3_noise_20m_support 的 stage_b_smoke.json）曾被覆盖，原字节身份没有恢复；现行验收器拒绝覆盖，复查使用只读模式。

部分旧全图大数组和三处历史反例的原图/大表已删除，报告与预览不能称可运行的完整会话。请按表和快照判断保留范围，不按旧文中的路径推断文件仍存在。


2026-10-04 文档整理只清理本轮已完成测试的 4 个临时夹具目录和初次展示导出草稿；最终公开图像复核、干净提交导出、原始采集与独立重成像全部保留。测试/XML/构建及媒体核对记录先集中打包、校验后清理，清单见 `local_data/evaluation/readme_restructure_20261004/summary.json`，没有修改历史验收报告。

2026-10-04 轮径试验清理：验证集中归档的 387 个文件哈希后，删除本轮三个开发目录及 12 个测试临时夹具目录，逻辑占用约 5.66 GB。开发原图和大时序表已删除，归档不再是完整可运行会话；保留首次失败报告、修正复核报告与物理诊断。最终三档全部原图、独立重成像和重建产物仍保留，清单和归档哈希见 `local_data/evaluation/wheel_error_implementation_20261004/cleanup.json`。

本轮删除自身候选计算、未评分的重复首批采集、生成中间目录及 12 个测试夹具目录。逻辑文件量约 17.16 GB，其中原图硬链接仍由正式输入持有；实际释放独占分配块约 5.91 GB，归档占约 0.43 GB，净回收约 5.5 GB。最终正式留出及既有反例原图、独立重成像完整保留。
