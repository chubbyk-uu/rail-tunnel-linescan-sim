# 历史文档

这里存放已完成或已被取代的过程记录，分批归档，最近一次为 2026-10-05；内容保持归档时的证据和状态，只调整相对链接并加归档说明。
现行设计见 [DESIGN.md](../../DESIGN.md)，阶段 B 现状与操作见 [STAGE_B.md](../STAGE_B.md)。

| 文件 | 内容 |
|---|---|
| [DESIGN_v0.8.md](DESIGN_v0.8.md) | 整理前的完整设计文档快照，含 B2 板缝、接触动力学、工作灯、镜头畸变各轮的过程说明 |
| [STAGE_B_DEVLOG.md](STAGE_B_DEVLOG.md) | 阶段 B 按时间顺序的开发记录：B1 Wall 04 样块、2K 背景、早期 GUI、B2 板缝与裂缝、性能优化、接触动力学、工作灯、眩光、畸变标定 |
| [STAGE_B_QUALITY_PLAN.md](STAGE_B_QUALITY_PLAN.md) | 2026-09-29 外观复核后的六项修正计划（B1–B5）及其完成情况 |
| [STAGE_B_TEXTURE_AUDIT.md](STAGE_B_TEXTURE_AUDIT.md) | 背景纹理清晰度调查与选材决定：Wall 04、Concrete030、concrete_wall_009 淘汰原因，Concrete034 接入、重复度控制、裂缝索引优化 |
| [REVIEW_FIXES_2026-10-01.md](REVIEW_FIXES_2026-10-01.md) | 2026-10-01 代码审核的修复记录与验证结果 |

## 2026-10-04 新归档

| 文件 | 内容 |
|---|---|
| [阶段 D 开发记录](STAGE_D_DEVLOG_2026-10-03.md) | D1–D3 实验、历次模型和门限、失败及性能/证据保护复核的完整快照 |
| [阶段 B 快照](STAGE_B_SNAPSHOT_2026-10-03.md) | 旧画质复核、生成链、任务测试与轨道/车轮参数扫描 |
| [数据清理快照](DATA_RETENTION_SNAPSHOT_2026-10-03.md) | 历次清理、去重、报告覆盖失误与保留范围 |
| [已完成的修复计划](REPAIR_PLAN_2026-10-02.md) | 第 0–7 项实现、验证及后续审核修复记录 |

现行重建见 [STAGE_D](../STAGE_D.md)，验收见 [EVALUATION](../EVALUATION.md)，待办见 [ROADMAP](../ROADMAP.md)。历史文中的“待完成”和“最新”只针对归档时，不能替代现行文档。

## 数据清理说明

2026-10-01 清理了 `local_data/` 和 `sessions/` 中不再使用的数据（约 152 GB），清单见
`local_data/data_inventory_2026-10-01.tsv`。2026-10-01 归档组引用的会话和资产目录大多已删除，
文中的数字是当时实测结果的记录，原始数据不再保留。需要复核时，按现行代码和资产重新采集；
重新采集不能保证还原已删除旧会话的原始数据。逐字节一致的确定性重放仅适用于
位姿流、资产、成像配置与运行环境均保持相同的情况。

当前原图和成果保留范围以 [DATA_RETENTION](../DATA_RETENTION.md) 为准，不在历史索引重复维护会话列表。

## 2026-10-05 里程碑文档整理

以下快照保存整理前原文，只调整链接并加历史标记；不重写失败、旧门限和首次数据身份。当前结果统一在 [MILESTONE_20M](../MILESTONE_20M.md)。

| 快照 | 用途 |
|---|---|
| [DESIGN](DESIGN_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [DATA_RETENTION](DATA_RETENTION_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [EVALUATION](EVALUATION_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [D3_PERFORMANCE](D3_PERFORMANCE_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [STAGE_B](STAGE_B_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [STAGE_C](STAGE_C_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [STAGE_D](STAGE_D_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [WHEEL_ERROR](WHEEL_ERROR_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [MOUNT_ERROR](MOUNT_ERROR_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [RELATIVE_SCALE](RELATIVE_SCALE_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [SENSOR_NOISE](SENSOR_NOISE_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [COMBINED_ERROR](COMBINED_ERROR_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [ROADMAP](ROADMAP_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [DEPLOYMENT](DEPLOYMENT_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |
| [ASSETS](ASSETS_SNAPSHOT_2026-10-05.md) | 整理前设计、操作或历次证据记录；不是现行入口 |

媒体历史补充：[MEDIA 快照](MEDIA_SNAPSHOT_2026-10-05.md)，含旧版对比来源和导出说明。
