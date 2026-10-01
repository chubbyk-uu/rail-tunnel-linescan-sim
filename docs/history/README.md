# 历史文档

这里存放已完成或已被取代的过程记录，2026-10-01 归档，内容保持归档时原样，只修正了相对链接。
现行设计见 [DESIGN.md](../../DESIGN.md)，阶段 B 现状与操作见 [STAGE_B.md](../STAGE_B.md)。

| 文件 | 内容 |
|---|---|
| [DESIGN_v0.8.md](DESIGN_v0.8.md) | 整理前的完整设计文档快照，含 B2 板缝、接触动力学、工作灯、镜头畸变各轮的过程说明 |
| [STAGE_B_DEVLOG.md](STAGE_B_DEVLOG.md) | 阶段 B 按时间顺序的开发记录：B1 Wall 04 样块、2K 背景、早期 GUI、B2 板缝与裂缝、性能优化、接触动力学、工作灯、眩光、畸变标定 |
| [STAGE_B_QUALITY_PLAN.md](STAGE_B_QUALITY_PLAN.md) | 2026-09-29 外观复核后的六项修正计划（B1–B5）及其完成情况 |
| [STAGE_B_TEXTURE_AUDIT.md](STAGE_B_TEXTURE_AUDIT.md) | 背景纹理清晰度调查与选材决定：Wall 04、Concrete030、concrete_wall_009 淘汰原因，Concrete034 接入、重复度控制、裂缝索引优化 |
| [REVIEW_FIXES_2026-10-01.md](REVIEW_FIXES_2026-10-01.md) | 2026-10-01 代码审核的修复记录与验证结果 |

## 数据清理说明

2026-10-01 清理了 `local_data/` 和 `sessions/` 中不再使用的数据（约 152 GB），清单见
`local_data/data_inventory_2026-10-01.tsv`。以上文档引用的会话和资产目录大多已删除，
文中的数字是当时实测结果的记录，原始数据不再保留。需要复核时，按现行代码和资产重新采集；
重新采集不能保证还原已删除旧会话的原始数据。逐字节一致的确定性重放仅适用于
位姿流、资产、成像配置与运行环境均保持相同的情况。

仍保留的数据：`sessions/gz_a`（阶段 A 基线）、`sessions/review_portable_gui_final` 与
`sessions/review_portable_replay`（阶段 B 最终验收），以及现行演示依赖的资产（见 STAGE_B.md）。
