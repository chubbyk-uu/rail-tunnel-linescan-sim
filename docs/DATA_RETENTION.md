# 数据保留与安全清理

更新 2026-10-05。本页只列当前正式里程碑及清理规则；完整历史目录账本、已删数组、迁移和旧报告覆盖失误在 [整理前快照](history/DATA_RETENTION_SNAPSHOT_2026-10-05.md) 与 [早期清理记录](history/DATA_RETENTION_SNAPSHOT_2026-10-03.md)。历史路径不表示文件现仍存在。本次文档整理不删除采集或资产。

## 1. 必须保留的20米证据

| 内容 | 本机位置与依赖 |
|---|---|
| 联合 GUI 原始采集 | `sessions/milestone20_gui_seed20270119_20261005/`，实际会话在其 `sessions/20261005_165603_2cd2422f/` |
| 实际独立重成像 | 同一 GUI 根目录的 `wall_replay/`，不是新建硬链接冒充重渲染 |
| 公开 D1/D2/D3 和完整优化图 | `sessions/milestone20_holdout_seed20270119_20261005/`；capture/reimage 引用上述实际目录，必须保留目标 |
| 冻结协议、真实网格和专项评价 | 同名 `local_data/evaluation/`，含首次记录、`defects_verified/`、`rings_verified/` 与 `evidence_summary.json` |
| 综合生成包 | `local_data/stage_b/milestone20_seed20270119_bundle/`；仅生成端，含真值/密钥 |
| 公开对比页与媒体导出 | `sessions/milestone20_review3_aligned_20261005/`、`local_data/readme_export_milestone20_aligned_20261005/` |
| 名义默认运行包 | `local_data/stage_b/contact_demo_buffered/`；与综合验收包不同 |
| 官网原图、配方和视频来源 | `local_data/stage_b/sources/`、Git 素材清单、`local_data/readme_media_20261002/` |

历史正式会话和反例是否完整以快照及各次 `cleanup.json` 为准；开发归档一般只有报告、配置、日志和小型诊断，不承诺可以原会话重放。D1 不保存浮点原始列缓存，仍依赖 `native_source.json` 中的原图；已有展开结果不是删原图的理由。

## 2. 最近一次清理的边界

20 m 收尾记录已在 `local_data/evaluation/milestone20_holdout_seed20270119_20261005/retained_records/development_preparation_tests_v2.tar.gz` 集中归档，8422 条记录回读哈希核对；清单为 `cleanup.json` / `archive_hashes.json`。删除仅限本轮准备目录、重复/失败展示、开发数组和已完成测试临时目录。

逻辑文件量约 12.63 GiB，按唯一 inode 估算实际约 2.43 GiB，不能按硬链接目录重复相加宣称释放更多空间。正式原图、独立重成像、最终全图及综合包完整保留；首次失败和首次评价不覆盖。

## 3. 清理规则

1. 查运行与评价依赖、进程占用、符号链接目标和硬链接，再判断是否可删除。不能仅依据目录命名像中间产物就清理。
2. 正式原图/元数据、首次失败、协议与哈希、已发布媒体来源优先保留；派生大数组仅在可再生产路径明确时删除。
3. 删除前集中归档小记录并回读校验，记录范围、不能复现的部分及实际占用。归档缺失不可用新报告伪造。
4. 独立重成像先实际渲染、校验，再决定是否去重；硬链接的再次比较不能作为新的独立证据。共享资产不可原位写入。
5. WSL 工作数据放 Linux 文件系统，优先顺序大块 I/O 与少量压缩归档；不按行/窗口生成海量文件，不在 GUI 定时递归哈希。

目前没有自动按时间或容量删除正式任务的策略；`sessions/mission/` 与 `local_data/mission_runs/` 会累积，按需盘点后手动清理。每个任务所需空间随长度、门控/超扫和输出种类变化，不能把旧 3 m 或 20 m 数字当作硬性上限。

## 4. 搬迁与复查

保留文件哈希与明确迁移映射，使用新的 `--session`、`--unroll`、`--trajectory` 和 `--raw`；历史报告不改路径或首次结果。支持 `holdout_protocol audit-public --relocation` 绑定迁移后的相同产物；通过只证明身份，不代表新的采集或几何验收。

旧反例因清理已不完整时，必须明确标为历史记录，不能在 README 用不存在的会话作快速运行前提。首次部署按 [ASSETS](ASSETS.md) 从网站重新生成。
