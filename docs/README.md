# 项目文档导航

更新 2026-10-05。当前状态只在 [20米里程碑](MILESTONE_20M.md) 维护；规范、操作、证据和历史分开。20 m 综合场景已收尾，50 m 尚未建模/验收。

## 首次部署与运行

| 文档 | 用途 |
|---|---|
| [公共部署](DEPLOYMENT.md) | 依赖、源码、构建与 OptiX 自检 |
| [WSL](deployment/WSL.md) / [原生 Linux](deployment/LINUX.md) | 分开的驱动/图形环境与启动步骤 |
| [素材与生成](ASSETS.md) | 官网下载、规格、从零生成成套资产 |
| [快速运行](QUICKSTART.md) | RViz 任务、无界面采集、重建与浏览 |
| [误差场景](ERROR_SCENARIOS.md) | 默认/综合区别、轮径/装配/起伏/噪声和冻结命令 |

## 设计与阶段操作

| 文档 | 用途 |
|---|---|
| [总设计](../DESIGN.md) | 交付范围、真值边界与原章节目录 |
| [参数/几何](design/GEOMETRY.md) / [机构与采集](design/SIMULATION.md) | 参数来源、触发/行频、接触、照明、保存链路 |
| [重建设计](design/RECONSTRUCTION.md) | 连续轨迹、尺度/深度、可观测性与输出 |
| [阶段 B](STAGE_B.md) / [阶段 C](STAGE_C.md) | 场景/标定和原始壁面任务 |
| [阶段 D 操作](STAGE_D.md) / [算法实现](RECONSTRUCTION_METHODS.md) | D1/D2/D3、反采样与支撑 |
| [轮径](WHEEL_ERROR.md) / [固定安装](MOUNT_ERROR.md) / [相对尺度](RELATIVE_SCALE.md) | 细分误差方法 |
| [噪声](SENSOR_NOISE.md) / [可选融合](SEAM_FUSION.md) / [性能](D3_PERFORMANCE.md) | 模型假设、默认策略与速度边界 |

## 验收、保留与后续

| 文档 | 用途 |
|---|---|
| [里程碑](MILESTONE_20M.md) / [专项诊断](MILESTONE_20M_DIAGNOSTICS.md) | 当前完整 20 m 结果与环缝/裂缝有限采样 |
| [验收协议](EVALUATION.md) / [开发守则](DEVELOPMENT_RULES.md) | 冻结、独立数据、真实网格评价、公开输入审计 |
| [媒体](media/README.md) / [数据保留](DATA_RETENTION.md) | 视频/对比图来源与清理边界 |
| [后续计划](ROADMAP.md) | 50 m 容量预算、重叠分段与未实现模型 |
| [参考资料](design/REFERENCES.md) / [历史归档](history/README.md) | 原来源与被取代的状态/失败记录 |

历史的“当前/待完成”只代表记录当时；不从归档文档复制任务命令或宣称旧路径仍可用。数值变更先记录新证据，不覆盖旧失败。文档更新不意味着重新采集或新盲验。
