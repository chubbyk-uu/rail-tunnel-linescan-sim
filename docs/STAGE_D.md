# 阶段 D：重建操作

D1/D2/D3、共享深度与完整 20 m 优化图已完成。融合可选、默认关闭，真值禁止进入生产。当前结果只在 [里程碑](MILESTONE_20M.md) 维护，历次失败/参数与旧命令见 [快照](history/STAGE_D_SNAPSHOT_2026-10-05.md)。

下文 0.2 m 命令为名义流程；综合误差必须使用 [专用参数](ERROR_SCENARIOS.md)。

## 输入与几何

只读取完成信息中的公开计数/哈希、`config/observable_config.json`、曝光/门控和编码器表、原始 Mono8 块及图像估计的标定文件。不读取 `evaluation/`、仿真位姿、实际轮径、安装真值或场景/缺陷资产。路径检查同时限制目录和文件符号链接，表和实际用到的原图块均校验哈希。

初始扫描轴位置与角度复用公开双测量轮和扫描编码器插值。标靶估计的逆镜头映射给出每个原始列在标靶距离上的轴向坐标，按名义半径/标靶距离缩放至壁面。模型假设扫描轴居中、线方向沿隧道轴向；未知的真实姿态与安装误差不被当作已知量。

暗场和平场在原始列上进行，浮点 DN 不裁剪，原始饱和与坏列保持无效。D1 v2（2026-10-03）不再落盘原始列浮点缓存：采样时直接从会话的 uint8 原图块读取相关行，按 `float32 (raw − offset) × gain` 校正（饱和 255 或无效列为 NaN），与原缓存逐位相同；原图块首次访问时按 D1 记录的哈希校验。畸变校正以几何映射保存，与展开采样合并，不先生成拉直图再作第二次空间插值。相邻有效曝光行之间做角向插值，遇到缺行或不连续的触发格点不跨越；有限像素足迹外无效，未标定边缘不外推。

每圈条带可用 `load_bands()` / `BandSampler.sample()` 独立按世界坐标窗口读取，供 D2 比较重叠区域。D1 初始全图在有效条带中选择原始列更靠近传感器中心的一圈，不做跨圈融合，也不拟合车体运动。

## 运行流程

先按 [部署文档](DEPLOYMENT.md) 构建并加载已安装包。输入必须是完成的 `Wall coverage` 会话；`Vehicle travel` 未预先声明壁面目标，当前不能直接用于正式 D1，也不能事后篡改归档配置补目标。下例中的 SESSION 和标定文件须属于同一光学条件，所有输出目录必须是新的。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m ssb_tools.initial_unroll --session SESSION \
  --calibration local_data/stage_b/contact_demo_buffered/calibration.json \
  --backend cuda --output sessions/d1_NEW > /tmp/ssb_d1.log 2>&1
python3 -m ssb_tools.match_bands --unroll sessions/d1_NEW \
  --spacing-m 0.2 --output sessions/d2_NEW > /tmp/ssb_d2.log 2>&1
python3 -m ssb_tools.optimize_bands \
  --unroll sessions/d1_NEW --matches sessions/d2_NEW \
  --observable SESSION/config/observable_config.json \
  --attitude-spacing-m 0.02 --observed-knots \
  --output sessions/d3_NEW > /tmp/ssb_d3.log 2>&1
```

D1 默认 CUDA；`--backend cpu` 是显式选择，CUDA 失败不静默回退。可用 `--target-x A B` 在原定目标内诊断子区间，报告保留原目标；子区间不能冒充完整任务验收。下例名义参数为匹配间距 0.2 m、姿态节点 0.02 m、只在有曝光支撑的区间设置细节点；不依赖工具旧的 0.4/0.05 m 默认值。

## D1 输出与资源

`--mosaic none|roi|full` 默认 `none`，`roi` 另需 `--mosaic-x A B`。初始全图只供查看，D2/D3 不读它，最终全图从原图重新采样。

| 文件 | 内容 |
|---|---|
| `projection.npy` | 原始曝光序号、触发格点、圈号、编码器估计的轴向位置及扫描角 |
| `native_source.json` | 本区域用到的原图块（文件、首行序号、行数、SHA-256）、宽度及来源会话；D2 据此读原图 |
| `mapping.npz`、`calibration.json` | 测量镜头映射、有效标定域、名义角向足迹、逐列暗场/平场参数及标定 |
| `bands.json` | 每圈对应的行范围、实际记录的曝光边界、目标网格和坐标约定 |
| `coverage_runs.bin` | 全分辨率覆盖的精确游程（`q_bin, x_begin, x_end, count`，int32），零为无效、大于一为重叠；dtype 记在报告中 |
| `preview.png` | 每隔 stride 个网格中心的精确全图值，不需要先生成全图 |
| `overlap.png` | 每个预览格内的最小覆盖数：格内任一像素无效即为红色，不会被稀疏取样漏掉 |
| `overlap_*.png`、`review.html` | 最多三对相邻圈的原尺度复核窗口，固定 0–255 显示，无锐化和自动对比度 |
| `mosaic_u16.npy`、`mosaic_count.npy` | 仅 `--mosaic roi/full`：DN×64 四舍五入的 uint16（65535 为无效，量化误差 ≤1/128 DN）及独立条带数（uint8，255 饱和） |
| `report.json`、`provenance.json` | 缺口/饱和统计、覆盖直方图、全图选项、资源与限制、代码和输入/输出哈希 |

每个输出像素的采样来源由网格、曝光投影表、镜头映射和原图块共同确定，不保存坐标图或来源条带图。`trace_pixel(output, q_index, x_index)` 按与展开相同的条带顺序、评分和 float32 比较重算来源条带，恢复原始块、曝光序号、列号和权重；有全图时同时核对存储值。最终优化后从原始列重采样。

默认输出约 25 MB / 3 m、137 MB / 20 m；3 m 单幅全分辨率图另需约 2.4 GiB。50 m 默认展开预计 0.35–0.4 GB，单幅全图约 43 GB，仅是预算，尚未完成 50 m 端到端验收。原始采集、重成像和多份成果另计。

计算按默认 32×8192 像素块进行，原图 LRU 最多 128 块并受文件句柄限额约束。对外返回像素拷贝，不暴露可能因 mmap 淘汰失效的视图。批量顺序读写，阶段边界做哈希；不保存全部原始列浮点缓存，不生成逐窗口小文件。

## D2 匹配与环缝处理

详细实现见 [RECONSTRUCTION_METHODS](RECONSTRUCTION_METHODS.md)。

## D3 连续轨迹优化

详细实现见 [RECONSTRUCTION_METHODS](RECONSTRUCTION_METHODS.md)。

## 看图与完整输出

```bash
python3 -m ssb_tools.feature_review \
  --unroll sessions/d1_NEW --trajectory sessions/d3_NEW \
  --observable SESSION/config/observable_config.json \
  --output sessions/features_NEW > /tmp/ssb_features.log 2>&1
python3 -m http.server 8765 --bind 0.0.0.0 --directory sessions/features_NEW
```

展示完整拟合中的子区间可加 `feature_review --target-x 8 11`；只裁切显示，不改D1、轨迹或验收域。原图和优化概览共用1800像素长边/1 MP预算步距，局部从公开轨迹定位同一观测，两侧裁切原点分别记录；这不是额外的真值对齐或图像变形。

打开 `http://localhost:8765/review.html`。WSL localhost 转发不可用时，用 `hostname -I` 的地址替换 localhost。页面左侧为未经螺旋、畸变和平场补偿的原始条带，右侧是采后处理结果；名义展开是中间状态和 D3 定量基线。位置只从公开图像选，最大误差窗口另行保留。不能把去螺旋本身的效果全部归功于优化，也不把实际斜向裂缝强行拉直。

```bash
python3 -m ssb_tools.global_mosaic \
  --unroll sessions/d1_NEW --trajectory sessions/d3_NEW \
  --output sessions/full_NEW > /tmp/ssb_full.log 2>&1
python3 tools/validate_global_mosaic.py \
  --unroll sessions/d1_NEW --trajectory sessions/d3_NEW \
  --mosaic sessions/full_NEW --output local_data/evaluation/full_NEW \
  > /tmp/ssb_full_validation.log 2>&1
```

默认同时输出名义和优化全图；`--optimized-only` 减半。值为 uint16 DN×64（65535 无效），另存 uint8 覆盖计数及完整游程。量化误差 ≤1/128 DN；固定显示 DN 0–255，不融合、补洞或锐化。全图保存值核验、完整覆盖、接缝几何与四边支撑是不同检查，不能相互替代。

## 首尾双圈的公开支撑证明

详细实现见 [RECONSTRUCTION_METHODS](RECONSTRUCTION_METHODS.md)。

## 接缝融合

融合可选，默认关闭；正式默认输出为优化后未融合图（2026-10-05 用户确认）。

已有完整未融合图和几何验收后，使用 `python3 -m ssb_tools.seam_fusion` 生成单独产品；每圈受限增益只在训练图像有一致强差异时启用，默认约 2 mm 窄带混合，CUDA 分块。原图、未融合图、覆盖与几何报告保留；完整计数/游程相等及独立 CPU 窄带权重检查通过。操作、范围和实测见 [SEAM_FUSION](SEAM_FUSION.md)。
