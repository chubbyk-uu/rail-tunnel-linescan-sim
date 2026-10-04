# README 界面媒体

更新：2026-10-03。这里保存真实运行界面的两张静态图、两段 GIF，以及实际采集图像的重建对比，不包含生成图冒充采集数据。

2026-10-03：采集响应增益从 3.2 改为 2.4，下面的 GUI 动图/静态图仍对应旧配置，仅展示界面和机构运动；2026-10-04 重建对比使用增益 2.4。用户已于 2026-10-03 确认增益 2.4 为新视觉基线；媒体未重新录制，不能作为新基线的成像画质证据。

## 来源与录制

- 运行代码基于 `f1e9eab`，附加本次 README/部署文档工作树；录制前重新构建，未修改仿真或重建算法。
- 使用现行 `contact_demo` 资产包，`Vehicle travel` 模式，起点 3 m、行程 3 m，名义速度 0.2 m/s、螺距 0.6 m/圈。
- Gazebo 与 RViz 同时运行；为避免窗口遮挡，分别在两个真实任务中录制对应窗口，不能把两段动画当作同一时刻的对照。
- Gazebo 视角跟随车体位置，保持面向机构中部的视线，完整包含扫描头、支架、车体和轨道。RViz 使用 `sim_truth/base` 跟随取景；该姿态只用于显示。
- 通过 Windows FFmpeg 的 `gdigrab` 直接录制桌面上对应窗口区域为 H.264 MP4，每段 12 s、15 fps。WSLg 的窗口 DC 录制会得到黑画面，因此使用桌面区域录制，录制时将目标窗口置前并移除菜单遮挡。
- 最终 GIF 从视频第 1–9 秒转换，6 fps、8 s，不加速播放；Gazebo 动图宽 900 px，RViz 动图宽 1400 px。静态图由视频第 5 秒导出并保留界面原尺寸。没有使用连续截图拼成动图，没有改变原始采集图像。
- 裁切仅去掉窗口外桌面、装饰边框及 Gazebo 右侧通用插件说明区，保留实际视口与控制界面；没有锐化、补光或合成替换界面内容。

录制任务原始行图已在 2026-10-02 清理中删除，完成状态、元数据清单和验证报告保存于本地清理归档（见 [数据保留与清理](../DATA_RETENTION.md)）；不能直接再读取这些历史任务原图。源视频和录制状态保留于 `local_data/readme_media_20261002/`（不进 Git）。媒体哈希与视频信息见 [manifest.json](manifest.json)。录制帧不是原始线阵曝光行，GIF 的颜色和显示分辨率也不能用来验收采集画质或实时率。

## 从源视频导出

在仓库根目录执行（FFmpeg 是文档媒体工具，不是仿真必需依赖）：

```bash
ffmpeg -y -ss 5 -i local_data/readme_media_20261002/gazebo.mp4 \
  -vf 'crop=980:900:38:59' -frames:v 1 docs/media/gazebo.png
ffmpeg -y -ss 5 -i local_data/readme_media_20261002/rviz.mp4 \
  -vf 'crop=1540:930:38:59' -frames:v 1 docs/media/rviz.png

ffmpeg -y -ss 1 -t 8 -i local_data/readme_media_20261002/gazebo.mp4 \
  -filter_complex '[0:v]crop=980:900:38:59,scale=900:-1:flags=lanczos,fps=6,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle' \
  -loop 0 docs/media/gazebo_scan.gif
ffmpeg -y -ss 1 -t 8 -i local_data/readme_media_20261002/rviz.mp4 \
  -filter_complex '[0:v]crop=1540:930:38:59,scale=1400:-1:flags=lanczos,fps=6,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle' \
  -loop 0 docs/media/rviz_capture.gif
```

源视频不随 Git 提供。若更新机器人、界面或光照，需重新运行并录制视频，不能仅改文字后继续把这些媒体称为当前版本。

## 历史重建对比图片（2026-10-03）

更新保留的 20 m 带假设噪声档、2 mm 轨道起伏数据的板缝/裂缝对比与单圈原始条带。采集/重建仍为 `c77d762`，未重新采集；未补偿螺旋的复核页与导出器为 `284e265`，源数据 `sessions/d3_noise_20m_v2_20261003/feature_review_rawleft/`。原图、旧评价、预览和溯源保留；完整全图大数组已清理，可按溯源重建。

`joint_comparison_20m.png` 与 `crack_comparison_20m.png` 分别选图像检测器给出的首个宽/细结构，不使用缺陷真值或真实误差选图。每侧 512×512，两个面板逐字节等于源 PNG 的 RGB 像素，只在面板之外加标题。左侧原始 Mono8 每圈仅以局部中央角度的一次编码器位置固定摆放，没有螺旋、畸变和平场补偿；右侧经过采后光学校正、展开与全局优化。两侧固定 DN 0–255，无融合、锐化或调色。此为完整处理链对比，D3 定量精度仍比较名义展开和优化。`raw_band_20m.png` 是源 `raw_band.png` 的原样复制：原始 59,259×4096 曝光数据只转置并每隔 33 像素抽样，未作光学校正和圈内位移补偿。

完整输出覆盖和保存值核验通过，但末圈 15 个预定双圈评价点缺测，整体验收仍失败；展示图不能豁免该失败，也不代表所有位置都达到 1 px。两组局部图由本机可选假设噪声档生成，不替换默认关闭噪声的演示。来源、候选位置和哈希见 `manifest.json` 的 `reconstruction_figures_history`。

重新生成自己的复核页后，使用已构建的 Python 包导出（输出目录须为新目录）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/export_readme_comparisons.py \
  --features sessions/d3_features_NEW \
  --output local_data/readme_export_NEW
```

输出 `joint_comparison.png`、`crack_comparison.png`、`raw_band.png`、报告与自动溯源；导出前核验源文件身份，导出后核验所有面板像素。当时导出文件的后缀 `20m` 表示数据范围；当前 README 使用下节组合图，导出器也可用于其他范围。


## 重建对比图片（2026-10-04）

当前 [组合图](reconstruction_comparison.png) 为 1048×1092，来自 `7abfc5b` 冻结后新种子 20261029 的 `sessions/boundary_crop81_holdout_20261004`：真实轮径 81 mm、标定 80 mm，带起伏，噪声关闭，增益 2.4；阶段 B 25 项、协议 v7 16 项与独立接缝／四边支撑均通过。窗口内／间 P95 为 0.475／0.645 px，不表示每点都 ≤1 px。

- 总览展示 8–11 m 的顶部约 ±30°，源预览步距 32 像素；目标与评价仍为完整 240°。
- 局部只依据公开图像，使用中心位于目标内的首个宽／细候选（编号 0／2）；没有读缺陷标签或按真值分数挑选。源局部 512×512，组合图截取结构附近的 512×192，保持 1:1 像素。
- 左侧每圈固定摆放原始 Mono8，不补偿圈内前进、畸变或平场。右侧包含采后校正、展开、匹配、全局优化及共享径向深度。固定 DN 0–255，无融合、锐化和自动对比度。去螺旋主要来自展开，D3 定量基线仍为名义展开。
- 保存后的六个面板矩形与哈希核验过的源图逐像素一致，来源、裁切和哈希见 manifest.json；整幅数组不入 Git。

| 形态 | 原始条带 | 采后重建 |
|---|---|---|
| 板缝候选 | [原图](reconstruction_joint_raw.png) | [重建](reconstruction_joint_optimized.png) |
| 裂缝候选 | [原图](reconstruction_crack_raw.png) | [重建](reconstruction_crack_optimized.png) |

重新导出时使用新目录，工具读取已安装包：

```bash
python3 -m ssb_tools.feature_review \
  --unroll sessions/boundary_crop81_holdout_20261004/unroll \
  --trajectory sessions/boundary_crop81_holdout_20261004/fit \
  --observable sessions/boundary_crop81_holdout_20261004/capture/config/observable_config.json \
  --output sessions/FEATURES_NEW > /tmp/ssb_features_NEW.log 2>&1
python3 tools/export_readme_summary.py --features sessions/FEATURES_NEW \
  --output local_data/EXPORT_NEW
```

当前导出目录为 `local_data/boundary_readme_export_20261004`。历史 20261005 种子、带噪声对比与对应哈希保留于 Git 历史及 manifest 的 reconstruction_figures_history，不将旧图的噪声档位或精度沿用到新图。
