# 界面与重建媒体来源

更新 2026-10-05。当前比较图来自最新 20 m 综合采集；GUI 动图/静图为 2026-10-02 的历史界面视频，旧增益 3.2，不能作为增益 2.4 的新画质/性能证据。所有媒体为真实运行或采集输出，不含生成图冒充。哈希与之前来源记录见 [manifest.json](manifest.json)，旧各版说明见 [快照](../history/MEDIA_SNAPSHOT_2026-10-05.md)。

## 20米里程碑对比（2026-10-05）

当前 `reconstruction_comparison.png` 来自 `03cde91` 完整构建后、干净源码上的种子20270119综合20米新采；正式独立接缝为0.515／0.587 px。公开展示工具收尾版本 `6a1265b`，不重新拟合或读取评价真值。

显示8–11米顶部约60°的概览（stride32），不是另一组3米盲验；局部为公开图像检测器给出的首个目标内宽/细结构，候选0/2。512×192局部保持1:1，完整512×512版本见下表。左侧原始Mono8每圈只用一次编码器位置固定摆放，无螺旋/畸变/平场补偿；右侧为采后校正、展开、公开优化与共享深度，无融合、锐化或自动对比度。

局部右图按拟合轨迹定位与左图相同的原始观测，裁切框平移分别约(−19.49,+22.75)毫米、(−5.94,+20.18)毫米；只改变展示原点，不修改像素、旋转、缩放或用真值对齐。中心位置不作为几何精度证据。原图与优化概览共用抽样步距；导出后的六个矩形与源PNG逐像素相同。旧20米概览步距不一致及局部未跟随坐标修正的失败展示记录已归档，不发布为当前图。

源页 `sessions/milestone20_review3_aligned_20261005/`，导出 `local_data/readme_export_milestone20_aligned_20261005/`；manifest记录采集与展示提交、范围及图片哈希。GIF仍是历史录制，未冒充本轮新画质/性能证据。

| 原尺度局部 | 原始条带 | 优化图 |
|---|---|---|
| 板缝 | [原图](reconstruction_joint_raw.png) | [优化](reconstruction_joint_optimized.png) |
| 裂缝 | [原图](reconstruction_crack_raw.png) | [优化](reconstruction_crack_optimized.png) |

## 来源与录制

- 运行代码基于 `f1e9eab`，附加当时的 README/部署文档工作树；录制前重新构建，未修改仿真或重建算法。
- 使用当时的 `contact_demo` 资产包，`Vehicle travel` 模式，起点 3 m、行程 3 m，名义速度 0.2 m/s、螺距 0.6 m/圈。
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
