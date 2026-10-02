# README 界面媒体

更新：2026-10-02。这里保存真实运行界面的两张静态图和两段 GIF，不包含示意图或生成图片。

## 来源与录制

- 运行代码基于 `f1e9eab`，附加本次 README/部署文档工作树；录制前重新构建，未修改仿真或重建算法。
- 使用现行 `contact_demo` 资产包，`Vehicle travel` 模式，起点 3 m、行程 3 m，名义速度 0.2 m/s、螺距 0.6 m/圈。
- Gazebo 与 RViz 同时运行；为避免窗口遮挡，分别在两个真实任务中录制对应窗口，不能把两段动画当作同一时刻的对照。
- Gazebo 视角跟随车体位置，保持面向机构中部的视线，完整包含扫描头、支架、车体和轨道。RViz 使用 `sim_truth/base` 跟随取景；该姿态只用于显示。
- 通过 Windows FFmpeg 的 `gdigrab` 直接录制桌面上对应窗口区域为 H.264 MP4，每段 12 s、15 fps。WSLg 的窗口 DC 录制会得到黑画面，因此使用桌面区域录制，录制时将目标窗口置前并移除菜单遮挡。
- 最终 GIF 从视频第 1–9 秒转换，6 fps、8 s，不加速播放；Gazebo 动图宽 900 px，RViz 动图宽 1400 px。静态图由视频第 5 秒导出并保留界面原尺寸。没有使用连续截图拼成动图，没有改变原始采集图像。
- 裁切仅去掉窗口外桌面、装饰边框及 Gazebo 右侧通用插件说明区，保留实际视口与控制界面；没有锐化、补光或合成替换界面内容。

源视频和录制状态保留于 `local_data/readme_media_20261002/`（不进 Git）。媒体哈希与视频信息见 [manifest.json](manifest.json)。录制帧不是原始线阵曝光行，GIF 的颜色和显示分辨率也不能用来验收采集画质或实时率。

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
