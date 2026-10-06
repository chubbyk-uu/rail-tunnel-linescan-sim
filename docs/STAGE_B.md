# 阶段 B：场景、照明与标定

更新 2026-10-05。20 m 场景、轮轨接触、Concrete034 纹理、板缝/裂缝、共转 COB 条光、镜头畸变及标靶标定均已实现；完整优化图和可选融合也已实现。当前综合结果见 [MILESTONE_20M](MILESTONE_20M.md)。旧资产标签、画质实验和短程数字见 [历史快照](history/STAGE_B_SNAPSHOT_2026-10-05.md)。

## 1. 运行包与资源

默认为 `local_data/stage_b/contact_demo_buffered/`：有效 [0,20] m，构造区 [−2.5,22.5] m，内含 capture/spec、calibration、光学 assets、world、GUI 配置和 bundle 哈希清单，运行引用均在包内。配置含生成真值/私有密钥，只供采集端；不能把整个包交给盲重建。

首次部署按 [ASSETS](ASSETS.md) 从网站下载生成，不依赖 `c034_DC4`、`geometry_b2_v5`、v10/v11 等历史中间目录。正式综合包与默认包的区别见 [ERROR_SCENARIOS](ERROR_SCENARIOS.md)。世界、配置、标定须成套，开始前检查物理装配和光学身份。

| 项目 | 当前基线 |
|---|---|
| 管片 | 环宽 1.2 m，6 块/环，错缝 18°，倒角/圆角及灰砂浆填缝 |
| 背景 | Concrete034 高清 Color/Normal/Roughness，0.1 mm 配方网格，叠加大尺度明暗变化 |
| 裂缝 | 60 个，48 细长/9 细短/3 网状；截断正态体宽 0.2–0.6 mm，实际布局 0.226–0.566 mm |
| 裂缝近似 | cavity_v2 反照率腔内模型，有效深度是合成参数，没有实际裂缝几何凹陷 |
| 相机 | 4096、7.04 µm、90 mm、8 µs；边缘 +0.6% 假设枕形畸变，增益 2.4 |
| 扫描光源 | 20×20 mm COB、80×80 mm 散热器；轴向 −115 mm，名义 1.2×0.12 m 半高宽光斑 |
| 采样 | 3 个曝光时刻，临界几何每时刻 16 条射线，复杂裂缝每时刻 64 条，2×2 纹理足迹积分 |
| 轨道与车轮 | 直轨真实接触，2 mm 共同起伏/左右高差，0.2 mm 静压缩量假设柔性，弹簧压紧双测量轮 |
| 缓存 | OptiX 纹理 GPU 2 GiB/CPU 1 GiB；GUI 与 RViz 使用轻量预览，不加载完整光学纹理 |

具体槽口、衬面闭合和光照参数以包内配置/清单为准，设计见 [机构与成像](design/SIMULATION.md)。裂缝端点当前为 double 格式，旧 float 仅兼容读取；不要复制旧清单数字作为新包网格数量。

## 2. Gazebo 与 RViz

WSL：

```bash
tools/run_gz_gui.sh                       # 默认短程，点击 Play 开始
tools/run_gz_gui.sh sessions/gui_NEW      # 新会话目录
tools/run_mission.sh --gz-gui             # RViz 任务 + Gazebo
```

首次静置 2 s，头部从 θ=180° 朝下开始；转 55° 到 −125° 门控，经过顶部到 +125° 关门，输出仍固定上方 240°。短程 Gazebo 演示的车体行程不等于完整壁面任务；完整目标使用 [阶段 C](STAGE_C.md)。原生路线见 [LINUX](deployment/LINUX.md)。

GUI 环境散光 0.6，无顶部固定灯；4 盏 90° 工作灯与 17 束窄锥扫描预览灯投影阴影。光晕插件默认关闭。GUI 可以照亮/遮挡机器人支架；OptiX 采集使用独立等效 COB、弱补光与内壁阴影，两者不共享照明数值。

## 3. 独立标定与采后校正

暗场、均匀场、条纹和留出靶使用独立居中夹具渲染；标定只从图像拟合列暗偏移/增益和镜头射线映射，不输出真实安装外参。修改安装/响应后重新标定；噪声档需自己的标定。HMAC 身份只证明兼容，不把密钥写入公开配置。

采集不生成 `processed/optical/`。正式 D1 在采后读取原图时合并校正，命令见 [QUICKSTART](QUICKSTART.md#3-采后校正展开与匹配优化)。若使用依赖该目录的历史 `stage_b_review`，先单独运行 `optical_calibration apply`；其旧裂缝编号/位置不能当作任意布局的复核入口。

## 4. 独立重成像与验收

已有完成会话可使用存档私有位姿独立重新渲染，属于生成/验收用途，不是生产重建输入。先 source ROS/install；WSL 运行库内再次 source：

```bash
bash tools/ssb_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  install/ssb_core/lib/ssb_core/ssb_render \
    --config SESSION/evaluation/config_source.yaml --session sessions/REIMAGE_NEW \
    --poses SESSION/evaluation/pose_stream.bin --batch-rows 333
' > /tmp/ssb_reimage.log 2>&1
python3 -m ssb_tools.validate_stage_b SESSION --compare sessions/REIMAGE_NEW
```

首次报告不能覆盖；复查用 `--read-only` 或新输出路径，见 [EVALUATION](EVALUATION.md)。当前 20 m 里程碑实际重成像一致、25 项阶段 B 通过，不用旧短程 22/24 项替代。

## 5. 已知局限

OptiX 尚未加入机器人自身遮挡；裂缝深度是反照率近似，工作灯以弱补光近似，没有多次反射或绝对光度标定。MTF/离焦未实现，噪声是可选仿真假设。主素材面积有限，分层合成降低重复但不能消除；拼接仍需唯一性/退化检查。

轨道只验证直线共同起伏与左右高差，没有弯轨、轨向不平顺、波磨/焊缝及真实轮胎滚动半径模型。WSL 显示驱动的所有偶发黑屏尚不能排除。50 m 和额外模型不属于当前通过结论。
