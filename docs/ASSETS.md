# 素材下载与从零生成演示

首次部署先完成 README 的环境安装、构建和对应平台的 OptiX 自检，再按本页获取公开原图和生成资产。不需要旧机器的 `local_data`，也不需要已有演示包。当前默认生成 20 m；后续扩展最多 50 m，尚未完成 50 m 整场生成及采集验收。

## 1. 公开素材与下载规格

| 目录 | 官方素材页面 | 所需文件 | 物理范围 |
|---|---|---|---|
| `concrete034` | [ambientCG Concrete034](https://ambientcg.com/view?id=Concrete034) | 16K-PNG 包中的 Color、NormalGL、Roughness | 约 1.1×0.55 m；16384×8192，约 0.067 mm/纹素 |
| `painted_plaster_wall` | [Poly Haven Painted Plaster Wall](https://polyhaven.com/a/painted_plaster_wall) | 4K Diffuse PNG | 2×2 m；仅使用 ≥20 mm 明暗层 |
| `grey_plaster` | [Poly Haven Grey Plaster](https://polyhaven.com/a/grey_plaster) | 8K Diffuse PNG | 1×1 m；仅作砂浆填缝细节 |

三者使用 CC0 许可。尺寸与下载规格来自上述官方页面及其 API，固定文件地址、字节数和 SHA-256 记录在 [demo_sources.json](../assets/materials/demo_sources.json)。主背景不是 Wall 04，也不是 Concrete030；不需要下载历史选材实验中的其他纹理。

### 自动下载（推荐）

在仓库根目录、Linux 文件系统执行：

```bash
# 按实际网络配置设置代理；curl 会继承当前终端的环境。
# 例如自己的代理是 http://HOST:PORT 时：
# export https_proxy='http://HOST:PORT'
# export http_proxy="$https_proxy"
python3 tools/download_demo_sources.py --output local_data/stage_b/sources \
  > /tmp/ssb_source_download.log 2>&1

# 不联网、不写资产，只检查全部原图与来源清单。
python3 tools/download_demo_sources.py --output local_data/stage_b/sources --verify-only
```

下载约 652 MB（含 Concrete034 压缩包）；只抽取三张需要的图，保留压缩包以便复查。工具顺序下载、流式解压和哈希，使用临时 `.part` 文件，验证成功才改名；已验证的文件不会重复下载。中断后可重新执行同一命令。已有文件哈希不符时拒绝覆盖，需要先定位并移走损坏文件后重试；不能通过改清单跳过检查。

### 网站手动下载

1. 打开 Concrete034 页面，选择 **16K-PNG .zip**；只解压 `Concrete034_16K-PNG_Color.png`、`Concrete034_16K-PNG_NormalGL.png`、`Concrete034_16K-PNG_Roughness.png` 到 `local_data/stage_b/sources/concrete034/`。不要选择 JPG 或 NormalDX。
2. 打开 Painted Plaster Wall 页面，分辨率选 **4K**，在 **Diffuse** 下下载 **PNG**，存为 `local_data/stage_b/sources/painted_plaster_wall/painted_plaster_wall_diff_4k.png`。
3. 打开 Grey Plaster 页面，分辨率选 **8K**，在 **Diffuse** 下下载 **PNG**，存为 `local_data/stage_b/sources/grey_plaster/grey_plaster_diff_8k.png`。
4. 再运行自动下载命令。它会校验已有原图并写出 `downloads.json`，不会重新下载完整文件。单独放 PNG 而没有清单，不能直接进入生成步骤。

## 2. 在本机生成完整演示

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# WSL：经 tools/ssb_runtime.sh 用隔离 OptiX 运行库渲染标定靶。
python3 tools/build_demo_from_sources.py --runtime wsl \
  --sources local_data/stage_b/sources \
  --work local_data/stage_b/build_from_sources_NEW \
  --output local_data/stage_b/contact_demo_buffered \
  > /tmp/ssb_asset_generation.log 2>&1
```

原生 Linux 将 `--runtime wsl` 改为 `--runtime native`，标靶仍经 `ssb_runtime.sh` 运行已安装的 `ssb_probe`，但不加载 WSL 组件，使用系统驱动侧 OptiX 库。生成不需要 Gazebo/RViz GUI 或私有 Mesa，但需要 `libvips-tools`、ROS/Gazebo Python 绑定和真实可用的 OptiX 后端。原生主机整套部署验收尚未完成。

`work`、`output` 必须是新目录且互不嵌套。已有默认演示时，可把输出改为 `local_data/stage_b/contact_demo_buffered_NEW`；工具拒绝覆盖。失败后查阅工作目录的 `generation.log` 和 `FAILED`，修复原因后用新工作目录重跑，下载原图可继续复用。

默认有效区仍为 [0,20] m，物理内壁两端各延伸 2.5 m；新增包名为 `contact_demo_buffered`，已有旧包不覆盖。

生成器调用现有模块，顺序如下：

1. 验证所有下载原图与来源清单。
2. 用 `stage_b_runtime_surface prepare-set` 生成 Concrete034 的分层合成配方，`prepare-filler` 生成砂浆细节。保持 0.1 mm 生成网格及 4 种抹痕方向一致的变换，不预烘焙整条隧道的高清大图。
3. 校验 Git 中的两份裂缝 PNG，并在工作目录复制、重定位它们的矢量目录；运行 `stage_b_defects` 的布局、细化、深度步骤。不会修改 Git 中的原目录，也不需要重新生图。
4. 用 `stage_b_scene` 生成隧道网格并做漏光审计。
5. 用 `stage_b_optics` 绑定新资产；3 个曝光时刻、临界几何 16 条射线、复杂裂缝 64 条射线、2×2 纹理足迹积分，生成新的私密光学密钥。
6. 用 `stage_b_gui` 生成有界预算的同源低分辨率 GUI 预览。
7. 用 `prepare_contact_demo.py` 显式传入新世界、配置和规格，生成前驱、双测量轮、2 mm 竖向起伏/左右高差、0.2 mm 车轮柔性的世界，校验物理装配。
8. 准备并渲染独立标靶，拟合当前演示的镜头/暗场/平场标定，核对光学身份，再由 `demo_bundle` 导出全部运行依赖并改成相对引用。

这些步骤的具体参数可查 [build_demo_from_sources.py](../tools/build_demo_from_sources.py)；逐模块命令见 [历史逐模块生成记录](history/STAGE_B_SNAPSHOT_2026-10-03.md#4-重新生成资产)。首次部署使用本页入口，它处理了历史裂缝目录中的原机器绝对路径，并避免演示生成器的默认旧资产输入。

## 许可

仓库代码与随仓库发布的生成素材（`assets/cracks/generated/` 中的裂缝图集、矢量目录与生成提示词）采用根目录 [LICENSE](../LICENSE) 的 Apache-2.0。Concrete034 等官网原图不入库，由使用者按 §1 下载，适用各自来源的许可（ambientCG 为 CC0）；本机生成的资产包放在 `local_data/`，不随仓库分发。现有 v1 目录的 `source_file` 字段是本机绝对路径，仅作来源记录；文件哈希受下游绑定，因此不改写。之后生成的新版本只记录文件名，身份由同一记录中的 `source_sha256` 确定。

## 3. 输出与验证边界

```text
local_data/stage_b/sources/       官方原图、压缩包与 downloads.json
local_data/stage_b/build_from_sources_NEW/
  generation.log                顺序生成的完整日志
  surface/ filler/ cracks*/      纹理配方、砂浆与裂缝派生资产
  geometry/ optics/ gui/         隧道网格、光学场景和 GUI 预览
  catalogs/ demo/                本机裂缝目录、接触世界与标靶标定
local_data/stage_b/contact_demo_buffered/
  capture.yaml spec.yaml         成套的生成配置
  calibration.json gui.config   当前光学身份的图像标定与 GUI 配置
  assets/ world/ bundle.json     相对引用的全部运行依赖与哈希清单
```

新包是由当前代码和配方重建的演示，不是旧机器演示的逐字节副本。随机生成的光学密钥使光学身份不同，必须使用新包自己的标定。生成阶段的哈希、漏光、物理装配和光学身份检查，不等同于完成新包的采集、画质及实时率验收；生成后按 README 运行 3 m **Wall coverage** 任务验证，再扩大到 20 m。

演示配置含仿真真值及密钥，仅供采集生成端使用。重建只读采集会话的公开配置、原图、编码器和图像估计的标定，不能把生成目录当成重建输入。

文件放在 WSL 的 Linux 文件系统。下载、原图解码和二进制资产采用顺序大块 I/O；阶段日志集中写入一个文件，不生成逐像素或逐行小文件。GUI 预览仍有有限数量的网格/图片，不能据此宣称生成阶段完全没有小文件。

生成链的 2026-10-02 本机验证与清理范围见 [历史快照](history/ASSETS_SNAPSHOT_2026-10-05.md)；该记录不替代本次生成包的短程采集检查。
