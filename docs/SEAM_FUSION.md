# 接缝亮度与窄带融合

D3 的位姿、共享深度和未融合几何验收保持独立。融合生成另一个完整产品，原图与未融合整图不修改；不以融合画面替代接缝几何评价。

## 方法与边界

- 按公开名义重叠域固定选 64 个角度、65 个轴向分数位置，经过已有优化映射从原始行取样。偶数角度行估计，奇数行留出；无效、饱和、过暗/过亮及高梯度点只从亮度拟合排除，仍进入输出覆盖与几何评价。
- 每个相邻圈对取 `log(B/A)` 中位数，连通分量内求每圈一个增益，有单位增益先验、几何平均增益为 1。增益限制在 `[1/1.08, 1.08]`，超界时整个分量统一衰减；不拟合偏移、逐像素亮度场或颜色。
- 保留视场中心更近的原有主来源，只在最优与次优来源评分接近时线性混合；默认宽度约 2 mm（约 10 个输出像素），由测得的原始列间距换算。镜头映射不均匀和姿态会使实际壁面宽度稍有变化，这不是精确 2 mm 的世界距离硬边界。其他区域使用单一来源。整圈缺失时拒绝融合，不跨缺失圈补洞。
- CUDA 仍从 Mono8 原始数据一次重采样，采后暗场/平场及畸变映射沿用 D1。CPU 参考独立选取两圈并计算权重；GPU 分块新增两个浮点值及一个圈号，共 10 字节/像素，受现有 256 MiB 工作分配预算约束，出错不降级。
- 法线受光方向不同造成的局部明暗变化不能被每圈一个增益消除。窄带混合也可能在残余错位处产生轻微细节平均，必须保留无融合裂缝对照；不做金字塔混合、锐化、自动对比度或填洞。

## 运行与核验

先完成 D3 未融合几何验收及完整 `global_mosaic` 输出。以下路径分别指已有公开观测根、D1、优化拟合与未融合整图，不要把 `evaluation/` 传给生产工具：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m ssb_tools.seam_fusion \
  --public-root sessions/NEW/capture --raw sessions/NEW/capture/raw \
  --unroll sessions/NEW/unroll --trajectory sessions/NEW/fit \
  --baseline sessions/NEW/mosaic --output sessions/NEW/fusion \
  > /tmp/ssb_fusion.log 2>&1
python3 -m ssb_tools.validate_global_mosaic \
  --unroll sessions/NEW/unroll --trajectory sessions/NEW/fit \
  --mosaic sessions/NEW/fusion --output local_data/evaluation/NEW/fusion_pixels \
  > /tmp/ssb_fusion_validation.log 2>&1
```

WSL 下用 `tools/with_optix_runtime.sh` 包装以上 Python 命令，并在包装器内部加载 ROS/工作区；原生 Linux 直接使用系统 CUDA 运行库。默认 CUDA，可显式选 `--backend cpu` 作为参考，不建议用 CPU 生成全图。

`report.json` 保存增益、取点规则、每对训练/留出数量、增益边界衰减及留出 DN 差；`fusion_report.json` 保存与原未融合图的完整覆盖文件哈希相等证明及阶段耗时；`fusion_provenance.json` 绑定原未融合图、公开输入、轨迹和融合报告。CLI 的 `public_audit.json` 记录实际 Python 读事件，原生/外部 I/O 不在完整覆盖范围。

融合图沿用 uint16 DN/64、无效码 65535，另存完整覆盖计数和游程。验收器逐像素核对无效掩码、计数和游程，再检查固定 CPU 原图采样、板缝深度点以及按公开几何预先选择的窄带融合点；来源圈与权重随探针保存。全图覆盖文件必须与未融合图逐字节一致。这些检查证明重采样和来源正确，不能证明几何误差或裂缝清晰度。

## 验证状态

方法开发中的合成测试已覆盖增益、边界、缺行、饱和、非零姿态、共享深度、分块一致性、输入篡改和对齐的细裂缝。实际 3 m 与 20 m 的完整结果、耗时和资源数据将在冻结代码后的回归中记录；已有采集只作回归，不称作新的盲验采集。
