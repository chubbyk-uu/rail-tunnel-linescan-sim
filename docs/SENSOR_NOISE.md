# 线阵传感器噪声

默认演示关闭噪声，20 m 综合验收开启假设档；当前结果见 [MILESTONE_20M](MILESTONE_20M.md)。本页只说明模型，旧 3/20 m 噪声批次与测试记录见 [快照](history/SENSOR_NOISE_SNAPSHOT_2026-10-05.md)。

## 模型与参数

先完成像素面积、曝光期间运动和照明积分，再对每个传感器像素执行一次噪声读出，最后量化、裁切至 Mono8。不会给每条光线独立加噪声后求平均，以免错误降低传感器噪声。

设原有渲染响应给出的线性 DN 为 `s`，固定列响应为 `g`，电子/DN 为 `k`，曝光为 `t`，暗电流为 `d`。电子计数均值 `μ = max(s,0)·g·k + t·d`，输出为 `bias + (photon_count(μ) + read_e·N(0,1))/k`。低于 64 个电子用泊松逆 CDF 抽样，高计数用四舍五入至整电子的高斯近似；不声称全量精确泊松。高计数近似的舍入可额外贡献约 1/12 电子²方差。固定列响应为均值 1、指定相对标准差的正值对数正态分布。

安装配置 `ssb_core/config/sensor_noise_assumed.yaml` 定义第一档：

| 参数 | 数值 | 解释 |
|---|---:|---|
| `electrons_per_dn` | 20 e⁻/DN | 假设转换增益 |
| `read_noise_e` | 8 e⁻ RMS | 模拟读出噪声，约 0.4 DN |
| `dark_current_e_per_s` | 100 e⁻/s | 8 µs 时均值仅 0.0008 e⁻ |
| `bias_dn` | 4 DN | 暗电平，减少零附近的读出裁切 |
| `prnu_fraction` | 0.005 | 0.5% 固定列响应不均匀 |

这些全部是**仿真假设，不是 DALSA Linea 的实测参数**。原有相对照明/响应作为电子计数均值的输入，尚不建立光源功率、量子效率与曝光的绝对电子响应；未模拟温度漂移、热像素、暗信号不均匀、完整满阱及镜头 MTF/离焦。

## 确定性与输入隔离

随机键由私有 `realization_seed`、全局曝光 `record.sequence`、传感器列和独立抽样流构成；不使用批内行号、线程次序或图像分块编号。固定列响应只使用 `pattern_seed` 与列号，初始化时生成 4096 个系数，额外设备内存约 16 KiB，不保存逐行随机状态或新的逐像素文件。

更改批大小或重放同一私有配置，会逐字节复现噪声原图。更换 `realization_seed` 得到新噪声实现，不改变光学签名；更换固定响应种子、转换增益、读出/暗电流、暗电平或 PRNU 会改变签名，必须重做标定。噪声关闭时签名与原方案一致。

参数和种子仅进入生成配置和会话 `evaluation/truth.json`；公开配置只声明噪声是否开启，重建不读取真实系数或种子。标定只用独立暗场、亮场和条纹图估计暗偏移、列增益和射线映射，增加实际观测的暗场/亮场时间标准差与采样行数。各标靶使用不同的时间随机种子，固定列响应相同；不能让标定与留出图共用同一噪声实现。噪声生成器已统一使用 `ssb_probe --centered-bench` 的独立居中夹具，防止非零安装偏差污染已知距离靶；非零偏移与双轴倾斜的标定入口有独立回归；当前测试汇总见里程碑。

## 生成与验证

先完整构建并加载已安装包，再运行（WSL 需用已有 OptiX 包装环境）：

```bash
python3 -m ssb_tools.sensor_noise_demo \
  --demo local_data/stage_b/contact_demo_buffered \
  --output local_data/stage_b/noise_assumed_NEW \
  > /tmp/ssb_noise_demo.log 2>&1

bash tools/run_d3_holdout.sh \
  sessions/d3_noise_NEW local_data/evaluation/d3_noise_NEW \
  12 3 local_data/stage_b/noise_assumed_NEW \
  > /tmp/ssb_noise_holdout.log 2>&1
```

生成器验证源演示哈希，硬链接只读资产（跨文件系统才复制），单独写新的采集配置、512 行/靶的标定图与标定结果。标定通过之前不生成可用 `bundle.json`，也不复制旧标定冒充可用。共享资产视为不可变，编辑时须新建文件，不得原位改写共享 inode。运行资源保持相对引用；标定阶段记录属于生成证据，不能作为生产重建输入。
