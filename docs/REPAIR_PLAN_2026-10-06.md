# 2026-10-06 全面审核修复计划

状态：阶段 0 完成，其余未开始（分支 `repair/2026-10-06`）。来源是 2026-10-06 全项目审核；用户已确认四项取舍（见 §0.2）。完成后按惯例移入 `docs/history/` 并在索引登记。

审核基线：HEAD `6eba2ab`，工作树干净；`tools/run_tests.py` 全部通过，995 项（906 Python + 84 gtest + 5 ctest 包装），0 跳过，34 s。

## 0. 总约束与已确认决策

### 0.1 贯穿约束

- **不改写 20 m 里程碑证据。** 历史会话、首次报告、协议和验证记录都不覆盖；所有新结果写到新目录。
- **冻结协议绑定当前工作树文件，这是隐藏约束。** `holdout_protocol verify` 的 `production_sources_unchanged` 会按协议里记录的绝对路径，对当前工作区的 28 个生产源码重新计算哈希（`holdout_protocol.py:215-225`）。本计划第 1、2、5 阶段都会修改其中的文件，修改后在主工作区复核里程碑协议必然失败。因此第 0 阶段必须先打标签，并支持在标签工作树中复核。
- **每项单独提交。** 按 DEVELOPMENT_RULES，提交后完整构建再测试；文档提交同样改变构建身份。
- **数值不变的改动要有证据。** 用里程碑公开输入重放 D3，证明轨迹逐位相同。不能只用"测试全绿"代替。
- **需要先征得同意的操作。** 推送标签或 workflow 到 GitHub、`sudo apt` 安装工具、删除任何数据，执行前逐项确认。

### 0.2 用户已确认的取舍（2026-10-06）

| 问题 | 决定 |
|---|---|
| IRLS 收敛 | 现在只增加报告字段，数值逐位不变；在里程碑公开输入上做诊断重放，量化多迭代的影响。算法改动并入 50 m 冻结，届时用新种子重新验收 |
| CI | GitHub 托管机跑 CPU CI，本机 GPU 全套作为合并门禁。新增显式 `-DSSB_IMAGING=OFF`，写入构建印记，采集时拒绝使用该构建；默认仍为 ON，找不到 OptiX 照旧报错 |
| 数据清理 | 先盘点，再逐目录由用户确认；先归档小记录并回读校验，然后才删除大数组 |
| 代码风格 | 增加 `.clang-format` 和 ruff 配置；只格式化本轮改动的行和审核点名的压缩代码块，不做全仓库格式化 |

### 0.3 审核后补充发现

截至审核时，**全部 63 份 D3 报告（包括里程碑的两轮拟合）的 IRLS 都正好跑满 `max_irls=5` 轮**。报告里没有收敛标志，无法区分"第 5 轮恰好收敛"和"撞到上限"。

里程碑第 1 轮拟合中，最小窗口权重依次为 0.1153 → 0.1109 → 0.1084，末轮仍变化约 2%。而收敛判据是逐点相对变化不超过 0.5%，所以大概率没有收敛。第 2 轮末轮变化约 0.6%，处于边界。这一项因此从"报告缺字段"升级为需要量化影响的问题（§1.3）。

## 阶段 0：保护基线（最先做）

### 0.A 给里程碑打标签

- 在 `03cde91`（协议 `code_commit`，即采集和重建所用代码）打注释标签 `milestone-20m-code`。
- 在 `6eba2ab`（里程碑文档收尾）打 `milestone-20m`。
- 标签先只建在本地；推送到 origin 需要用户确认。

### 0.B 协议复核支持源码搬迁

- **改动**：`holdout_protocol.py` 的 `verify` 增加 `--source-root PATH`。把协议里的工作区前缀 `/home/jerry/robot_ws/Subway_scan_bot_sim` 映射到 PATH（例如 `git worktree add ../ssb_m20 milestone-20m-code`），然后再计算源码哈希。
- **报告**：记录映射关系和映射后的实际路径。映射只影响 `production_sources_unchanged` 这一项，其余检查照旧读归档记录。这与现有 `audit-public --relocation` 的语义一致。
- **测试**：
  - 映射到未改动的副本时通过；
  - 副本中任一文件改一个字节时失败；
  - 副本中缺文件时失败；
  - 映射前缀不匹配协议路径时直接报错，不允许静默回退到主工作区。
- **验收**：在标签工作树中执行里程碑协议复核，写入新的 `local_data/evaluation/milestone20_holdout_seed20270119_20261005/protocol_verification_relocated_20261006.json`，结果为 pass。原有 `protocol_verification.json` 不动。

### 0.C 文档

在 EVALUATION.md 增加一段，说明代码前进后如何在标签工作树复核旧协议。

### 阶段 0 执行记录（2026-10-06）

- 0.A：本地注释标签 `milestone-20m-code`→`03cde91`、`milestone-20m`→`6eba2ab`；尚未推送。
- 0.B：`5256396` 实现 `verify --source-root`。新增 9 项测试（检出一致通过；改一字节、缺文件、检出提交不同各自失败；不可映射路径报错）。把映射临时改回读冻结路径时，2 项新测试失败，恢复后全部通过。
- 新发现：`b847ef9`（10-05 17:25）在首次核验（17:08）之后修改了 `global_resample.py` 和 `global_cuda.py` 中的对比预览与描述字符串，不涉及全图反采样几何。因此从那以后，在主工作区直接复核里程碑协议必然失败，不是本计划造成的。
- 实测：主工作区直接复核失败，仅 `production_sources_unchanged` 一项；在 `milestone-20m-code` 检出中复核 19 项全过，记录为 `protocol_verification_relocated_20261006.json`。原 `protocol_verification.json` 未改动。
- 0.C：EVALUATION.md §4 写明复核方法与上述发现；顺带修正该文件 3 处标题前的字面 `\n`，它们会导致标题无法渲染。
- 全套测试 1004 项（915 Python）通过，0 跳过；Python 用例数比基线多 9 项，与新增数一致。

## 阶段 1：D3 正确性与配置入口（审核 P1-3、P1-4、P2-9 部分）

### 1.1 统一 D3 配置来源，拆开轴偏航开关

**现状**

| 入口 | 问题 |
|---|---|
| `optimize_bands` 命令行 | 默认 `--attitude-spacing-m 0.05`；`GeometrySettings` 默认 backend 为 cpu，而它默认 cuda；没有 `--slow-translation`，无法复现里程碑 |
| `match_bands` | 默认 `--spacing-m 0.4`；`public_reconstruction` 和 `holdout_protocol` 是 0.2；里程碑实际用 0.1 / height 256 |
| `reconstruction_settings` | `fit_axis_yaw=slow_translation`（`global_geometry.py:93`），两个自由度绑在一个开关上 |

**改动**

1. `reconstruction_settings(..., fit_axis_yaw=None)`：传 `None` 时等于 `slow_translation`，旧调用得到的 `asdict` 完全不变，已冻结协议的 `d3_settings_unchanged` 不受影响。
2. `public_reconstruction`、`holdout_protocol declare`、`reconstruction_budget` 增加 `--fit-axis-yaw / --no-fit-axis-yaw`（`argparse.BooleanOptionalAction`，默认跟随 `--slow-translation`）。
3. `optimize_bands.main()` 改为通过 `reconstruction_settings` 构造设置，补上 `--slow-translation` 和 `--fit-axis-yaw`。默认值改成与 `public_reconstruction` 一致（姿态 0.02 m，observed knots 开，backend cuda）。这是单独命令行的行为变化，写入 STAGE_D。
4. `match_bands --spacing-m` 默认改为 0.2，与另外两个入口一致；里程碑使用的 0.1 / 256 仍需显式传入，文档写明。

**测试**（新增 `test/test_d3_entrypoints.py`）

- 三个入口用同一组参数解析后，`asdict(settings)` 必须完全相等。
- 用里程碑参数（`--adaptive-attitude --slow-translation --relative-encoder-scale`，cuda）得到的设置，必须等于冻结协议的 `d3` 字典。该字典以字面量形式写进测试，不读取 `local_data`。
- `--slow-translation --no-fit-axis-yaw` 得到 `fit_axis_yaw=False`，且能通过 `validate()`。
- 修复后，把默认值临时改回旧行为，确认对应测试会失败（守则 W#33）。

**验收**：§1.3 的 D3 重放用新命令行复现里程碑，结果逐位相同。

### 1.2 报告 IRLS 收敛状态（数值不变）

**改动**（`optimize_bands._fit`）

- 每轮记录 `max_relative_weight_change = max|updated−current|/base`，写入 history。
- 返回值和 `report.json` 新增：

  ```
  irls = {converged, iterations, max_irls, criterion: 0.005, final_max_relative_weight_change}
  ```

  每个 `fit_passes` 条目也带同样字段。
- **不改变**求解次数、权重和系数。`current = updated` 只影响下一轮，末轮之后没有求解，所以现有逻辑保持原样。只是报告需要写明：末轮计算的 `minimum_window_weight` 描述的是下一组权重，并未用于最终系数。
- `public_reconstruction` 的摘要和 `holdout_protocol verify` 输出 `irls_converged`，**只作信息记录，不作门限**：现在加门限会追溯性地否定里程碑，而改算法已决定并入 50 m 冻结。
- 报告 schema 递增次版本（`v4` → `v4.1` 或新增字段说明），旧报告仍可加载。

**测试**

- 合成小问题：`max_irls=1` 时 `converged=False`；放宽到能收敛时为 `True`；两种情况下 `iterations` 都等于 `len(history)`。
- 同一输入在改动前后的系数逐位相同（用固定合成表，期望值由改动前的提交生成并写死）。

### 1.3 IRLS 诊断重放（在里程碑公开输入上）

- **输入**：`sessions/milestone20_holdout_seed20270119_20261005/{unroll,matches}` 和 capture 的 `observable_config.json`，只读。
- **运行**，输出到新目录 `sessions/irls_diag_20261006/`：
  - (a) 里程碑设置，`max_irls=5`：`trajectory.json` 与原 `fit/` **逐位相同**，同时验证 §1.1 和 §1.2 没有改变数值。若不同，先查原因，不放宽比较。
  - (b) `max_irls=10`（`validate` 允许的上限）：记录是否收敛、第几轮收敛，以及留出点 P50/P95/P99 和窗口内/间 P95 的变化。
  - (c) 如果 10 轮仍不收敛，记录权重变化曲线，供 50 m 设计参考；不在本轮调参。
- **性质声明**：这是已看过数据上的开发诊断，不是盲验，不能把 (b) 的结果写成新的里程碑数字。
- **记录**：结果写入 RECONSTRUCTION_METHODS.md 的"已知限制"，并在 ROADMAP §3 增加一项"IRLS 收敛策略随 50 m 冻结"。
- **资源**：只做 D3 与图像残差评价，不做整图反采样，磁盘增量应在 1 GB 以内。

### 1.4 匹配图连通性改为显式判断

- **改动**：`PreparedFit.prepare` 不再依赖"pair 数等于 n−1"这一隐式条件，改为：
  - (1) 断言每个接受窗口满足 `bands[1] == bands[0]+1`；
  - (2) 对训练和留出两个子集各自调用 `graph_components`，要求只有一个连通分量。
- **测试**：非相邻 pair 应报错；存在环但缺一个相邻边时，旧判断会误通过，新判断必须报错（用旧代码确认能复现这种误通过）。

### 1.5 系数上限去重

- 把 `refine_attitude_knots` 和 ROADMAP 中写死的 2048 合并为 `global_geometry.MAX_COEFFICIENTS`。
- `observability()` 在 `model.size > MAX_COEFFICIENTS` 时明确报错，而不是去分配稠密矩阵。改为稀疏或分块实现留给 50 m 分段，在 ROADMAP 登记。
- **测试**：上限常量只有一处定义；超限时报错信息包含实际系数数。

## 阶段 2：采集管线小修（审核 P2-9 部分）

### 2.1 右测量轮边沿计数

- **改动**：`TimingStats` 增加 `right_odo_edges`，`timing.cpp` 统计，`pipeline.cpp` 的 `timing_stats` 增加 `odometer_right_edges` 键。
- **兼容**：先查 `validate_stage_a.py`、`public_capture.py`、`wall_coverage.py`、`check_session.py` 中是否有对 timing 键集合的严格比较；如有，改为允许新增键。
- **测试**：在 `test_timing` 中开启 contact，断言统计数等于输出边沿数；在 `test_pipeline_failure` 中断言 `session.json` 的计数等于表行数。

### 2.2 生产端失败时 Progress 不再报告 complete

- **现状**：`Pipeline::Wait` 在 `producer_error` 非空时，先设置 `s.complete=true` 再抛出异常，`Progress()` 因此显示 phase=complete、failed=true。
- **改动**：
  - 新增原子量 `ended_failed`，只有成功时才设置 `complete`；
  - 发生 worker 失败或生产端失败后，`Progress()` 返回 phase `"failed"`；
  - `mission_manager.merge_capture` 的 phase 顺序把 `failed` 视为终态，同序号快照不能把它回退成其他 phase。
- **测试**：
  - gtest：生产端错误时 Progress 返回 phase=failed；worker 错误时同样如此。
  - pytest：`merge_capture` 的 failed 与 complete 乱序到达时，结果仍为 failed。
  - 回放旧行为，确认测试会失败。

### 2.3 头文件

`pipeline.cpp` 显式 `#include <fstream>`。

**验收**：完整构建后跑全套测试；再按 `tools/integration/mission.py`（§6.3 改名后的路径）跑一次 WSL 任务冒烟，确认 RViz 面板对完成和失败的显示正常。

## 阶段 3：运行时可移植（审核 P1-1）

### 3.1 统一运行时入口

**新增 `tools/ssb_runtime.sh`**

- 模式由 `SSB_RUNTIME=wsl|native|auto` 决定，默认 `auto`。
- `auto` 读取 `/proc/sys/kernel/osrelease` 判断 WSL；测试时可用 `SSB_OSRELEASE_FILE` 覆盖。
- 无法判断时直接报错，不猜测。
- **wsl 模式**：保持现有行为，即私有 Mesa 加隔离 OptiX，`LD_LIBRARY_PATH` 与现在逐字相同。
- **native 模式**：不改 `LD_LIBRARY_PATH`，不设置 `GALLIUM_DRIVER` / `MESA_D3D12_*`，直接执行命令。

**`with_optix_runtime.sh` 去掉写死的驱动版本**

- 版本取自 `SSB_OPTIX_DRIVER_VERSION`；未设置时，在运行库目录里查找 `libnvidia-rtcore.so.*`，必须恰好匹配一个。
- 匹配到零个或多个都报错。
- 默认目录仍是 `~/opt/optix-runtime-<version>`。

**GUI 环境变量**：`GALLIUM_DRIVER` 等只在 wsl 分支设置。

### 3.2 替换调用点（审核时 grep 到的全部）

| 文件 | 处理 |
|---|---|
| `src/ssb_core/CMakeLists.txt:122` | GPU 测试改用 `ssb_runtime.sh` |
| `src/ssb_tools/setup.py:11` | 安装 `ssb_runtime.sh`，保留 `with_optix_runtime.sh` |
| `src/ssb_tools/ssb_tools/package_paths.py:15` | `probe_command()` 改走 `ssb_runtime.sh`，这是生产自检路径 |
| `tools/build_demo_from_sources.py:85` | `--runtime` 改为传给 `ssb_runtime.sh` |
| `tools/run_mission.sh`、`run_gz.sh`、`run_gz_gui.sh`、`run_contact_dynamics.sh`、`run_d3_holdout.sh` | 走统一入口 |
| `tools/test_gazebo_plugins.py:119`（§6.3 后改名） | 走统一入口 |

### 3.3 记录运行模式

`config/provenance.json` 记录运行模式和 OptiX 运行库目录，便于事后区分。

### 3.4 测试与验收

- **新增 pytest**：分别伪造 WSL 和原生的 osrelease，检查生成的环境：wsl 模式的 `LD_LIBRARY_PATH` 与旧脚本逐字相同；native 模式不含 `/usr/lib/wsl/lib`。版本匹配到零个或多个时报错。
- **本机（WSL）**：全套测试，加一次 `run_mission.sh` 冒烟。
- **原生 Linux**：本机无法验证。LINUX.md 改写为"脚本已支持 native 模式，尚未在原生主机验收"，在另一台主机实测前不删除这条警示。

## 阶段 4：许可证与 CI（审核 P1-2）

### 4.1 LICENSE

- 根目录添加 Apache-2.0 全文，与四个 `package.xml` 一致。
- ASSETS.md 补充一段：仓库内生成的裂缝图集和目录随仓库许可证发布；Concrete034 等官网素材不入库，按各自许可证（ambientCG 为 CC0）由用户下载。
- `assets/cracks/generated/*.catalog.json` 和 `*.prompt.txt` 中的本机绝对路径不改写，因为这些文件的哈希被下游包绑定。只修改生成器，让以后的新版本写仓库相对路径。

### 4.2 显式 CPU 构建开关

- `ssb_core/CMakeLists.txt` 增加 `option(SSB_IMAGING "Build OptiX/CUDA imaging" ON)`。ON 时行为完全不变，找不到 OptiX 照旧报 `FATAL_ERROR`。
- **拆库**：从 `ssb_core` 中拆出 `ssb_capture`（`session.cpp`、`pipeline.cpp`，不依赖 CUDA）。`ssb_core` = `ssb_capture` + OptiX 渲染器。`test_pipeline_failure` 改为链接 `ssb_capture`。
- **OFF 时**：
  - 只构建 `ssb_timing`、`ssb_normal`、`ssb_ray_numeric`、`ssb_capture` 以及对应 gtest；
  - 构建印记写入 `imaging: false`；
  - `ssb_render`、`ssb_probe` 等不生成，采集入口在启动时检查印记并拒绝运行。
- **测试**：本机分别用 ON 和 OFF 构建；OFF 构建下调用采集入口必须失败，并报出明确原因。

### 4.3 测试执行档位

- `test_runner` 增加 `--profile cpu-ci`。仍然先收集全部用例，再按显式 marker（`requires_cuda`、`requires_optix`、`requires_gazebo`）剔除。
- 汇总报告必须列出被剔除的用例 ID 和数量；收集失败或剔除数为 0 但有 marker 时报错。
- 默认档位（本机）不变：零剔除，零跳过。
- **marker 审计**：在没有 CUDA 运行库的干净环境（venv 加 `-DSSB_IMAGING=OFF` 构建）里先跑一遍 CPU 档，逐一标记失败用例的依赖原因，不能用 marker 掩盖真实失败。

### 4.4 GitHub Actions

- 新增 `.github/workflows/ci.yml`：ubuntu-24.04，`ros-tooling/setup-ros` 安装 Jazzy，apt 安装依赖。
- 步骤：`colcon build --packages-select ssb_core ssb_tools --cmake-args -DSSB_IMAGING=OFF` → `run_tests.py --profile cpu-ci` → ruff → 文档链接检查（§6.4）。
- 触发条件为 push 和 pull_request，使用托管机，不涉及自托管 runner 的安全问题。
- workflow 提交前在本地用 `act` 或干净 venv 模拟；推送需要用户确认。

### 4.5 门禁规则

DEVELOPMENT_RULES 增加一条：合入 main 前必须本机跑 `tools/run_tests.py` 全套。CI 通过不等于 GPU、OptiX、Gazebo 通过；README 的测试数字只引用本机全套结果。

## 阶段 5：代码风格（审核 P2-6、P2-9 部分）

### 5.1 配置

- **`.clang-format`**：基于 Google，`ColumnLimit: 120`，`IndentWidth: 2`。先对 `pipeline.cpp` 的早期代码运行 `clang-format --dry-run` 调参，使未改动的规整代码几乎没有 diff。clang-format 需要 `sudo apt install clang-format`，执行前确认。
- **`ruff.toml`**：line-length 120；全仓库启用 `F`（pyflakes）规则。`E501`、`E701`、`E702` 只对改动行检查。
- **新增 `tools/lint_changed.py`**：根据 `git diff -U0 <base>` 过滤出改动行，只报告这些行上的问题；CI 和本地使用同一个脚本。

### 5.2 只格式化点名的压缩代码块

- `timing.cpp`：右测量轮相关的成员、`Push` 中的相关块、`TimingOutput::Clear`。
- `pipeline.cpp`：`FinishDistanceMotion`、`Wait` 中 physical 快照与 motion 分支的压缩行。
- `stage_b_runtime_surface.py`：52 行超过 120 列，并把分号串联语句拆开。这会改变该生成器的源码哈希，旧资产包仍记录旧哈希，新资产包由新代码生成，在 ASSETS.md 注明。
- **要求**：格式化单独提交，与功能改动分开；完成后 §1.3 (a) 的 D3 重放和全套测试都必须保持不变。

### 5.3 pyflakes 指出的死变量

| 位置 | 处理 |
|---|---|
| `global_geometry.py:218` `raw_vx` | 删除 |
| `stage_b_runtime_surface.py:187` `factors` | 删除；先确认它没有被用于随机数流的副作用，以免改变生成结果 |
| `tools/test_mission_panel.py:40` `timer` | rclpy 节点自己持有定时器，改名为 `_timer`，表明是有意保留引用，行为不变 |

## 阶段 6：文档与集成脚本（审核 P2-7、P2-8、P1-3 文档部分）

### 6.1 仓库外部链接

- 现行文档 `DEVELOPMENT_RULES.md`、`design/REFERENCES.md`：把 `../../4WIDS_agv/...`、`../../climbot_sim/...` 改为纯文字引用，例如"相邻项目 4WIDS_agv `docs/LESSONS.md` W#25（未公开）"，保留条目编号。
- history 文档：按归档规则只调整链接，把这些链接改为行内代码路径，不改正文。

### 6.2 STAGE_D 和 QUICKSTART

- 三步示例标明为"名义基础模型"。里程碑配置给出 `public_reconstruction --spacing-m 0.1 --height 256 --max-q-shift-mm 10 --adaptive-attitude --slow-translation --relative-encoder-scale` 的完整命令，与冻结协议一一对应。
- 删除"不依赖工具旧的 0.4/0.05 m 默认值"等将随 §1.1 失效的说法。

### 6.3 集成脚本改名

- `tools/test_*.py`（11 个）移到 `tools/integration/` 并去掉 `test_` 前缀，例如 `tools/integration/mission.py`。
- 新增 `tools/integration/README.md`，表格列出每个脚本的前置条件（Gazebo / OptiX / RViz / 资产包）、用途、典型耗时和建议运行时机（采集链改动后、发布前）。
- 更新引用：`docs/WHEEL_ERROR.md`，以及脚本 docstring 中互相引用的路径。history 中的旧路径不改，并在 history/README 中加一句说明改名。

### 6.4 文档链接检查

- 新增 `tools/check_doc_links.py`，在审核时用过的脚本基础上增加锚点检查。
- 新增 pytest `test_docs_links.py`：所有 md 文件中的相对链接必须存在，且不得指向仓库外部。这条会进入默认全套和 CPU CI。

### 6.5 收尾文档

- README：测试数字更新为实施后本机全套的实际数量。
- docs/README 索引：登记本计划，完成后改为指向 history。
- 同步更新 ROADMAP（IRLS 策略、观测性扩展、原生 Linux 验收）、DEVELOPMENT_RULES（CI 与本机门禁、lint）、LINUX.md、WSL.md、ASSETS.md。

## 阶段 7：数据与仓库卫生（审核 P1-5、P2-9 部分）

### 7.1 只读盘点

**新增 `tools/inventory_sessions.py`**

- 只做一次 `os.scandir` 加 `stat` 遍历：不计算哈希，不跟随符号链接，按 WSL 守则避免高频元数据操作。
- 输出到 `local_data/inventory_2026-10-XX.json`，并附一份 tsv。

**每个顶层目录给出**

| 字段 | 说明 |
|---|---|
| 唯一 inode 字节数 | 硬链接按 inode 去重 |
| 与其他目录共享的字节数 | 删除该目录实际能释放多少 |
| 文件数 | |
| 被谁引用 | DATA_RETENTION 必保清单、现行文档、history 快照、`docs/media/manifest.json`、`local_data/evaluation/**/protocol*.json` 中的路径、指向该目录的符号链接 |
| 是否被进程占用 | |

**建议分类**

- 必保：清单内，以及被现行文档或协议引用的；
- 媒体来源：被 `media/manifest.json` 引用的；
- 仅历史快照引用；
- 未被引用。

### 7.2 用户逐目录确认

把盘点表交给用户，逐目录勾选"归档后删除大数组 / 保留 / 整体保留"。未经确认不删除任何内容。

### 7.3 执行已批准项（沿用现行清理规则）

1. 把小记录（report、config、log、`provenance.json`、小型诊断）打包到 `local_data/evaluation/cleanup_2026-10-XX/retained_records/*.tar.gz`；
2. 回读核对每个归档成员的哈希；
3. 写入 `cleanup.json` 和 `archive_hashes.json`；
4. 删除已批准的大数组；
5. 再次运行盘点脚本，报告实际释放的空间（按 inode 计）；
6. 更新 DATA_RETENTION.md，并在 history 中追加快照说明。

### 7.4 Git 仓库

- 本地执行 `git gc`：仓库目前有 3238 个未打包对象。这是安全操作，不改历史。
- 媒体政策写入 `docs/media/README.md`：GIF 尽量不重复提交，新动图优先压缩或改用外链/Release 附件。已推送的历史不改写。

## 阶段 8：总验收

1. 本机完整构建（`SSB_IMAGING=ON`），`tools/run_tests.py` 全套通过，0 跳过；新增测试数与预期一致（守则 W#26）。
2. 本机 `SSB_IMAGING=OFF` 构建加 `--profile cpu-ci` 通过，剔除清单与 marker 一致；推送后 GitHub CI 为绿（推送前确认）。
3. §1.3 (a)：D3 重放与里程碑轨迹逐位相同。
4. §0.B：标签工作树中的里程碑协议复核通过。
5. WSL 任务冒烟：通过 `run_mission.sh` 启动，Start/Stop 正常，完成和失败状态显示正确。
6. 文档链接检查为零失败，pyflakes 为零。
7. 本计划移入 `docs/history/`，状态写为"完成"，记录每项的提交号和证据路径。

## 执行顺序与工作量

| 顺序 | 内容 | 依赖 | 估计 |
|---|---|---|---|
| 1 | 0.A 标签、0.B 搬迁复核 | 无 | 0.5 天 |
| 2 | 1.1 入口统一、1.2 IRLS 报告、1.4、1.5 | 0 | 1 天 |
| 3 | 1.3 诊断重放 | 2 | 0.5 天（主要是计算时间） |
| 4 | 2.1–2.3 管线小修 | 0 | 0.5 天 |
| 5 | 3 运行时入口 | 0 | 1 天 |
| 6 | 4.1 LICENSE、4.2 拆库与开关、4.3 档位、4.4 CI | 5 | 1.5 天 |
| 7 | 5 风格 | 2、4 之后（避免与功能改动冲突） | 0.5 天 |
| 8 | 6 文档与改名 | 1–7 | 0.5 天 |
| 9 | 7 盘点 → 用户确认 → 清理 | 无（可并行） | 0.5 天 + 用户确认时间 |
| 10 | 8 总验收 | 全部 | 0.5 天 |

## 风险

| 风险 | 对策 |
|---|---|
| §1.3 重放不能逐位复现 | 先查 CUDA 非确定性、线程分块和 BLAS；找到原因前不合入 1.1/1.2，也不放宽比较 |
| 拆出 `ssb_capture` 改变 `ssb_core` 的链接，影响 Gazebo 插件 | ssb_gazebo 仍链接 `ssb_core`（包含 `ssb_capture`）；跑 Gazebo 插件集成脚本加任务冒烟 |
| marker 审计时发现 CPU 用例隐式依赖 GPU 库 | 修正导入方式（延迟加载），不能把用例整体标为 GPU 了事 |
| 清理时误删被引用的数据 | 盘点覆盖协议、媒体清单和符号链接；只删用户批准的目录；先归档再回读校验 |
| 格式化改变生成器哈希 | 单独提交，在 ASSETS.md 说明；旧资产包不受影响 |
