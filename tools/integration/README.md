# 集成回归脚本

这些脚本会启动真实的 Gazebo、OptiX、RViz 或完整重建流程，不属于 `tools/run_tests.py` 和 CI 的 pytest 收集范围。2026-10-06 前它们位于 `tools/test_*.py`，名字容易被误认为已被测试覆盖，因此移到本目录并去掉前缀。历史文档里的旧路径保持原样，按下表对应。

全部在仓库根目录运行，先 `source /opt/ros/jazzy/setup.bash && source install/setup.bash`；需要 GPU 或界面的脚本用 `tools/ssb_runtime.sh` 包装（界面加 `--gui`），并在包装器内部重新 source ROS 与工作区。输出目录都必须是新目录，完整日志重定向到文件。

| 脚本 | 验证内容 | 前置条件 | 建议运行时机 |
|---|---|---|---|
| `gazebo_plugins.py --output DIR` | 默认 Stage A 命令和两个 Stage B 插件在真实服务器上的采集；各类物理世界不匹配在采集前被拒绝 | OptiX、Gazebo；无界面 | 修改插件、物理世界生成或启动脚本之后 |
| `runtime_shutdown.py --output DIR` | 采集中途改变物理步长：两个插件都必须排空并报告失败 | OptiX、Gazebo | 修改采集管线或插件收尾逻辑之后 |
| `capture_drain.py --output DIR` | SIGINT 后 OptiX 积压能排空，进度与物理时间无关 | OptiX、Gazebo | 修改管线队列或写盘之后 |
| `distance_stop.py --output DIR [--capture]` | 真实轮轨接触下，名义及正负测量轮比例误差的停车距离；`--capture` 另加 3 m 壁面原图、独立重成像和阶段 B 验收 | 资产包 `local_data/stage_b/contact_demo_buffered`、OptiX、Gazebo | 修改接触、编码器或停车逻辑之后 |
| `mission_telemetry.py --output DIR` | Gazebo 暂停心跳与 SIGSTOP 遥测丢失（只跑动力学，不写原图） | Gazebo、ROS | 修改任务管理器或遥测之后 |
| `mission_panel.py --output DIR [--limits-only\|--telemetry-only]` | RViz 面板命令看门狗（假管理器，无 Gazebo）；完整模式要等真实的 90 s 超时 | RViz（`ssb_runtime.sh --gui`），不能同时开着其他任务 | 修改 RViz 面板或命令协议之后 |
| `mission.py --output DIR [--wall-only --wall-start A --wall-length L] [--viewers none\|gz\|rviz\|both]` | 已安装任务栈：暂停、排空、重启、回放和界面帧率；默认做 20 m 壁面目标 | 资产包、OptiX、Gazebo、RViz（`--gui`）；磁盘余量按采集长度预留 | 发布前；修改任务流程之后 |
| `initial_unroll.py ...` | 只导出公开 ROI 输入，用另一种后端或分块大小复现 D1 | 已完成的采集会话；CUDA（默认）或 `--backend cpu` | 修改 D1 之后 |
| `band_matching.py ...` | 只迁移公开 D1 产物、子进程禁读其他工作区数据时复现 D2 | 已有 D1 结果 | 修改 D2 或读取审计之后 |
| `global_optimization.py ...` | 用迁移后的公开产物和原图块复现 D3，不读真值资产 | 已有 D1/D2 结果 | 修改 D3 之后 |
| `wall_coverage_scale.py --reference R --calibration C --output DIR` | 20 m / 240° 覆盖规划在合成公开传感器表上的资源回归（不是真实采集） | 参考会话的 `observable_config` 与标定 | 修改壁面覆盖规划之后 |

## 已知状态

- `gazebo_plugins.py`：2026-10-06 在未改动的 `main`（`6eba2ab`）和修复分支上都在 `irregular` 工况失败（`irregular track must move the body`，车体 z 峰峰值 0.130 mm，阈值 0.2 mm），其余 16 个工况通过。该脚本上次运行是 10-04，原因见修复计划 6.3 的记录。
- 其余脚本本次只验证了能导入、能解析参数（`--help`），没有完整运行；各自的耗时没有测量。
