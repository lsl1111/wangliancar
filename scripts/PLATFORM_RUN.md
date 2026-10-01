# 在 NEVCsim 中运行当前整车链路

平台控制器仍使用这两个文件：

| 控制器字段 | 当前电脑的绝对路径 |
| --- | --- |
| 启动脚本 | `F:\wangliancar\nevc_auto\scripts\StartCaptain.bat` |
| 结束脚本 | `F:\wangliancar\nevc_auto\scripts\KillCaptain.bat` |

`StartCaptain.bat` 启动项目的 `main.py`，依次执行感知、决策、规划、控制并由运行层发送。因此不需要把决策或控制模块的 Python 文件单独填入平台。已有控制器路径相同，就无需重新填写；下一次启动会读取磁盘上的更新代码。已经运行的队长进程需要先结束，才会加载新代码。

## 实跑步骤

1. 结束上一轮仿真，确认控制器的结束脚本执行。必要时手动运行 `KillCaptain.bat`。同一项目同时启动第二个进程会被单实例锁拒绝。
2. 在平台“资源库 → 控制器”核对上述开始、结束脚本路径，保存控制器；修改平台控制器后按平台要求刷新本地端。
3. 在案例页选择该控制器，启动一次**新的仿真运行**。从历史回放中播放不会验证当前代码，也不要同时手动再次启动队长。
4. 查看 `runtime_data\launcher.log`，确认本次启动时间、巡航参数和本次 `captain-console-*.log` 文件名。
5. 查看 `runtime_data\captain.log` 的本次“队长启动”和“决策实现”记录。决策版本应为 `decision-constraints-v4`、引擎为 `DecisionEngine`，当前默认应显示 `cruise_speed=8.333 m/s (30.0 km/h)`。路线修正版另有“路线接续实现 version=successor-continuation-v1”及“前方参考”记录。
6. 查看本次 `runtime_data\latest_pipeline.json`：`runtime` 记录决策版本、路线接续版本、巡航参数、进程号和启动时间；`route_reference` 记录前方车道 ID、状态及点数。再核对来源帧、修改时间、决策、轨迹、控制和 `send`。发送成功应有 `attempted=true`、`ok=true`。旧文件不能证明本次运行成功。

本次接入的是 main 中合并的新规则决策及现有数据合同的兼容修正。巡航参数对所有场景统一生效：平台启动脚本和决策内部默认均为 **30 km/h（30 / 3.6 ≈ 8.333 m/s）**，控制跟踪上限同为 30 km/h。弯道、停车、跟车和安全监护可以要求更低速度；规划基于完整原始地图几何计算弯道约束，避免插值点间距影响曲率限速。若调用者已设置 `NEVC_DECISION_CRUISE_SPEED`，脚本保留该值；这个变量的单位始终是 m/s，不能填入 30 来表示 30 km/h。直接启动 `main.py` 时也采用同一默认巡航。

启动脚本优先加载 `config\local.ini`，不存在时使用共享默认配置。本机覆盖文件不提交；实际是否发送仍由 `send_control` 和 `safety_brake_enabled` 决定，二者都为 `true` 才布防发送。更新脚本不会修改这些开关。

带目标的前进规划需要车头偏移和车身半宽。脚本现按本机用户提供的 SimOne-Car 主车截图传入 `NEVC_VEHICLE_FRONT_OFFSET_M=3.9187`、`NEVC_VEHICLE_HALF_WIDTH_M=0.9`，单位 m；前者为轴距 2.9187 m 加前悬 1 m，后者为车宽 1.8 m 的一半。赛题车辆位置参考为后轴中心，不能把轴距单独当成车头偏移。决策和规划读取同一组参数，`launcher.log` 会记录本次实际值。其他车型由调用环境显式设置这两个变量；脚本保留已有覆盖。不要将单独的 `NEVC_DECISION_FRONT_OFFSET_M` 设成与共用值不同的尺寸。直接运行 `main.py` 不经过批处理，须自行传入相符的环境变量；尺寸校验不是制动能力标定。

控制执行能力及离线覆盖见 [控制动作说明](../members/control/MANEUVERS.md)。当前生产链可在可信目标、车道和尺寸齐全时生成前进障碍停车/跟车轨迹；倒车、泊车、换道仍需上游明确意图和规划路径，不能仅靠脚本切换到对应题目。

实时查看日志（PowerShell）：

```powershell
Get-Content F:\wangliancar\nevc_auto\runtime_data\captain.log -Tail 50 -Wait
```

案例结束后检查同一次运行的日志。离线测试通过不等于平台通过；当前已补唯一后继车道接续，下一轮重点检查进入弯道时前方参考包含后继车道，交界处不再进入 HOLD。`map_end` 表示整条已查明参考最终有终点，并不表示车道交界需要立即停车；应结合 `trajectory.stop_distance/stop_required` 和点速度判断。分叉未选择或连接异常的原因与边界见 [路线接续说明](../perception/ROUTE_CONTINUATION.md)。
