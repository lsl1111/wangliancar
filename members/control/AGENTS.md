# 控制成员约束

继承根 `AGENTS.md`。本文件自然作用于本目录；`members/control_stub.py` 及控制测试由根规则要求补充读取本文件。

- 职责：经 `compute_control(perception, trajectory)` 把 `Perception` 和 `Trajectory` 转换为 `ControlOut`，完成轨迹跟踪；行为意图与路径由上游提供。
- 可修改本目录算法、固定控制入口和相关测试；不修改感知/决策/规划算法，不连接 SDK 或直接发送车辆指令，发送由队长运行层负责。
- 输出前限制 `throttle/brake` 为 `[0, 1]`、`steering` 为 `[-1, 1]`；实现未完成时保持 `valid=False`。有效控制还需验证数值有限、量纲正确和输入可信。
- 与规划成员确认轨迹参考点、速度/时间语义、倒车表达及 `emergency_stop`；明确无效/过期轨迹的处置。不能假定“不发指令”一定能使车辆停车。
- 验证：离线检查限幅、异常输入、控制状态和跟踪方向；获团队授权后按低速简单场景验证真实响应，再扩展场景。
- Skill：可按控制任务选择相关 Skill；本项目 `path-planning` 的安装不授权把控制器实现迁入规划模块。
