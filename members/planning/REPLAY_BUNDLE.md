# 离线数据验收与规划重算

2026-10-05，基于 main `cbb80e4`。工具为 [replay_bundle.py](replay_bundle.py)，只读取文件并调用当前固定 `plan(perception, recorded_decision)`；不运行决策恢复、控制、监护、车辆模型或评分。

Owner 在 [PR #11 的回复](https://github.com/lsl1111/wangliancar/pull/11#issuecomment-5950999034) 明确建议先处理正式目标源与全过程记录，规划侧消费和回放。新附件中 15 个列出文件的 SHA256 校验一致，但实际是旧版失败证据/单帧摘要、空测量模板和待批准草稿：连续帧、事件、响应及成绩文件位置都未交付。本工具完成规划侧接收准备，不表示 R01–R04 已验收，也不批准 R05/R07 公共接口。

## 输入格式：文件交接建议

下列结构是独立的离线文件格式草案。队长维护运行层采集，当前没有新增自动全帧记录器；公共 `Perception/DecisionTarget/Trajectory/ControlOut` 不增加字段。

- `manifest.json`：明确 `session_id`、数据种类/录制种类、录制代码版本、SDK 原始时间单位、车辆/传感器/任务证据；提供完整的实际 `planner_settings` 和 `source_quality_config`（`sensor_timeout_ms/max_sensor_frame_gap`）。完整设置可以从当前 `PlannerSettings` 的属性字典获取，不能用未知实测值填工程默认值。
- `frames.jsonl`：每行保留同循环 `perception/decision/trajectory/control/safety/send` 原序列化结果，以及 `session_id`、连续 `record_index`、`relative_time_s`、`capture_monotonic_s` 和 `recording_errors`。未执行或没有记录的阶段显式写 `null`，不能用空有效输出补齐。
- `events.json`：事件列表，每项包括 `event_id/start_relative_time_s/end_relative_time_s`，另由提供方标注对象、预期行为、实际结果及来源。本工具只检查时间区间覆盖，不自动给行为或成绩结论。

`capture_monotonic_s` 必须属于原记录进程、与原 `valid_until` 同一时钟，manifest 明确 `capture_clock="process_monotonic_seconds"`。它不是 SDK 仿真时间。建议在规划输入可用时记录，并注明采集位置/时序；工具按所记录的捕获时刻重算，捕获晚于原实际调用时可能已经过期，不能据此倒推原调用状态。

相对秒数用于排序和事件范围；SDK 时间戳保留原整数，不凭格式猜单位，也不要求异步目标时间等于 GPS 时间。原始包与报告放 `runtime_data/` 或团队共享位置，不提交原始运行记录、SDK、凭据或平台完整配置。

## 怎么运行

在仓库根目录、已确认的 SimOne Python 3.6 环境执行：

```powershell
python -B -m members.planning.replay_bundle --manifest runtime_data/handoff/manifest.json --frames runtime_data/handoff/frames.jsonl --events runtime_data/handoff/events.json --report runtime_data/planning_replay/report.json
```

也可以先用 [合成样例](examples/README.md) 检查工具入口：

```powershell
python -B -m members.planning.replay_bundle --manifest members/planning/examples/replay_manifest.synthetic.json --frames members/planning/examples/replay_frames.synthetic.jsonl --events members/planning/examples/replay_events.synthetic.json --report runtime_data/planning_replay/synthetic_report.json
```

示例故意包含未执行控制和一个过期输入，因此写出报告后返回 **1（交付不完整或输入需审查）**。返回 0 也仅表示结构检查完成，报告标为 `structural_only`；返回 2 表示文件/参数读取失败。默认最多检查 10000 行，超限明确报告未检查剩余记录，可用 `--max-records` 显式调整。报告路径不能覆盖输入文件。故障事件中真实源失效也会列出；它可以是正确记录的故障，不据此断言采集器失败，需结合事件标注复核。

## 报告分别说明什么

| 报告内容 | 含义与限制 |
| --- | --- |
| `delivery`、逐行 `issues` | 缺失输出、错误字段、会话/索引/时间回退、错帧、截止延长、原过期、未执行阶段；记录保留，不丢掉故障帧 |
| `gps_source_usable/formal_sensor_usable` 与各自来源问题 | 分别核对 GPS 及必需正式目标流：实际 ID、读取/验证/质量、顶层与来源状态冗余证据一致、年龄及帧差门槛；坏 GPS 或必需目标源禁止重算，`ground_truth` 永远是诊断参考。字段通过仍需独立平台证据，不证明该数据真实采集 |
| `original_input_remaining_s` | 原截止减原捕获时钟；缺时钟或预算不正时禁止重算。重算仅搬移原剩余寿命，不增加 TTL，也不改来源、年龄、帧号或 SDK 时间 |
| `gps_event` | 重复/回退仍保留，每行单独分类。组件重算无决策/运行状态机，不证明这些输入在运行层应获准执行 |
| `replay` | 当前规划入口的有效性、保护请求、原因、点数、实测初速度及停车边界。它是重新计算结果，不复制已录轨迹，不证明控制已执行 |
| `events.coverage` | 是否有记录覆盖事件区间；`covered` 也不证明每次循环都记录、全过程业务完成或采样率合格 |
| `claims` | 整车现场验收、数字评分始终 `not_evaluated`；全链回放未执行、连续采集未被独立证明 |

完整显式规划设置使重算不受本机环境变量影响。公共类的未知字段会报告版本/结构不匹配，不悄悄删除；核心决策目标、原始挡位或案例/任务上下文缺失时，禁止用默认值制造观测。同一会话的任务切换及采集时钟回退会报告并阻止该行重算。`null` 的观测保持未知，包括缺目标朝向，不变成 0 rad。原采集错误、各组件的 valid/reason/errors 及监护/发送事实保留于 `recorded`，与新规划结果分开。过期帧仍保留在报告中，不因补了模拟时间而复活。

## 验证与后续交接

[tests/test_planning_replay.py](../../tests/test_planning_replay.py) 覆盖时钟/来源/同帧合同、重复与回退、异步目标时间、缺失和矛盾证据、未知字段/未执行输出、设置冻结及输入不变。离线通过只证明文件消费与组件检查逻辑；收到新包后应另外核对真实采集配置、车辆响应、完整事件和成绩。

本轮在已确认的 SimOne Python 3.6 执行 `python -B -m unittest discover -s tests -v`：486 项中 484 项通过、2 项跳过，耗时 77.809 秒；新增回放回归 22 项全部通过。跳过的是既有启动/结束批处理测试，其门槛固定要求本机 `E:\Sim-One`，当前安装位于 `D:\MY_PROGRAMME\Sim-One`。没有连接 SDK、发送车辆指令或进行场景/评分验收。合成 CLI 样例共 3 行，2 行重算有效、1 行原输入过期，报告 `incomplete`、返回 1，符合例子设计。

下一步由队长交付同循环连续包或确认外层记录格式，规划用本工具检查并定位失败帧；有具体规划失败再修改算法。反馈与泊车合同按 [R01–R09 请求](DATA_INTERFACE_REQUEST.md) 继续协调。owner 的 PR #16 仍在完善规划链路，本轮保留主分支现有算法，避免重复其未合并修复。
