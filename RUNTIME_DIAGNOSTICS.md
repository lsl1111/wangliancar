# 用运行记录定位停车原因

队长记录的控制候选、发送回执和 GPS 反馈分别表示不同阶段。
尤其是重复帧或发送失败时，本帧没有有效新指令，不能推断之前的刹车已释放。
本次更新只改运行诊断，没有修改决策、规划、控制和监护算法或阈值。

## 先确认记录属于哪次运行

从实际脚本目录读取 `runtime_data/latest_pipeline.json` 和本次
`runtime_data/captain.log`，核对文件修改时间、`runtime.pid/started_at/project_dir`。
`runtime.source` 在取得单实例锁后、连接 SDK 前采集一次，不在控制循环调用 Git：

| 字段 | 含义 |
| --- | --- |
| `status=captured` | 已完成本目录 Git 元数据查询 |
| `git_commit` / `git_branch` | 启动时的提交及分支；分离 HEAD 的分支为 `null` |
| `dirty=true` | 存在 Git 未忽略的修改或未跟踪文件，不能称为该提交的原版代码 |
| `status=unavailable` | 没有 Git、查询失败/超时或目录是其他仓库内的导出副本；提交、分支和修改状态均为 `null` |
| `status=not_captured` | 未经正式 `run()` 启动，例如离线测试直接调用循环 |
| `runtime.sdk_version` | SDK 加载后实际读取的版本；未知时为空字符串 |

目录、提交和修改状态是启动现场记录；运行期间再修改磁盘或切换分支不能更新
已导入的 Python 代码。旧版本快照没有上述字段，需结合启动日志及历史提交检查。
记录不会保存 Git 凭据、远程地址或本机配置内容。

## 再确认哪个阶段要求停车

1. 看 `decision.reason`：目标冲突、信号、输入质量、跟车/停车约束。
2. 看 `trajectory.valid/reason/stop_required/stop_distance` 和 `control.errors/diagnostics`：
   决策允许前进时，规划或控制是否仍有保护条件。
3. 看 `safety.reason/candidate` 和 `send.attempted/ok/reason`：运行监护是否覆盖控制，
   最后发送成功的候选是什么。`gps_frame_repeated` 表示本轮没有重发普通指令。
4. 对照 `perception.ego.speed/brake` 和来源有效性，确认车辆反馈。
   `send.ok=true` 是平台调用成功回执，不是车辆已经执行的证明。

事件快照中的 `braking` 以及 `summary.braking` 同时保留：

| 字段 | 含义 |
| --- | --- |
| `requested` | 本帧有刹车候选、规划停车保护或监护请求 |
| `candidate_available` / `candidate_braking` | 所选候选是否有效、是否含刹车或手刹；候选不可用时后者为 `null`，不等于 `false` |
| `send_succeeded` | 本帧有效候选已尝试发送且回执成功 |
| `last_successful_command` | 最近成功发送指令的帧、来源、时钟和踏板/手刹；此前没有成功发送时为 `null`。发送失败或重复帧不覆盖它 |
| `release_sent` | 最近成功指令有刹车/手刹，本帧成功发送了零刹车且松手刹的指令 |
| `observed_brake/observed_frame_id/observed_valid` | 该帧 GPS 反馈及接口有效标志；反馈在发送前读取，不能据此认定新指令已经执行 |

即时日志和 `runtime_data/braking_events/` 的事件标签为：

| 事件 | 含义 |
| --- | --- |
| `brake_start/change/sample` | 刹车请求开始、选用原因变化、稳定请求两秒采样；可能处于观察模式或发送失败 |
| `brake_command_unavailable` | 原先刹车请求消失，但没有新的有效候选；没有确认释放 |
| `brake_release_requested` | 有零刹车候选，但没有确认从最近成功刹车指令转为成功释放 |
| `brake_release` | 已确认上述发送转换，仍须检查车辆反馈 |

当监护覆盖控制时，事件变化以所选监护原因、候选可用性及发送状态为准，
未采用的控制器错误交替出现不逐次触发事件。完整错误仍在快照及前八帧摘要内。
候选出现/消失、发送成功/失败以及监护原因变化仍会触发记录。

## 离线重算时使用对应节点的时钟

事件顶层 `recorded_monotonic` 是诊断采样时刻。评价保存或文件写入可能较慢，
不能把这个时钟直接当作决策开始时刻。`pipeline_timing` 另保存：

- `decision_started_monotonic`
- `planning_started_monotonic`
- `control_started_monotonic`
- `safety_started_monotonic`
- `send_finished_monotonic`

各节点起始值在调用相应模块前读取。离线组件重算应固定到该模块对应时钟，
保留原始 `valid_until`，不能刷新截止时间来掩盖过期。SDK `timestamp` 的单位
和单调时钟不同；跨进程不可比较原始单调值。新增诊断字段不改变公共数据合同。

事件快照仍最多 80 个/进程，稳定刹车两秒采样，附前八帧摘要。
这是事件证据，不是全程逐帧录像或正式评分验收；连续全链路回放包仍待完善。
写入失败会警告并继续控制循环。本次离线验证不代替新的 SimOne 实跑。
