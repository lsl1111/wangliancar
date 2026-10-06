# 交接输入样例

这些文件配合 [数据与接口请求](../DATA_INTERFACE_REQUEST.md)，供提供方理解所需内容。全部为手写合成示意或空模板，没有 SDK 采集、实车测量或真实成绩。

| 文件 | 对应需求 | 状态 / 用途 |
| --- | --- | --- |
| [current_frame.synthetic.json](current_frame.synthetic.json) | R01、R04、R08 | `perception/decision` 使用当前公共类的字段子集；来源、坐标和有效标记均为虚构，演示微速度前车、前方限速和未知占用车位 |
| [recording_manifest.template.json](recording_manifest.template.json) | R01–R04 | 环境、当前车辆绑定、评分口径、采集文件和事件标注的交付模板；未知值为 `null` |
| [vehicle_response.template.json](vehicle_response.template.json) | R02、R03 | 指令/回执/反馈/停车测量的建议输入列，全部测量值待填，不是运行接口 |
| [protocol_proposals.synthetic.json](protocol_proposals.synthetic.json) | R05–R09 | 待队长批准的反馈、邻道/路线、行为目标、泊车/路口观测讨论样例；未加入公共接口，禁止直接作为生产输入 |
| [replay_manifest.synthetic.json](replay_manifest.synthetic.json)、[replay_frames.synthetic.jsonl](replay_frames.synthetic.jsonl)、[replay_events.synthetic.json](replay_events.synthetic.json) | R04 / 规划侧消费 | 三行手写合成输入与离线计算轨迹；控制未执行，第三行故意过期并标诊断 GT，展示不完整交付报告；不是真实连续事件包 |

2026-10-05 按 main `cbb80e4` 更新当前接口单帧例子：补充已贯通的按目标净距/精确停车、目标所属车道宽度和路线点索引分段。具体字段状态与独立回放格式见 [离线验收与规划重算](../REPLAY_BUNDLE.md)。这里的新 JSONL 外层格式只供文件交接讨论，尚未接入运行层采集器，也不修改公共类。

## 单位、参考点与时间

- 世界位置/路径距离/尺寸为 m，速度为 m/s，加速度为 m/s²，朝向为 rad，单独的事件相对时间为 s；示意世界 x 轴是道路前进方向，y 向左。目标中心与主车 GPS 后轴不是同一参考点，评分口径还需确认。
- 当前 `Target.relative_speed` 是主车速度减去目标沿车头方向的速度；示例主车 2 m/s、前车 5 m/s，因此为 -3 m/s。`source_status` 的正常质量值按现有生产者使用 `ok`。
- `along_distance` 是从主车 GPS 沿车道到标志的距离，不是已扣车头偏移的停车距离。车位四角保留 SDK 的 a/b/c/d 顺序，入口为 a–d；占用未知仍是未知。
- `timestamp` 保留 SDK 原始值，真实交付必须另确认单位；当前帧仅在 `_metadata` 说明虚构时间单位，不能作为真实 SDK 单位证据。
- `valid_until=0.0` 表示示例不能直接在线使用。离线重建时明确采用合成时钟/有效期，并绑定同一链条，保留原来源和质量状态；真实数据的新鲜度仍须按采集时间验收。
- 当前帧只列必要字段；未列出的当前类字段维持默认值，未给 `trajectory/control/safety/send` 不代表这些交付可省略。真实连续包要按请求 R04 保留完整原始序列化结果。

## 已有协议与候选扩展

`current_frame` 中现有类字段以 [core/interfaces.py](../../../core/interfaces.py) 为准；最外层 `_metadata` 是样例说明，不属于 `Perception`。其 `sensor:demo_fusion`、车道/车位 ID 不指向任何实际资源。

`protocol_proposals` 最外层明确为 `not_implemented`；每项名称、状态和结构均是讨论建议，需要队长与成员确认后再修改生产者、消费者、时效校验和测试。文件中有目标位姿不表示已生成轨迹，且自由空间/后方覆盖为不可用，不能发车。

`null`、`unknown`、`false` 需要结合字段解释：`null` 是没有交付，`unknown` 是尚未知道，已确认有效空列表才可表示“本次观测没有对象”。模板及合成文件不能勾选真实场景验收。
