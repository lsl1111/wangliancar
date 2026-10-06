# 通用地图语义输入与交接

2026-10-02。此接入按 API 能力和当前车道工作，没有按题号选择地图坐标或构造对象。固定入口、Python 3.6 和单进程调用链保持不变。

2026-10-04：动态信号消费核对灯的完整车道作用范围、所选转向和朝向，发布独立路线停车边界。地图观测目录仍保留原生道路关联信息，不能把目录里的 `association_signal_ids` 直接当作本车适用信号。新 `traffic.signal_groups` 与决策/规划/监护合同见 [信号约束](SIGNAL_CONSTRAINTS.md)。

## 实际数据链

`HDMapAPI → SimOneAdapter/MapObservationReader → PerceptionBuilder → Perception → decide / plan / compute_control`

三个模块收到同一个 `Perception`；`latest_perception.json` 保留全部新增字段，`latest_pipeline.json.map_observations` 记录数量、限速观测和来源状态。这些是环境输入，发送给平台的输出仍为既有 `ControlOut` 与 `SoSetDrive` 结构，不把地图列表写进控制结构。

| API/能力 | 统一字段 | 语义与限制 |
| --- | --- | --- |
| `getTrafficSignList()` | `traffic_signs/valid` | 保留 ID、类型/子类型、值/单位、三维朝向向量及其派生 yaw、位置、车道范围、停止线/斑马线关联 ID。heading 是朝向向量，不能直接 `float()`；零向量保留数据并标记朝向未知 |
| `getParkingSpaceList()` | `parking_spaces/valid` | 世界坐标 m，保留四个边界点顺序、入口边 a-d、朝向向量及各边标线。地图无占用字段，`occupancy=unknown`；不能据此认定车位无障碍 |
| `getStoplineList(signal,lane)` | `map_stop_lines/valid` | 与当前车道关联的边界点、ID 和关联信号/标志 ID；独立于动态灯色。不把灯杆坐标当停止线 |
| `getCrosswalkList(signal,lane)` | `map_crosswalks/valid` | 与当前车道关联的多边形、ID 和关联信号/标志 ID。不表示已检测到行人 |
| 已核对的限速标志 | `speed_limit_observations` | m/s，附沿当前车道距离、适用性、是否已通过及未解析原因。适用且已通过的最近限速写入 `lane.speed_limit`，继而被交通与既有决策读取 |
| 来源范围 | `map_observation_status` | `api/read_ok/reason/invalid_records/errors`，`clock=static_map`、查询范围和覆盖是否完整。缺接口、读取失败、有效空列表和坏记录分别表示 |

限速类型依据安装 SDK 的 `Autopilot/util/GetSignType.h`，仅识别其 `SpeedLimit_Sign=1010203800001413`。数值与 km/h、m/s、mph 等明确单位才可换算；核对道路/车道段/车道编号及标志正面与行驶方向。未知类型、单位、作用范围、动态标志或零朝向不提供有效当前限速。前方标志仅作为带距离的观测；提前减速仍需规划消费该字段，不能据此宣布限速题已通过。当前限速不跨越未核实的车道作用范围；解除限速和其他国家类型尚需核对，不能猜测。

## SDK 绑定的实际边界

比赛命题表 4 允许道路标志、交通标志、车道、轨迹等来自 HDMap API，表 5 给出车位 ID、位置、方向向量和四个边界点。Python API Reference 另列 `SoGetSensorRoadMarkInfo`。文档列出接口不等于当前安装的 Python 模块导出了它。

本机 Python 绑定核对结果：

- 可调用 `getParkingSpaceList()`、`getTrafficSignList()`、`getStoplineList(signal,lane)`、`getCrosswalkList(signal,lane)`。
- `MSignal.heading`、`MParkingSpace.heading` 是三维向量；现已修正旧的标志转换错误，避免整批标志因 `float(vector)` 被清空。
- C++ 头文件另有 `GetSpecifiedLaneStoplineList`、`GetSpecifiedLaneCrosswalkList`，但本机 `HDMapAPI.pyd` 没有对应导出；Sensor Python 模块也无 `SoGetSensorRoadMarkInfo`。
- 因此停止线/斑马线查询范围明确为 `signal_associated_lanes`，`coverage_complete=false`。即使返回空列表，也不能证明地图上没有未关联对象。地面箭头、未关联停止线/斑马线仍需匹配的 SDK 绑定，或另外核实官方地图数据途径后接入。

地图目录每 5 秒刷新；当前车道对象按车道缓存，目录刷新及地图重载清缓存。异常记录不丢弃其他正常记录，但对应目录的 valid 为 false，并保留错误计数。地图缺失不改变 `Perception.valid` 的 GPS 含义，也不增加全场景必需传感器；各模块仍按实际行为核对所需来源。

## 验证和实际运行

`tests/test_map_observations.py` 覆盖三维 heading、关联 ID、坏记录与有效空目录、车位入口和未知占用、无动态灯的关联几何、缓存隔离、限速单位/方向/范围，以及在多个普通车道场景中真实 `decide → plan → compute_control` 入口和 JSON 传递。测试替换 SDK 数据来源，不证明平台正在发布地图或目标数据。

2026-10-02 在本机 SimOne Python 3.6 下执行全量 `python -B -m unittest discover -s tests -v`，361 项通过；另用实际 SDK 三维点类型验证 heading 转换。新增地图字段尚未完成新一轮平台数据验收。

数据验收器按收到的停车位、停止线、限速观测检查结构，不再把这些字段永久标记为不存在。泊车数据结构验收不强制依赖道路车道；生产规划仍要求其既有车道参考，尚未生成自由空间泊车路径。`STRUCTURAL_PASS` 的事件审查仍为 `PENDING`，不能代替车辆动作验收。

正式移动目标仍由 Sensor API 提供；地图语义接入不修复平台未下发传感器配置的任务，也不把 Ground Truth 诊断提升为生产目标源。API 读取与平台任务发布必须分别检查。
