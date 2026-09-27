# NEVC 队长部分（API、感知与集成）

这是四人方案中的“队长项目”。它不是整车算法，而是全队共同运行的底座：只在这里连接官方 SimOne API，把原始数据整理成统一接口，然后依次调用决策、规划和控制成员的模块。

组员开始开发前，请先阅读 [四人协作开发说明](TEAM_COLLABORATION.md)。

## 当前已经做到

- 使用官方 `SoInitSimOneAPI` 连接 SimOne，读取 GPS，并按场景传感器配置读取目标；目标传感器不可用时 Ground Truth 仅作诊断回退。
- 加载 HD Map，输出当前车道号、中心线、左右邻车道、车道宽度和车辆相对车道误差。
- 自动从案例名称识别场景编号，例如 `06.车道居中控制-测试` 识别为场景 6。
- 每一帧生成统一的 `Perception`，写入 `runtime_data/latest_perception.json`，方便全队直接查看。
- 同步写入 `runtime_data/latest_pipeline.json`，记录决策、轨迹、控制和平台发送回执；重复 GPS 帧仍更新快照与各传感器状态，但不会重复发送控制。
- 额外读取任务路径点、传感器配置、环境，以及 SDK 可用时的 IMU、毫米波雷达、超声波与摄像头车道观测；这些数据在 `Perception` 中各自标明来源/有效性，不与目标真值混用。
- 地图车道输出左右边界和标线；交通灯仅在当前车道关联到真实停止线时才输出有效灯色与沿车道的停车距离。
- 已留下决策、规划、控制三个固定入口，并提供离线单元测试。
- 默认 `send_control=false`，所以目前只观察、不抢车辆控制权。
- 运行层另有刹车安全监护：当前场景必需的目标流或车道失效、紧急停车、轨迹/GPS 失效或 GPS 帧停滞时生成零油门刹车候选，并在 `latest_pipeline.json` 记录原因。06 等车道场景没有目标传感器时不因此停车；01 等目标场景仍要求有效的 Sensor API 目标源。只有 `send_control` 和 `safety_brake_enabled` 同时显式开启才会尝试发送；两项默认均为 `false`，候选刹车比例尚待场景标定。
- 来源帧采用本机单调时钟判断停滞、回退和跨传感器帧差；超时及模块输出的来源帧、有效期在传输节点验证。传感器没有帧号时标记 `timing_unknown`，不伪称同步。

## 四个人怎样接入

数据流只有一条：

```text
SimOne API -> 队长 Perception -> 决策 DecisionTarget
           -> 规划 Trajectory -> 控制 ControlOut -> 队长调用 SoSetDrive
```

三个成员只需要改各自文件中的一个函数：

| 成员 | 文件 | 函数 | 输入 | 输出 |
|---|---|---|---|---|
| 决策 | `members/decision_stub.py` | `decide(perception)` | `Perception` | `DecisionTarget` |
| 规划 | `members/planning_stub.py` | `plan(perception, decision)` | 前两级结果 | `Trajectory` |
| 控制 | `members/control_stub.py` | `compute_control(perception, trajectory)` | 感知和轨迹 | `ControlOut` |

接口字段全部在 `core/interfaces.py`。三个成员不要直接 import SimOne SDK；SDK 字段变化只由队长在 `simone_platform/simone_adapter.py` 中处理。

## 运行方法

1. 在 SimOne 中选择并启动案例。
2. 双击 `scripts/StartCaptain.bat`。
3. 日志在 `runtime_data/captain.log`；感知和整条链的最新快照分别在 `runtime_data/latest_perception.json`、`runtime_data/latest_pipeline.json`。
4. 停止时按 `Ctrl+C`，或运行 `scripts/KillCaptain.bat`。

只验证 SDK 能否加载，不连接案例：

```bat
E:\Sim-One\Tools\python36\python.exe main.py --self-test
```

只读取一帧后退出：

```bat
E:\Sim-One\Tools\python36\python.exe main.py --once
```

## 开启车辆控制前必须完成

控制成员需要让 `compute_control()` 返回 `valid=True` 的 `ControlOut`，全队联调确认油门、刹车和转向量纲后，再把 `config/default.ini` 中的 `send_control` 改为 `true`。当前占位控制始终无效，即使误开开关也不会发控制指令。

## 感知数据的含义

- `Perception.valid` 表示 GPS 自车基础状态可用；目标、车道、交通、路线及辅助传感器要分别查看各自的 `valid/status`。成功读取但没有目标与接口失败不同。
- `source_status` 记录每个来源的 `read_ok`、帧号、本机重复帧持续时间、质量与 `usable`。`targets_valid=False` 表示目标读取失败、损坏、过期或不同步；空 `targets` 不能单独解释为“道路无车”。
- `source_status.targets.sensor_presence` 区分 `configured`、`not_configured`、`unknown`；`sensor_read_ok` 记录目标 Sensor API 是否读取成功。`not_configured` 在不要求目标流的场景仅作状态记录，未知场景保守处理。
- 本机 IMU 独立结构没有帧号；读数正常时 `imu_valid=True`，同时 `source_status["imu"].quality="timing_unknown"`、`usable=False`，需要同步状态的算法不能据此假定它与 GPS 同帧。
- `Target.same_lane` 只在地图车道归属已核实时有效；`same_lane_valid=False` 时可参考 `lateral_band_match`，但它只是横向距离筛选。车道限速缺少可信来源时保持 `-1`，交通灯方向不明确时保留候选列表并标记 `ambiguous`。
- `ego.acceleration` 是沿当前车头方向的有符号加速度（m/s²）；`ego.speed` 是水平速度大小（m/s），不表示倒车方向。`ego.age_ms` 和 `targets_age_ms` 是本进程从首次看到该帧起计算的时间，`-1` 表示未知。
- `route_points` 是二维案例任务路径提示，`route_waypoints` 另保留原始点序号和朝向四元数；两者都不是规划成员输出的 `Trajectory`。`traffic.stop_line_distance` 是沿当前车道中心线到真实停止线的距离，参考点为 GPS 位置；没有可靠停止线时交通字段保持无效。
- 本机 Python 地图模块未提供可直接调用的车道限速接口，所以 `traffic.speed_limit=-1` 仍表示未知。图像、点云、V2X 原始流需要具体场景需求和独立处理链，不会自动进入当前结构化感知快照。
- 41 场景的数据需求、必需 API 与现场验收步骤见 [场景数据验收清单](perception/SCENE_DATA_ACCEPTANCE.md)；只读采样命令为 `python scripts/accept_scene.py --scene 6 --frames 20 --timeout-sec 30`。报告的 `STRUCTURAL_PASS` 仅表示帧结构通过，事件仍需现场核对。
- `DecisionTarget`、`Trajectory`、`ControlOut` 通过 `frame_id`、`timestamp`、`valid_until` 关联同一帧；`valid_until` 是本机单调时钟截止时间，不能跨进程持久化复用。当前规划只输出车道中心线前方的参考预览，控制入口仍返回无效，不代表已经具备整车闭环能力。
