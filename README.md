# NEVC 队长部分（API、感知与集成）

这是四人方案中的“队长项目”。它不是整车算法，而是全队共同运行的底座：只在这里连接官方 SimOne API，把原始数据整理成统一接口，然后依次调用决策、规划和控制成员的模块。

## 当前已经做到

- 使用官方 `SoInitSimOneAPI` 连接 SimOne，读取 GPS、传感器目标；传感器不可用时回退到 Ground Truth。
- 加载 HD Map，输出当前车道号、中心线、左右邻车道、车道宽度和车辆相对车道误差。
- 自动从案例名称识别场景编号，例如 `06.车道居中控制-测试` 识别为场景 6。
- 每一帧生成统一的 `Perception`，写入 `runtime_data/latest_perception.json`，方便全队直接查看。
- 已留下决策、规划、控制三个固定入口，并提供离线单元测试。
- 默认 `send_control=false`，所以目前只观察、不抢车辆控制权。

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
3. 日志在 `runtime_data/captain.log`，最新一帧感知数据在 `runtime_data/latest_perception.json`。
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

