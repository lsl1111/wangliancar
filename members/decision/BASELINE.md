# 决策基础版与速度修正

2026-09-30 实现 V1；2026-10-01 修正跟车起步、限速与停车仲裁。当前交付是**可离线验证的行为选择与速度/停车需求**；仍需平台场景和车辆参数验证。

> 当前运行入口与数据链约定见 [本地接入说明](INTEGRATION.md)。场景目标流是否必需、原始档位与逐帧计数以接入实现为准。

默认巡航速度沿用主分支已确认的 30 km/h（约 8.333 m/s）；平台脚本与控制跟踪上限未由本次修改调整。

## 入口与范围

- 固定入口仍是 [`decide(perception)`](../decision_stub.py)，薄封装负责持有模块级 `DecisionEngine` 实例并转发。
- 算法在 `members/decision/`：`settings.py`（参数）、`protocol.py`（感知归一化）、`candidates.py`（目标筛选）、`speed_policy.py`（速度需求）、`engine.py`（行为仲裁）。
- 输入输出沿用 `Perception -> DecisionTarget`，**不新增公共字段，不改感知、规划、控制或运行循环**。
- 只输出行为目标（模式、目标速度、目标车道、停车距离），不输出油门/刹车/转向，不直接调用 SDK。
- Python 3.6，仅标准库。

## 与旧占位实现的差异

旧 `decide()` 在无目标可用时一律 `KEEP_LANE` 并把目标速度设为**实测车速**，因此静止时永远停着。V1 的行为变化：

| 项 | 旧占位 | V1 |
| --- | --- | --- |
| 巡航速度 | 实测车速 | 参数化巡航速度，受可信限速压低 |
| 静止起步 | 不会起步 | 会起步（目标速度不再等于实测车速） |
| `FOLLOW` | 声明未使用 | 同车道目标会产生 `FOLLOW` |
| 目标观测失效 | 视为无障碍继续行驶 | 连续失效后锁存盲停 |
| 未经地图核实的近距目标 | 按 `lateral_band_match` 可能直接放行 | 距离够近即停车并说明原因 |

V1 仍**不实现**变道、倒车、路口通行和绕障。

## 行为判定顺序

按优先级从高到低；红灯与障碍物同时要求停车时，采用更近的可测停车点：

| 顺序 | 条件 | 输出 |
| --- | --- | --- |
| 1 | 自车帧不可用、数值非法或过期 | `EMERGENCY_BRAKE`，目标速度 0 |
| 2 | 车道几何不可用 | 在动：`EMERGENCY_BRAKE`；已静止：`STOP` |
| 3 | 场景必需的目标观测连续两帧不可用，盲停已锁存 | 在动：`EMERGENCY_BRAKE`；已静止：`STOP` |
| 4 | 同车道目标进入紧急包络 | `EMERGENCY_BRAKE`，停车距离 0 |
| 5 | 交通控制要求停车 | `STOP`；已知更近障碍物时提前停车；停止线未知且在动时紧急制动 |
| 6 | 场景必需的目标观测单帧不可用 | 暂不加速，也不使用该帧目标 |
| 7 | 同车道目标但尺寸未知 | `STOP`，沿用原有最小车距退化值 |
| 8 | 同车道目标 | `FOLLOW`，按间距、限速与制动包络给速度 |
| 9 | 距离够近但未经地图核实的车道外目标 | `STOP`，给出已知的保守停车距离 |
| 10 | 其余 | `KEEP_LANE`，巡航速度（受可信限速压低） |

地图已核实属于其他车道的目标不触发第 9 项。

### 关键判定细节

**紧急包络**：间隙 ≤ `emergency_clearance`，或（自车速度 ≥ `emergency_clearance/emergency_ttc` 且 TTC ≤ `emergency_ttc`）。TTC 规则在低速下**主动失效**——静止时 TTC 没有意义，若不失效，车一想起步就会因为"距离 4.5m、TTC 很小"被判紧急，永远起不来。这是 V1 修掉的一个起步自锁。

**跟车速度**：前车速度不再是硬上限。距离足够远时，可在巡航速度内向慢车接近；距离变小时，速度需求按间距误差下降。对静止前车，只有留有足够停车距离才给正速度：

```text
期望间距 = min_gap + time_headway * 前车速度
间距速度 = max(0, 前车速度 + (实际间隙 - 期望间距 - resume_margin) / time_headway)
制动上界 = sqrt(前车速度² + 2 * follow_deceleration * max(0, 实际间隙 - min_gap))
目标速度 = min(可信巡航/限速, 间距速度, 制动上界)
```

`follow_deceleration` 是未标定的需求侧假设，不能保证实际车辆能按此减速；规划和控制仍要各自验证可行性。

**车道归属**：只有 `same_lane_valid=True` 且 `lane_id` 与当前车道一致才算"同车道"。`same_lane` 是队长的横向距离带兜底判断，**不作为车道结论**；据此丢弃目标会把真障碍当成邻道车。未经核实的目标在距离 ≤ `min_gap` 时停车并说明，而不是按横向距离自行决定。

**盲停锁存**：只有场景必需目标流时，失效的不同 GPS 帧累计到 2 帧才锁存；单帧失效不加速。恢复要求目标流重新可用且自车速度降到 `blind_speed_tolerance` 以下。车道场景的可选目标流缺失不阻止行驶；来源 `usable=False` 的空列表始终不能当作已证实的无车道路。

## 参数（`DecisionSettings`）

全部是**未标定的实验值**，不是车辆能力或赛事阈值。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `cruise_speed` | 30 / 3.6 ≈ 8.333 m/s | 无可信限速时的期望速度，当前默认 30 km/h |
| `min_gap` | 4.0 m | 静止跟车最小车距 |
| `time_headway` | 1.2 s | 跟车时距，同时用作接近速度的时间常数 |
| `resume_margin` | 2.0 m | 接近前车前保留的额外间距 |
| `follow_deceleration` | 2.0 m/s² | 跟车接近的暂定制动上界 |
| `emergency_clearance` | 2.0 m | 紧急包络的间隙阈值 |
| `emergency_ttc` | 2.0 s | 紧急包络的 TTC 阈值 |
| `launch_ttc_cap` | 6.0 s | 历史保留参数；当前紧急判据未读取，勿用于现场调参 |
| `stop_margin` | 3.0 m | 通用停车余量（当前无调用点，保留给控制联调对齐） |
| `obstacle_stop_margin` | 3.0 m | 障碍物停车余量 |
| `traffic_stop_margin` | 3.0 m | 停止线停车余量 |
| `blind_speed_tolerance` | 0.5 m/s | 盲停解除与紧急制动判定的速度阈值 |

`cruise_speed` 在**没有可信限速来源**时是唯一的速度来源，它直接决定赛题能不能跑完。联调前必须和赛题要求对齐。

### 现场改参数不需要改代码

所有参数都可以用环境变量覆盖，因此**不需要动队长维护的 `config/default.ini`**：

```powershell
# 单个参数
$env:NEVC_DECISION_CRUISE_SPEED = "11.1"

# 多个参数
$env:NEVC_DECISION_TIME_HEADWAY = "1.5"
$env:NEVC_DECISION_MIN_GAP = "5"
python main.py
```

变量名规则为 `NEVC_DECISION_` + 参数名大写。非法值（非数字、非有限、负值、`cruise_speed=0`）会**直接报错退出，不会静默忽略**——静默忽略会让现场以为改了参数其实没改。

优先级：代码显式传入 > 环境变量 > 内置默认值。

## 与规划、控制的接口语义

- `stop_distance` 按公共协议解释为**沿车道中心线从当前 GPS 投影到停车点的距离**；`-1` 表示未知。V1 的余量在决策侧扣除，规划侧对 `STOP` 模式**不再重复扣减**。这个责任划分是**暂定**的，需与规划组确认。
- `target_speed` 是标量需求，实际执行速度由规划安排在每个轨迹点的 `speed` 中；规划可因弯道或地图末端低于这个标量。
- `FOLLOW` 当前会被规划**拒绝**：`lane_planner.py` 在目标列表非空时直接返回 `valid=False`。因此 V1 的跟车行为在规划接入避障之前不会形成有效轨迹。这不影响安全性（无效链条不会发控制），但意味着**跟车能力目前无法闭环验证**。
- 目标尺寸未知时决策输出 `STOP`，规划对 `STOP` 需要非负 `stop_distance`，而 V1 在这种情况下给出的是 `min_gap`，可以形成有效轨迹。

## 验证与范围

离线测试在 `tests/test_decision.py`（47 个用例）与 `tests/test_replay_decision.py`（9 个用例，覆盖快照回放工具）。`test_decision.py` 覆盖无效/过期/非有限感知、起步与限速、交通灯（含灯组方向未解决）、紧急包络边界、跟车起步、限速及制动包络、车道归属未核实、盲停锁存与解除、参数环境变量覆盖与校验、输入不变性、输出帧继承与可序列化、规划互操作。

```powershell
python -B -m unittest tests.test_decision -v
python -B -m unittest discover -s tests -v
```

### 用真实快照离线回放

`scripts/replay_decision.py` 可以把 `latest_perception.json` 或一组历史快照喂给决策引擎，统计行为分布和原因。**只读**，不连接 SimOne：

```powershell
python scripts/replay_decision.py --snapshot runtime_data/latest_perception.json
python scripts/replay_decision.py --directory runtime_data\scene_acceptance --summary-only
python scripts/replay_decision.py --directory runtime_data\scene_acceptance --revive-ttl 5 --json-out report.json
```

它回答的是合成单测答不了的问题：真实数据里**有多少比例的目标真正通过了地图车道核实**、目标尺寸是否发布、行为树是否塌缩成单一行为。

⚠️ `valid_until` 是进程内的单调时钟截止时间，所以保存下来的快照**必然是过期的**，默认运行时引擎会在新鲜度门控处停下，每帧都输出同一种行为。这是**安全规则在生效，不是决策故障**。要观察完整行为树需要加 `--revive-ttl`，它会把每帧有效期重置为"当前时刻 + N 秒"——这会**破坏新鲜度语义**，脚本会打印显著警告，其输出只能用于看行为选择，**不能作为实时数据结论**。

**验证范围限制**：本轮在 Python 3.13 上运行 `python3 -B -m unittest discover -s tests -q`，共 241 项，240 项通过、1 项跳过；修改文件通过 Python 3.6 语法解析。本机没有 SimOne Python 3.6 解释器，尚未在目标解释器或 SimOne 场景复测。离线通过不代表闭环跟车能力。

## 待协作确认

| 优先级 / 责任方 | 需要确认 | 当前影响 |
| --- | --- | --- |
| 联调前：规划组 | （已确认）跟车速度来源就是决策的 `target_speed`；本版已给出 | 规划接入避障前 `FOLLOW` 无法形成有效轨迹 |
| 联调前：规划组 | `STOP` 停车余量由决策扣还是规划扣 | 当前由决策扣，重复扣会停车过远 |
| 联调前：控制组 | `FOLLOW` 目标速度跟踪及紧急制动的场景表现 | 控制实现已接入，带目标路径尚未闭环验证 |
| 联调前：队长 / 赛题 | 30 km/h 巡航在各场景的时间与安全表现 | 已设为默认速度，仍需场景验证 |
| 联调前：队长 | 场景 06 连续 `Perception` 快照 | 尚无真实数据检验目标尺寸、`same_lane_valid` 比例与交通灯关联 |
| 障碍物阶段：感知 | 目标尺寸语义与漏检边界 | 尺寸缺失时决策保守停车，可能影响可用场景范围 |
| 换道阶段：感知 / 队长 | 邻道与后继车道完整几何 | V1 不变道；`target_lane_id` 保持当前车道 |
