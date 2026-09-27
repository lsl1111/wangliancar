# NEVCsim 场景数据验收与必需 API 清单

2026-09-27，范围仅为 `nevc_auto` 的结构化感知输入与对决策、规划、控制公开的 `Perception`。依据是赛题《命题与运行》物理页 2–7、用户提供的 Python API Reference 1.1 物理页 2–24、本机 SDK Python 绑定及当前实现。资料中的示例不是本项目运行指令。**接口存在、代码已调用、真实场景有数据、事件被正确识别是四个不同结论。**

## 当前验收结论

当前主机尚无正在运行的 NEVCsim/SimOne/BridgeIO，也没有历史 `latest_perception.json`，所以 41 场景均为 **待现场数据**，没有一个场景可以宣称通过。离线结构与字段检查见 `DATA_CHAIN_AUDIT.md`。`scripts/accept_scene.py` 已提供只读收集与结构性检查；其 `STRUCTURAL_PASS` 只证明采到连续、可用的数据帧，仍须看事件期间的目标分类/位置、车道关联及动作结果。

官方赛题的输入边界：移动目标来自**限定探测范围的 Sensor API**；道路/交通标志、车道、轨迹来自 HDMap API；信号状态来自交通灯 API。GPS 的地图坐标参考点为**后轴中心**，而 `traffic.stop_line_distance` 当前按 GPS 位置计算；停车点若要相对车头，必须另取车身前悬长度并现场标定。不能把灯杆距离充作停止线距离，也不能把目标回退到全场景 Ground Truth 后仍称传感器探测范围已验收。

## 必需输入/API 与实现状态

“必需”指相关场景需要的**数据能力**，不是要求同时调用表中所有备选 API。`已接`只说明代码调用并保留字段；现场列全部待验。

| 数据能力 / 典型场景 | 官方/本机 API | `Perception` 出口 | 当前状态与现场核对 |
| --- | --- | --- | --- |
| 自车后轴位置、姿态、速度、加速度；全部场景 | `SoGetGps` | `ego`、`source_status.gps` | 已接；查坐标、航向单位、连续帧、新鲜度，坡道 22 查 pitch |
| 范围内目标及类型/运动/尺寸；01–03、11–18、21–28、30–41 | `SoGetSensorConfigurations` + `SoGetSensorDetections(vehicle_id, sensor_id)` | `sensor_configurations`、`targets`、`target_source`、`targets_valid`、`source_status.targets` | 优先使用配置 ID，缺失时选择配置中首个摄像头/激光雷达/融合/Perfect 目标源；无合适配置标记 `not_configured`，配置未知标记 `unknown`。现场核探测范围、遮挡、稳定 ID 和空帧有效性 |
| 目标回退，仅诊断 | `SoGetGroundTruth` | `target_source=ground_truth` | 已接；它是全场景物体真值，不能以它证明限定范围的 Sensor API 正常。若现场出现，目标传感器项不通过；竞赛使用边界需按赛方规则确认 |
| 车道中心、边界、宽度、邻道、连通、标线；04–06、16–20、26–32、41 | `loadHDMap`、`getNearMostLane`、`getLaneSample`、`getLaneWidth`、`getLaneLink`、`getRoadMark` | `lane`，目标 `lane_id/same_lane_valid` | 已接；现场核左右方向、地图与车身偏差、弯道/路口切换、实虚线及目标车道归属 |
| 传感器车道观测；视觉关卡或地图互证 | `SoGetSensorLaneInfo` | `sensor_lane_observations/status` | 已按传感器类型接；不能与 HDMap 几何混作同一源，现场核传感器配置及坐标系 |
| 路口灯色、倒计时、当前车道停车线；10、路口/连续场景 | `getTrafficLightList` + `getStoplineList` + `SoGetTrafficLights` | `traffic`、`traffic_source` | 已接；只有单一车道停止线信号组才 `valid`。同线多方向灯仍 `ambiguous`，不能据列表顺序选绿灯 |
| 独立停止线/斑马线/地面箭头；03、18、20、30–38、41 | 文档 `SoGetSensorRoadMarkInfo`；本机 HDMap `getStoplineList`、`getCrosswalkList` | **没有独立语义出口** | 缺口：本机 Python Sensor API 无该函数；HDMap 函数可导入，但当前 `getStoplineList` 仅随灯查询，不能覆盖无灯停止线。需核查询签名/地图覆盖后新增结构化字段 |
| 限速标志与适用路段；09、41 | `getTrafficSignList`（类型、值、单位、validities）；本机 Python HDMap 无直接 lane-speed-limit 函数 | `traffic_signs` 原始列表；`lane/traffic.speed_limit=-1` | 原始已接，**未把数值/单位/适用车道与当前行驶方向解释成有效限速**；09 未具备验收条件 |
| 泊车位边界、朝向与近距障碍；07–08 | `getParkingSpaceList`（本机 Python 绑定可导入，地图内容待实调）；`SoGetUltrasonicRadars` | 超声已接；**泊车位无出口** | 07/08 缺口；需查车位几何、ID、占用与尺寸，超声是否配置且返回有效数据 |
| 案例任务路径；41 与跨路口路径 | `SoGetWayPoints` | `route_points`、`route_waypoints`、`route_valid` | 已接；路径点是提示，不是规划轨迹。现场核顺序、起终点、坐标和路线覆盖 |
| IMU、雷达、超声、环境；按实际传感器配置 | `SoGetImu`、`SoGetRadarDetections`、`SoGetUltrasonicRadars`、`SoGetEnvironment` | 独立原始观测及状态 | 已接条件读取。IMU 无帧头，`timing_unknown` 不等于同帧；不能用这些流代替目标 API 正常性 |
| 案例和平台身份 | `SoAPIGetCaseInfo`、`SoGetCaseRunStatus`；版本/车辆状态接口可作为诊断 | `case_name/case_id/task_id/scene_id` | 案例已接；验收时必须核实际场景号不依赖手工假定 |

图像/点云 Streaming、V2X、所有回调接口**不列为本轮通用必需项**。若所参加的赛段要求视觉识别，或者场景明确配置上述输入，需另开原始流与时间同步验收；不能从目前的结构化目标列表推断它们已接。

## 41 场景逐项数据验收矩阵

公共条件 `B`：案例 ID/名称、GPS 连续帧及新鲜度、HDMap 车道。目标源只在标有 `T` 的场景中为硬条件；无目标传感器配置时，`targets_valid=False` 不阻断 06 等车道场景。目标场景中空列表仍须先确认 `targets_valid`、Sensor API 来源和配置。`T`：目标类型/相对位置/速度及进入退出事件。`L`：车道中心/边界/宽度/标线/连通。`I`：路口车道连通、冲突目标、独立停止线/人行横道（存在时）。`S`：信号灯与本车车道/方向、实际停止线。`P`：泊车位。`V`：限速标志的数值、单位和适用范围。`R`：任务路线。数据能力“缺口”不等于该题算法已经失败；它意味着现接口无法完整证明该题所需感知。

| 编号 | 场景 | 必核数据 | 当前特别缺口/证据 |
| --- | --- | --- | --- |
| 01 | 前方车辆静止 | B,T | 目标静止/距离/同车道、停车参考点 |
| 02 | 前方车辆制动 | B,T | 目标减速度与帧连续性 |
| 03 | 前方行人横穿 | B,T,I | 行人类型/横向速度；人行横道独立出口缺 |
| 04 | 直道车道偏离抑制 | B,L | 左右偏移符号与道路边界 |
| 05 | 弯道车道偏离抑制 | B,L | 弯道中心线连续和航向误差 |
| 06 | 车道居中控制 | B,L | 第一现场基线：连续帧、中心/边界/宽度、偏移与航向 |
| 07 | 垂直泊车 | B,P | 泊车位几何出口缺，超声需现场配置核对 |
| 08 | 平行泊车 | B,P | 同上，需倒车时后轴/车身参考点 |
| 09 | 限速标志识别及响应 | B,V | 有原始标志，无适用限速值 |
| 10 | 机动车信号灯识别及响应 | B,S | 多方向同线灯仍不确定 |
| 11 | 系统无法处置的场景 | B,T | 无法处置事件与目标源失效必须区分 |
| 12 | 自动紧急避让 | B,T,L | 突发目标时序与可行避让空间 |
| 13 | 前方障碍物起步 | B,T | 障碍物是否移开、静止变动态 |
| 14 | 稳定跟车 | B,T | 前车稳定 ID、相对速度、车距 |
| 15 | 弯道内跟车 | B,T,L | 弯道目标车道归属 |
| 16 | 避让障碍物变道 | B,T,L | 邻道目标、边界实虚线 |
| 17 | 避让低速行驶车辆变道 | B,T,L | 低速目标、邻道可用性 |
| 18 | 无信号灯路口车辆冲突通行 | B,T,I | 无灯停止线独立出口缺 |
| 19 | 车道线识别及响应 | B,L | 地图标线与观测标线按来源核对 |
| 20 | 停止线识别及响应 | B,I | 无灯停止线独立出口缺 |
| 21 | 左侧车辆通行起步 | B,T | 左侧目标相对位置/速度 |
| 22 | 上坡-下坡路跟车 | B,T,L | GPS pitch/z/纵向加速度 |
| 23 | 跟车时前车切出 | B,T,L | 目标 ID/车道归属变化 |
| 24 | 跟车时邻车道车辆切入 | B,T,L | 切入前后同一目标 ID |
| 25 | 停-走功能 | B,T | 目标与自车零速/再起步时序 |
| 26 | 避让故障车辆变道 | B,T,L | 故障对象类型/状态是否可区分 |
| 27 | 避让事故车辆变道 | B,T,L | 事故对象类型/状态是否可区分 |
| 28 | 临近车道有车变道 | B,T,L | 邻道目标、空隙和禁止跨线 |
| 29 | 前方车道减少变道 | B,L,R | 地图后继/合流，路径提示 |
| 30 | 无信号灯路口非机动车冲突通行 | B,T,I | 非机动车分类、人行横道/停止线 |
| 31 | 路口车辆冲突通行 | B,T,I,S* | 若有灯须加 S；方向灯歧义待核 |
| 32 | 拥堵路口通行 | B,T,I,S* | 多目标持续跟踪；若有灯加 S |
| 33 | 群体行人通行 | B,T,I | 目标数量、行人分类/人行横道 |
| 34 | 群体非机动车通行 | B,T,I | 目标数量、非机动车分类 |
| 35 | 行人和非机动车通行 | B,T,I | 两类对象共存与各自轨迹 |
| 36 | 行人折返通行 | B,T,I | 同一 ID 横向速度反向 |
| 37 | 行人违章通行 | B,T,I | 非横道区域的目标事件 |
| 38 | 非机动车违章通行 | B,T,I | 非机动车轨迹/冲突区 |
| 39 | 事故工况-对向冲突 | B,T,I | 对向目标与碰撞风险时序 |
| 40 | 事故工况-冲突对象突然出现 | B,T | 首次可见帧、目标突然出现 |
| 41 | 连续赛道 | B,T,L,I,S*,V,R,P* | 逐事件覆盖；`S/P` 仅场景实际含灯/泊车时 |

`*` 为条件项，以现场事件与案例地图确认。矩阵是输入验收，不评价三位成员的决策、规划或控制分数。

## 现场执行与判定

1. 在 NEVCsim 运行 06，并启动 `scripts/StartCaptain.bat`。先确认 `config/default.ini` 的 `send_control=false`；不要用 `--once`，否则不足 20 帧。
2. 从项目根目录运行：`E:\Sim-One\Tools\python36\python.exe scripts\accept_scene.py --scene 6 --frames 20 --timeout-sec 30`。该命令只读队长发布的 JSON，不调用任何控制 API。
3. 保存的原始帧与报告位于被 Git 忽略的 `runtime_data/scene_acceptance/`。`FAIL_OR_INCOMPLETE` 逐条查失败原因；`STRUCTURAL_PASS` 只代表 20 帧基础结构通过，`event_review=PENDING` 仍需观察车道偏差变化及可视地图核对。
4. 后续场景按矩阵采完整事件窗，人工记录目标首次/最后可见帧、分类、相对运动、车道关联、地图几何与仿真画面是否一致。配置/SDK 失败时不可把空目标当“没有目标”。

必须单独解决的实现项：泊车位出口；独立停止线/人行横道/地面箭头出口及本机 SDK 的替代查询；限速值与适用范围；多方向灯关联；Ground Truth 回退的赛事数据边界；GPS 后轴到车头/车身包络的停止几何。这些未解决前，不标记对应场景“感知完整”。
