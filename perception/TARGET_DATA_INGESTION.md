# 目标数据接入修正

2026-10-01，接入标识 `sensor-discovery-v2`。范围是队长的 SDK/感知输入和验收工具，成员决策、规划、控制算法及公共类的字段语义保持原约定。

## 问题与证据

01 和 15 的平台运行日志反复出现 `Get GetSensorDetections not found`。15 的快照显示配置查询未成功、默认 ID 为 `perfectPerception1`、目标来源回退为 `ground_truth`。这些只能证明当时没有读到可用的 Sensor API 目标数据，不能据此断言平台没有传感器，也不能把回退真值算成正式目标接入成功。

本机 SDK `3.0.0001` 的 C++ `Service/SimOneIOStruct.h` 与 Python 绑定确认传感器类型包括 Camera=1、LiDAR=2、MMWRadar=3、PerfectPerception=7、SensorFusion=9。原先的目标选择主要按类型名称匹配，且只尝试一个源。配置查询失败和目标 API 返回 false 的原因也没有完整进入常规运行日志。

## 修正后的读取方式

1. 配置查询同时保留成功、空配置、返回 false、API 缺失与异常的原因。配置尚不可用或为空时 0.5 秒重查；已有非空配置时保持 5 秒缓存。
2. 兼容 SDK 数字类型和平台类型名称，从已配置的摄像头、激光雷达、Perfect 或融合源选择目标输入。用户指定 ID 只有属于目标能力类型时才优先；毫米波、超声和定位仍使用各自 API。
3. 注册官方目标检测回调，获得主车实际发布的传感器 ID。仅已核对的 SDK 3.0.0001 布局启用此回调，使用项目内的 228 字节目标项和 58388 字节列表，不修改安装目录或使用旧 Python 回调的错误步长。回调只复制原生数据，不执行成员算法。
4. 优先消费新鲜的回调数据；没有有效回调包时轮询相应源。首个源读取失败、数据损坏、停滞、回退或与 GPS 不同步时，继续尝试其他已知源。失败轮询按 ID 间隔 1 秒重试，回调新数据可以立即恢复。
5. 配置查询未知且尚未收到回调时，只按用户指定 ID 作有限探测，不遍历猜测 ID。实际回调发现的 ID 即使旧包过期也保留，用于重新轮询；旧目标数据不会充当新数据。
6. 读取成功的空列表表示该目标源本帧没有目标，保持有效。确认没有目标能力传感器、读取失败、坏数据、过期和异步都与有效空帧分开。Ground Truth 回退继续只用于诊断。

当前仍输出一个有效目标源，不进行多传感器目标融合或目标运动预测。不同摄像头覆盖范围的组合、目标 ID 关联和融合属于后续感知算法；本次验证的是实际来源、读取完整性和交付状态。

## 查看本次运行

控制器继续填写 `scripts/StartCaptain.bat` 与 `scripts/KillCaptain.bat` 的原路径。结束旧运行后重新启动，新进程会加载更新代码。

`latest_pipeline.json.runtime.target_ingestion_version` 应为 `sensor-discovery-v2`。`captain.log` 新增“目标接入”记录，包含配置原因、候选 ID、实际选择、来源、读取方式和每个尝试的结果。周期记录同时显示目标来源/可用性、决策模式和原因，不再只显示 `targets=1`。

`latest_pipeline.json.runtime.project_dir` 标明实际加载的项目目录；脚本路径绑定该目录的代码，不会因另一个工作目录的分支更新而自动更新。`target_input.state` 区分 `valid_objects`、`valid_empty` 与 `unavailable`，`required` 显示当前任务是否要求目标源。即使诊断真值有对象，正式状态仍为 `unavailable` 且 `diagnostic_only=true`；对象数量不能替代来源有效性。必需目标规则由感知、决策和监护共同复用 `core.scene_requirements`，不把可选 IMU、雷达或车道观测的缺失扩展为全车停车条件。

可用只读工具核对快照所属任务保存的配置：

```powershell
python scripts/audit_task_sensors.py --platform-root E:\Sim-One --snapshot runtime_data/latest_perception.json
```

也可使用 `--task-id <实际任务ID>` 选择此前任务。工具仅输出主车 ID、控制器名称、启动与结束脚本、传感器列表数量及 ID/类型，不复制完整原始配置或连接凭据。`customization.name` 是控制器名称，不是主车预设名称，不能据此认定案例使用了哪辆主车。结果表示保存后的运行配置，不能代替连续 Sensor API 帧；没有文件或字段时保持 `unknown`，不猜成无传感器。回放可能复用任务 ID 并覆盖部分缓存，须结合原始仿真时间、启动脚本日志和节点启动清单核对。

`latest_perception.json.source_status.targets` 保留以下诊断：

| 字段 | 含义 |
| --- | --- |
| `config_read_ok/config_reason` | 配置查询是否成功及原因；不等于目标 API 正常 |
| `sensor_presence` | 配置确认存在、配置确认没有目标源或配置仍未知 |
| `sensor_candidates/sensor_id` | 已知候选与选择的目标 ID |
| `callback_status/callback_sensor_ids` | 回调注册结果及实际回调发现的主车目标源 ID |
| `id_verified` | ID 由配置或实际回调证实；默认 ID 探测本身不算证实 |
| `transport` | `callback`、`poll` 或未取得目标流 |
| `attempts` | 每个已尝试源的读结果、来源帧、停滞年龄和失效原因 |
| `sensor_read_ok/source/usable` | 是否取得 Sensor API 数据、正式来源及感知质量门控结果 |

## 验收与边界

在新的 01 或 15 运行期间执行只读采集：

```powershell
E:\Sim-One\Tools\python36\python.exe scripts\accept_scene.py --scene 1 --frames 20 --timeout-sec 30
```

验收器排除启动采集前留下的旧 JSON。目标源必须是可用的 `sensor:<真实ID>`；ID 可以由配置或实际官方回调证实。只有回调证实而配置查询仍失败时，报告明确保留这一警告，不伪称静态配置已获取。有效空帧可通过结构检查，但含障碍物案例还须核对目标确实被看到、尺寸/位置/速度合理及进入退出过程。

本次在 SimOne Python 3.6.4 下完整测试 **259 项通过**，其中新增 20 项接入回归和 3 项验收回归。验证覆盖目标传到 `Perception` 与 JSON，并实际确认本机原生 DLL 接受新回调注册。注册成功不代表场景正在发布数据；真实目标包仍须在新场景验收。决策/规划中的跟车、障碍物轨迹和未知距离 STOP 消费不由本次接入改动解决。

## 2026-10-01 场景 01 实跑结果：目标输入未通过

21:50:00–21:50:12 和 21:58:56–21:59:08 两次新的 `01.前方车辆静止-测试` 均加载 `sensor-discovery-v2`，平台 BridgeIO 日志也确认执行本项目的开始、结束脚本。不是旧回放或旧进程的结果。

两次运行 GPS、地图和 Ground Truth 有数据；传感器配置查询返回 false，默认 ID `perfectPerception1` 的目标轮询返回 false；目标回调注册成功，但 `callback_sensor_ids=[]`、`callback_errors={}`，未收到主车目标包。最后帧分别为 460、455。真值中有静止汽车及合理尺寸，但来源仍为 `ground_truth`，不能替代正式目标源。

对应平台 Master 的任务启动清单和 BridgeIO 的 `OnTaskStart NodeInfo` 均只有 7 个节点：动力学、SimOneDriver、交通、裁判、可视化、BridgeIO、计时，没有目标传感器节点。这是平台此次未启动传感器节点的直接证据；尚不能单凭节点列表确定主车编辑器里传感器为空，还是配置存在但未被实例化。Windows 界面检查工具初始化时报“系统找不到指定的路径”，重置重试仍失败，因此这一步配置核对待完成。

Master 的本机资源注册还显示 3 个 `ScalableObjectBasedSensor` 节点处于 Free，并支持类型 11–16，物理传感器节点也已注册。因此不是安装目录完全缺少传感器能力，但这两次任务没有申请这些节点。用户确认 AEB 主车已有摄像头、激光雷达、毫米波雷达、目标级传感器、定位传感器和 OBU，但看不到具体参数；不能仅凭运行失败否定界面上的挂载。

随后只读核对平台实际下发的 `configJsonFromWeb/*_WorkJsonFromWeb.txt`：两次任务的 SimOneDriver 配置中主车 `mainVehicleId=0`、`customization.name=NEVC_AEB`、`generalSensors=[]`；第二次任务的 BridgeIO 配置同样为 `generalSensors=[]`。VehicleDynamic 的任务列表也未包含传感器任务，案例的 `scenario.sensors` 为 `{"byId":{},"allIds":[]}`（案例级列表单独为空不能证明主车挂载为空）。与节点清单和 API 返回值合并，可确认此次平台下发的运行实例未包含传感器配置/节点，界面已有配置没有进入运行任务。具体是案例引用旧主车、主车保存未生效，还是平台生成任务配置时丢失，仍须在平台界面核对；不能直接指定其中某一种。

已对照安装包官方 Sensor 示例：初始化后直接查询配置或注册目标回调，无额外的目标源订阅调用；项目已执行这两种读取路径。没有发现可通过补一个未调用的订阅 API 修正此次空任务配置的依据。只读诊断提取必要字段，不复制整个原始节点配置（部分原始配置可能包含平台连接凭据）。

程序输出盲停，监护制动发送有回执。本次不能验收跟车/避障行为；未知距离 STOP 的规划兼容问题也单独保留。场景很快结束，随后开启的两个连续采集器均为 0 帧，报告 `NO_DATA`；不能把这两个报告解释为已连续采样或目标结构验收通过，上述判定依据是运行日志和退出前快照。

本机证据位于忽略目录 `runtime_data/target_ingestion_acceptance/`：两次运行的快照、任务节点清单、已脱离原始完整配置的必要字段摘录、队长日志片段及 `scene_01_live_diagnosis.json`。原始平台日志位于安装目录 `Common/Foundation/log/`，原始任务配置位于 `Module/VehicleDynamic/configJsonFromWeb/` 与 `Common/Foundation/configJsonFromWeb/`。共享仓库只记录结论，不提交运行文件或 SDK。

### 平台侧下一步核对

1. 在 01 案例中确认实际选用的主车预设，双击进入该主车编辑器，查看传感器区域；只改资源库里另一辆主车不会改变当前案例。
2. 若区域为空，按平台自带参赛者使用说明，从下方主车资源库拖入赛题允许的目标传感器，保存后检查案例使用了更新主车。API 指导文件以 `perfectPerception1` 理想传感器演示目标读取；传感器范围/朝向应遵循题目规定，不能任意扩大。
3. 若已有理想传感器或目标级摄像头/激光雷达，核对保存是否生效，01 是否仍引用旧版 `NEVC_AEB`，以及新任务是否分配了相应传感器节点。运行配置只用于读证据，不手改 `configJsonFromWeb` 缓存；应修正主车资源/案例引用，使平台重新生成带传感器的任务。
4. 重新启动前先开启 `accept_scene.py`。确认 `sensor:<实际ID>`、`sensor_read_ok=True`、`id_verified=True`、目标时效正常，再验收目标 ID、位置、速度、尺寸及下游收到的内容。不同场景不要求所有传感器都有数据，但需要目标观测的 01 不能用缺少目标源解释为空道路。

核对依据是本机平台帮助 `contestant_instruction/contestant_instruction` 的“主车编辑介绍”和 `quick_start/quick_start` 的“选择和配置主车 / Step 3 配置传感器模型”，以及比赛命题文件表 4 指定的可移动目标输入渠道。没有通过修改决策算法或提升 Ground Truth 权限绕过这次输入故障。

## main 跟车运行与集成验收（2026-10-02）

用户通过 main 工作目录脚本运行 15 跟车任务时仍保护停车。已核对该任务 SimOneDriver 与 BridgeIO 的控制器名称为 `NEVC_自动驾驶`、`mainVehicleId=0`、`generalSensors=[]`，脚本指向 `nevc_auto_main/scripts/StartCaptain.bat`。同时旧 main 没有目标源发现修复，因此存在代码集成缺口和本次平台任务输入缺口两项问题；合入代码并不能保证平台配置自动改变。具体是主车挂载、案例引用、保存还是发布配置问题，仍需平台核验。记录仅适用于该次任务，不推广到所有案例。

集成测试使用真实感知、决策、规划、控制、监护和发送入口，SDK 输入与车辆模型为替身：覆盖 1/14/15/25 中非默认目标 ID 的有效空帧起步、14/15 短暂及持续目标源失效后的制动和恢复、静止前车接近停车及移动前车 FOLLOW；并保留直道/弯道、可选传感器缺失及控制结构检查。SimOne Python 3.6 下全量 369 项通过。尚未完成带正式目标数据的新一轮平台跟车验收，不能把离线结果称为场景成绩通过。

### 合入后新运行（2026-10-02 15:31）

15 跟车仿真任务 `97519466-407e-44d9-aef5-322fb263dbd2` 在 15:31:30 启动本机 `nevc_auto/scripts/StartCaptain.bat`，加载 `sensor-discovery-v2`；当时其已跟踪文件与 main `3b620f4` 的树一致。15:31:33–15:33:04 的 19 条 GPS 有效周期日志均显示保护制动发送，速度为 0.00 m/s，正式目标源不可用；配置查询和目标轮询失败，回调注册成功但未发现目标源 ID。退出前反馈油门 0、制动约 0.3。最后的 GPS 过期发生在案例结束阶段，不是整段运行的起始故障。

该仿真开始时 Master 明确启动 7 个节点，没有传感器节点；SimOneDriver 保存配置为 `generalSensors=[]`。随后回放复用此任务 ID，部分缓存被覆盖，判断原始仿真时必须使用 03:31 的原生节点日志（其时钟相对队长日志偏差 12 小时），不能混用 03:33 的回放节点清单。本轮连续采集器启动较晚，捕获 0 帧并报告 `NO_DATA`；上述 19 条是周期日志记录，不能声称已完成连续帧验收或正式目标恢复验收。

只读打开资源库的 `NEVC_自动驾驶(2)`，其控制器名称为 `NEVC_自动驾驶`，编辑器展示 `sensorFusion1`、ClassID `sensorFusion`、启用、10 Hz、100 m 和勾选的融合仿真开关。这里只证明该资源的编辑器状态，不证明本次案例引用该资源或此配置已经发布；未保存或修改主车参数。各类传感器文件夹名称也不能当成已挂载的实例清单。下一步需要核对案例实际引用与保存后的发布任务，并以有效 `sensor:<实际ID>` 帧为接通判据。
