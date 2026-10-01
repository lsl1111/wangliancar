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
