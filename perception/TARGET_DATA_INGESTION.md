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
