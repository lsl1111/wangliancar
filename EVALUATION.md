# 本地评价记录接入

连接 SimOne 后，适配层通过已安装 SDK 的 `InitEvaluationServiceWithLocalData(vehicle_id, true)` 开启本地评价。SDK 提供 Judge 和主车 GPS 记录，项目不自行生成替代评分数据。默认开启，与车辆控制发送开关独立。

当前原生包装只支持已经核对的 SDK `3.0.0001` 两参数签名。它复用 `SimOneServiceAPI` 已加载的 DLL，不修改安装目录的 Python 文件。在线文档的新版接口多了一个参数，因此未知版本会在调用原生接口前报错。

运行中默认每秒调用 `SaveEvaluationRecord()`，在控制指令发送后执行；暂停时也可保存已经收到的数据。案例停止、Ctrl+C、结束脚本停止、运行异常都会进入清理路径，先保存再调用 `SoTerminateSimOneAPI()`。初始化失败会阻止进入控制循环；定期保存失败会记录错误并重试，最终保存失败使进程返回错误，同时仍清理 SDK、PID 文件和实例锁。

`KillCaptain.bat` 现在调用 `stop_captain.ps1`，向本目录的 `runtime_data/captain.stop` 写入当前 PID。运行层只响应属于自身 PID 的请求，旧请求不会停止新进程。结束脚本等待最多 5 秒；SDK 不响应时才核对进程身份并强制结束，日志会提示最后一次保存可能不完整。Windows 强制结束进程不能保证执行 Python 的 finally，因此运行中也保存。

## 本机测试入口

PR12 工作目录的启动脚本：

```text
F:\wangliancar\nevc_auto_p0_fixes\scripts\StartCaptain.bat
```

结束脚本：

```text
F:\wangliancar\nevc_auto_p0_fixes\scripts\KillCaptain.bat
```

必须启动新任务才能加载本次修改。其他工作目录的控制器和历史回放不会使用该目录的新代码。

配置默认值：

```ini
[app]
evaluation_enabled = true
evaluation_flush_interval_sec = 1.0
```

可以在未提交的 `config/local.ini` 中覆盖。纯诊断时可显式关闭评价记录；控制仍由原有的 `send_control` 和 `safety_brake_enabled` 开关决定。

只检查 SDK 加载与评价导出绑定，不连接案例、不记录或发送控制：

```powershell
E:\Sim-One\Tools\python36\python.exe F:\wangliancar\nevc_auto_p0_fixes\main.py --self-test
```

## 平台验收

1. 本次 `captain.log` 应出现“评价记录已初始化”，随后出现“评价记录已保存”；正常退出前最后保存日志应为 `final=True`。
2. `latest_pipeline.json.evaluation` 应显示 `initialized=true`、`save_count>0`、`last_save_ok=true`。同时核对 `runtime.project_dir`、本次启动时间和快照修改时间。保存成功是 SDK 回执，仍需检查实际评价文件内容。
3. 案例运行期间检查平台用户目录 `evaluation/<taskID>.json` 中是否有真实 Judge / mainvehicle 记录。结束后平台会将其打包成 `<suiteID>.dat` 并删除单任务 JSON；不能仅以事后 JSON 不存在判断记录失败。
4. 以 NEVC 评分端认可的正式赛题验证上传响应和数字评分。本地场景 pass、SDK 保存成功、评分端接受赛题是分别需要验证的条件。

离线测试验证 C ABI 参数、版本限制、初始化失败、保存重试、异常清理和真实结束批处理的正常退出路径，不能代替真实案例记录及数字评分验收。尚未证明本机 SDK 每次定期保存的累计记录行为和平台打包时序；首轮平台测试需检查最终包中的记录数量与末帧事件。

接口依据：安装 SDK 的 `include/SimOneEvaluationAPI.h`；[官方评价初始化文档](https://simone-docs.51sim.com/en/api/python/evaluation/init/index.html)及[官方保存文档](https://simone-docs.51sim.com/en/api/python/evaluation/record/index.html)。在线版本差异按上文处理。
