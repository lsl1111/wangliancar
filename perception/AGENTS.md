# 感知模块约束

继承根 `AGENTS.md`。本目录由队长维护，现有感知实现就在这里，不另建 `members/perception/`。

- 职责：把适配层原始 GPS/目标数据与地图查询结果整理成 `Perception`；维护 `PerceptionBuilder`、`RouteManager`，为下游提供一致的数据。
- 接口：`build()`/`build_from_raw(raw)` 输出 `Perception`，`RouteManager.update(ego)` 输出 `LaneContext`。遵守 `core/interfaces.py`，明确坐标、单位、时间和有效性；缺失信息保留未知/无效状态。
- 范围：本目录实现及感知测试。SDK 装载、连接、原始字段适配仍由 `simone_platform/simone_adapter.py` 管理；当前 `RouteManager` 通过 adapter 的地图对象查询，不向成员模块暴露 SDK 对象，也不借本次规则建设重构此实现。
- 不承担行为选择、轨迹生成或控制输出。需要改适配层、公共接口或运行循环时遵循根文件的跨模块流程。
- 验证：使用假 adapter/地图对象检查 GPS 缺失、目标过滤、车道不可用、坐标转换和序列化；地图读取与真实场景准确性另行联调，不能用 SDK 导入成功代替。
- Skill：按感知任务选择已安装的相关 Skill；本项目的 `path-planning` 不作为感知任务的默认工作流。
