# 来源与项目适配

- 上游：[a5c-ai/babysitter path-planning](https://github.com/a5c-ai/babysitter/tree/feb68abe397acc14f32b34984975c48fedba2b33/library/specializations/domains/science/automotive-engineering/skills/path-planning)。
- 安装时将用户给出的 `HEAD` 固定为提交 `feb68abe397acc14f32b34984975c48fedba2b33`；后续升级需显式比对，不自动追随 HEAD。
- 上游目录仅包含 `SKILL.md`（1635 字节），没有脚本、资源文件或相对路径依赖。上游原始文件字节 SHA-256：`41cbe26af1321ae6a9d3a3e08bc9f1d326876235b1702b093542d16139d73947`。
- 使用 skill-installer 的稀疏 Git 安装方式，仅复制指定 Skill；另从同一提交保留上游 [MIT 许可](LICENSE.md)。许可证只适用于此处引入的上游内容，不改变整个项目的许可。

## 已做的适配

1. 缩小触发描述到 path/trajectory planning，保留轨迹优化、碰撞检查、舒适性/安全约束、紧急与泊车轨迹能力；上游行为决策和控制器实现不能扩张本成员职责。
2. 保留 `name`、上游版本/类别等元数据；移除 babysitter 专用 `graph`，把 tags 转为字符串，并移除宿主专用 `allowed-tools` 名称。工具权限由当前 Codex 会话决定。
3. 将上游 ROS/ROS2、Apollo、Autoware、MATLAB/Simulink 依赖列表明确为此适配不需要的外部框架；未安装这些依赖，也未安装 Nav2。
4. 保留 ADA 流程编号作为来源说明，不引入其关联流程或其他 Skill。添加适用于现有接口的专业检查步骤，并链接项目知识入口。

当前安装内容只有 `SKILL.md`、`LICENSE.md` 和本文件。没有引入其他 babysitter 目录、hooks、guards、plugins、执行脚本或 MCP 配置。该 Skill 提供规划工作流，不是已经实现或验证过的规划算法。
