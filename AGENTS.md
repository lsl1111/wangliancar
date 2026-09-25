# wangliancar Agent 协作规则

## 项目与共同约定

- 开始任务先读 `README.md`、`TEAM_COLLABORATION.md` 和实际接口；团队职责以协作说明为准。遇到冲突保留原有明确约定，并说明差异。
- 本项目是 NEVC/SimOne 四人协作底座，保持 Python 3.6 兼容和现有单进程调用链：`SimOne -> Perception -> DecisionTarget -> Trajectory -> ControlOut -> SimOne`。
- 保留 `members/*_stub.py` 的固定入口、签名和 import 路径；成员算法逐步放入自己的子目录，避免依赖其他成员内部实现。
- 维持现有 Python + SimOne 技术栈。引入 ROS/ROS2/Nav2、其他大型框架或改变整体架构，需要明确的团队决策；Skill 中的候选技术不构成此类授权。
- 面向非专业开发者解释重要术语、修改原因和取舍，报告验证结果及其适用范围。

## 职责与规则读取

根规则适用于整个仓库；局部 `AGENTS.md` 只补充本模块约束。开始修改下表中的文件或其测试前，主动读取对应局部规则，即使任务从仓库根目录启动。

| 负责人/模块 | 维护范围 | 局部规则 |
| --- | --- | --- |
| 队长/集成 | `core/`、`simone_platform/`、`main.py`、`runtime.py`、`config/`、`scripts/` | 本文件及团队说明 |
| 队长/感知 | `perception/` | `perception/AGENTS.md` |
| 决策成员 | `members/decision/`、`members/decision_stub.py` | `members/decision/AGENTS.md` |
| 规划成员 | `members/planning/`、`members/planning_stub.py` | `members/planning/AGENTS.md` |
| 控制成员 | `members/control/`、`members/control_stub.py` | `members/control/AGENTS.md` |

子目录规则不会自动覆盖同级的 `*_stub.py`；上表是明确的补充读取要求，不是改变 AGENTS 的目录作用域。`tests/` 下的相关测试也按被测模块读取规则。

## 公共接口与跨模块修改

- `core/interfaces.py` 是共同协议，保持独立于 SimOne SDK。成员不得绕过它直接调用其他成员内部代码或自行连接 SDK。
- 按团队说明，公共字段的增删和语义改变应先说明用途，由队长统一修改。已有明确授权时完成必要的最小变更；否则先提交接口需求，不擅自落实公共接口变更。
- 修改其他负责人文件前先检查影响，记录原因、范围和验证；保留已有个人约束与未提交工作，不用本模块偏好覆盖其他模块规则。
- 任何获授权的公共接口修改都需检索全部生产者、消费者、序列化、测试和文档，并同步验证兼容性；共享代码冲突交由队长协调。
- `send_control=false` 是联调默认值；开启控制遵守团队的全员联调确认与队长操作要求。成员算法不能自行改变开关或发送车辆指令。

## 验证与交付

- 业务修改补充与风险相称的离线测试；提交前在仓库根目录用已确认的 SimOne Python 3.6 执行 `python -m unittest discover -s tests -v`。仅有其他版本环境时如实说明，不能声称已验证目标环境。
- 文档/协作配置修改检查链接、作用域与原规则一致性；Skill 修改检查前置元数据、依赖和发现结果。离线通过不等于 SimOne 场景通过。
- 在成员分支开发，依团队流程通过 PR 交队长检查；不直接修改或强推 `main`。提交前检查差异及暂存清单，按任务范围选择文件，尊重用户明确要求保持未跟踪的资料。
- 不提交 SimOne SDK/DLL、`runtime_data/`、`__pycache__/`、`config/local.ini` 或凭据；不把本机绝对安装路径写进共享运行配置。

## 规则与知识维护

- 根文件只保存稳定的全队约束，模块规则留在局部 `AGENTS.md`，专业工作流放在 `.agents/skills/`。
- 规划经验入口为 `members/planning/KNOWLEDGE.md`；其他模块需要时采用同样方式。单次 bug、临时参数和未确认推测不写入 AGENTS。
- 只有经过多次任务验证、适用于整个项目并经团队确认的经验，才考虑提升为根规则。升级上游 Skill 时保留来源及项目适配说明。
