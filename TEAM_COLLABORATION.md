# 四人协作开发说明

这份文档给全体组员使用。我们的目标不是四个人分别做四套程序，而是在同一个项目中，通过固定接口依次完成感知、决策、规划和控制。

## 1. 四个人分别负责什么

程序的数据流固定如下：

```text
SimOne 官方 API
       ↓
队长：Perception（车辆、目标、车道、场景数据）
       ↓
决策成员：DecisionTarget（做什么）
       ↓
规划成员：Trajectory（怎么走）
       ↓
控制成员：ControlOut（油门、刹车、转向）
       ↓
队长：调用 SoSetDrive 发送给 SimOne
```

| 成员 | 主要目录或文件 | 不能擅自修改 |
|---|---|---|
| 队长 | `core/`、`perception/`、`simone_platform/`、`runtime.py` | 其他成员的算法实现 |
| 决策成员 | `members/decision/`、`members/decision_stub.py` | SimOne API、规划和控制代码 |
| 规划成员 | `members/planning/`、`members/planning_stub.py` | SimOne API、决策和控制代码 |
| 控制成员 | `members/control/`、`members/control_stub.py` | SimOne API、决策和规划代码 |

`core/interfaces.py` 是全队共同遵守的接口协议。任何人需要增加、删除或修改字段，都要先在群里说明用途，由队长统一修改。

## 2. 三个成员的固定入口

### 决策成员

入口文件：`members/decision_stub.py`

```python
def decide(perception):
    # 输入：Perception
    # 输出：DecisionTarget
    return decision_target
```

决策只回答“车辆下一步应该做什么”，例如保持车道、跟车、停车或紧急制动。不要在决策模块里直接发送油门、刹车或转向。

### 规划成员

入口文件：`members/planning_stub.py`

```python
def plan(perception, decision):
    # 输入：Perception、DecisionTarget
    # 输出：Trajectory
    return trajectory
```

规划负责把决策目标变成一系列轨迹点。不要在规划模块里调用 SimOne API，也不要直接发送控制指令。

### 控制成员

入口文件：`members/control_stub.py`

```python
def compute_control(perception, trajectory):
    # 输入：Perception、Trajectory
    # 输出：ControlOut
    return control
```

控制负责根据轨迹计算油门、刹车和转向。输出前必须限制数值范围：

```text
throttle：0.0 ～ 1.0
brake：   0.0 ～ 1.0
steering：-1.0 ～ 1.0
```

代码未完成时必须保持 `ControlOut.valid = False`，防止占位代码误动车辆。

## 3. 算法代码较多时怎么放

不要把所有算法都挤在 `*_stub.py` 中。可以建立自己的子目录：

```text
members/
├── decision/
│   ├── behavior.py
│   ├── obstacle_judge.py
│   └── speed_decision.py
├── planning/
│   ├── path_planner.py
│   └── speed_planner.py
├── control/
│   ├── lateral.py
│   └── longitudinal.py
├── decision_stub.py
├── planning_stub.py
└── control_stub.py
```

`*_stub.py` 始终作为固定入口。例如决策成员可以这样接入：

```python
from members.decision.behavior import DecisionEngine

engine = DecisionEngine()


def decide(perception):
    return engine.run(perception)
```

这样队长的 `runtime.py` 不需要跟着成员算法反复修改。

## 4. 第一次下载项目

```powershell
git clone https://github.com/lsl1111/wangliancar.git
cd wangliancar
```

根据自己的分工创建分支。分支名使用英文，不能直接在 `main` 上开发。

```powershell
# 决策成员示例
git switch -c decision/basic-behavior

# 规划成员示例
git switch -c planning/lane-path

# 控制成员示例
git switch -c control/pid-controller
```

## 5. 每天开始开发前

先更新 `main`，再把最新代码合入自己的分支：

```powershell
git switch main
git pull origin main
git switch 自己的分支名
git merge main
```

如果出现冲突，不要随意删除别人的代码。先确认冲突属于哪个成员负责的文件；公共接口或 `runtime.py` 冲突交给队长处理。

## 6. 完成一个小功能后

先查看改了什么：

```powershell
git status
git diff
```

然后提交和推送：

```powershell
git add members
git commit -m "feat: 实现基础决策逻辑"
git push -u origin 自己的分支名
```

提交说明建议使用：

```text
feat: 新增功能
fix: 修复问题
test: 增加测试
docs: 修改文档
refactor: 重构但不改变功能
```

不要提交以下内容：

- SimOne SDK 和 DLL
- `runtime_data/` 运行日志
- `__pycache__/`
- `config/local.ini` 本机配置
- 密码、Token、账号凭据

## 7. 在 GitHub 创建 Pull Request

推送分支后，在 GitHub 创建 Pull Request：

```text
自己的分支 → main
```

PR 说明至少包含：

```text
1. 完成了什么
2. 修改了哪些文件
3. 输入和输出是什么
4. 如何测试
5. 目前还有什么问题
```

不要自己强制推送 `main`。由队长检查接口、测试和联调结果后完成合并。

## 8. 提交前必须检查

在项目根目录运行：

```powershell
E:\Sim-One\Tools\python36\python.exe -m unittest discover -s tests -v
```

至少确认：

- 测试全部通过。
- 没有直接 import 或调用 SimOne SDK。
- 函数输入、输出类型符合 `core/interfaces.py`。
- 感知无效时不会继续输出危险控制。
- 控制输出没有超出规定范围。
- 没有误改其他成员负责的文件。

## 9. 四人联调顺序

联调按照以下顺序进行，不要一开始就打开车辆控制：

1. 队长确认 `latest_perception.json` 中 GPS、车道和目标正确。
2. 接入决策，日志中检查 `DecisionTarget` 是否符合场景。
3. 接入规划，检查轨迹方向、长度和目标速度。
4. 接入控制，先在低速、简单场景验证输出。
5. 全员确认后，队长才将 `config/default.ini` 的 `send_control` 改为 `true`。

出现问题时，先判断错误属于哪一层：感知数据错误找队长，行为选择错误找决策，轨迹错误找规划，车辆跟踪错误找控制。

## 10. 最重要的三条规则

1. 不直接修改或强制推送 `main`。
2. 不绕过接口直接调用其他成员内部代码。
3. 不在控制模块通过联调前开启 `send_control=true`。

## 11. 使用 Agent/Codex 协作

全队公共约定保存在根目录 [AGENTS.md](AGENTS.md)。各成员在自己的工作目录维护局部约束：

| 工作 | 局部规则 |
|---|---|
| 队长感知 | [perception/AGENTS.md](perception/AGENTS.md) |
| 决策 | [members/decision/AGENTS.md](members/decision/AGENTS.md) |
| 路径与轨迹规划 | [members/planning/AGENTS.md](members/planning/AGENTS.md) |
| 控制 | [members/control/AGENTS.md](members/control/AGENTS.md) |

感知保留在现有 `perception/`；三个成员的 `*_stub.py` 保留原位置和函数接口。局部 AGENTS 的自然作用域是所在目录及其子目录，不包含同级 stub。根 AGENTS 因此要求：无论从哪个目录启动任务，修改某模块的 stub、实现或测试前，都额外阅读对应局部规则。

Codex 启动时按仓库根目录到工作目录读取指令链；从成员目录启动可直接带入该目录规则，从仓库根目录启动则依上面的明确读取要求补充。规则更新后，在对应目录启动新任务以加载新规则。各成员可独立维护自己的局部规则；个人临时偏好在任务中说明，不覆盖队友约束。AGENTS 是 Agent 行为约定，不是文件权限隔离。

规划 Skill 安装于 [.agents/skills/path-planning/SKILL.md](.agents/skills/path-planning/SKILL.md)。在该仓库任务中写“使用 `$path-planning` 设计/评审轨迹规划”可显式触发，也可按任务匹配自动选择。项目根目录的 `.agents/skills` 对仓库内子目录可发现；新安装的 Skill 从后续轮次使用，若未显示可重新打开任务/重启 Codex。此 Skill 已按项目做依赖和职责适配，来源与差异见其 `SOURCE.md`。

AGENTS 保存稳定规则，Skill 保存专业工作流；规划经验记录在 [members/planning/KNOWLEDGE.md](members/planning/KNOWLEDGE.md)，按证据维护，不把实验记录不断追加到根规则。既有私人资料是否跟踪由其原约定决定，新增协作文件不意味着可以一并提交所有未跟踪文件。

作用域和发现机制参考 [Codex AGENTS 官方说明](https://learn.chatgpt.com/docs/agent-configuration/agents-md) 与 [Skill 官方说明](https://learn.chatgpt.com/docs/build-skills)。

