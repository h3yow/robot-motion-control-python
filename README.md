# 本地多关节机器人运动控制（Python）

一套**纯本地**的多关节机器人运动命令与执行流程实现。执行器由进程内模拟状态
驱动，**不依赖任何外部硬件或服务**，所有并发下发、急停、参数越界等场景的结果
都确定、可解释、可在本地复现验证。

## 快速开始

```bash
uv sync                       # 安装依赖（fastapi / pytest）
uv run pytest                 # 运行全部 23 个测试
uv run python examples/demo.py   # 一键演示全部场景（带日志）
uv run python main.py         # 启动 HTTP 服务：http://127.0.0.1:8000/docs
```

## 目录结构

```
robot_motion/
  config.py       # 安全限位与队列/超时配置项
  enums.py        # 控制器状态机、命令生命周期、拒绝原因枚举
  models.py       # MotionRequest / CommandRecord / ControllerSnapshot
  exceptions.py   # 领域异常（携带可区分拒绝原因）
  validator.py    # 位置/速度/加速度越界校验（固定判定顺序）
  actuator.py     # 本地模拟执行器（时间片插值，急停就地冻结）
  controller.py   # 核心：状态机 + 串行队列 + 幂等 + 急停/故障/复位
  api.py          # FastAPI HTTP 接口
tests/            # 越界/幂等/冲突/急停/并发/队列/API 测试
examples/demo.py  # 本地一键复现脚本
docs/architecture.md  # 详细设计（状态机流转图、并发模型）
```

## 命令模型

`MotionRequest`：

| 字段 | 含义 |
| --- | --- |
| `request_id` | 调用方提供的**幂等键**，同一 ID 重复提交只执行一次，返回首次记录 |
| `positions` | `关节号 -> 目标位置`，必须覆盖全部关节（默认 6 关节） |
| `velocity` | 标量速度约束，`0 < v <= 涉及关节最小速度上限` |
| `acceleration` | 标量加速度约束，`0 < a <= 涉及关节最小加速度上限` |
| `blocking` | `true`：忙时直接冲突拒绝；`false`：忙时入队等待 |

## 状态机

```
                 submit(空闲)                运动完成
        IDLE ------------------> RUNNING ------------------> IDLE
         ^                          |  ^                       |
         | reset                    |  │  worker 取出排队项     │
         |                 estop/   |  └───────────────────────┘
         |                 fault    v
         └──────────── EMERGENCY_STOP / FAULT（运动就地中止，队列作废）
                       （未复位前一切新运动以 NOT_RESET 拒绝）
```

命令生命周期：`QUEUED -> EXECUTING -> COMPLETED`；异常终态为
`REJECTED`（校验/冲突/满队）、`ABORTED`（被急停/故障中止，或在队列中被作废）、
`TIMEOUT`（排队超时）。

## 安全规则

1. **参数校验顺序固定**：结构合法性(`BAD_REQUEST`) → 位置 → 速度 → 加速度，
   每条拒绝都带机器可读 `reject_reason` 与人读 `reject_detail`（判定依据）。
2. **运动冲突**：有运动执行中时，`blocking=true` 请求以 `MOTION_CONFLICT`
   拒绝；`blocking=false` 进入 FIFO 队列。
3. **串行化（硬不变量）**：同一时刻至多一段运动。空闲时第一条命令在提交者
   线程内联执行；忙时命令由**单条 worker 线程**串行取出执行。
4. **队列溢出确定**：队列长度达到 `queue_capacity` 时，新排队请求立即以
   `QUEUE_FULL` 拒绝（不入队、不等待）。
5. **排队超时确定**：排队命令在**到达队首**时若已等待超过
   `command_timeout`，置 `TIMEOUT` 终态（不会开始运动）。任何命令都有明确
   终态记录，**不丢命令、不状态错乱**。
6. **急停 / 外部故障**：当前运动在一个时间片（默认 10ms）内就地冻结（停在
   中断点，不滑到目标），等待队列全部置 `ABORTED`，状态切到
   `EMERGENCY_STOP`/`FAULT`；复位前一切新运动以 `NOT_RESET` 拒绝。
7. **复位**：`RUNNING` 时忽略复位（须先急停或等完成）；在
   `EMERGENCY_STOP`/`FAULT` 下复位 → 执行器回零、队列与命令历史清空、
   状态回到稳定的 `IDLE`。
8. **幂等**：同一 `request_id` 无论提交多少次，只产生一条记录、最多执行一次，
   后续提交原样返回首次记录（即使二次参数不同也不会被覆盖）。

## 配置项（`ControllerConfig`）

| 配置 | 默认值 | 说明 |
| --- | --- | --- |
| `joint_limits` | 6 关节：位置 ±180，速度 ≤60，加速度 ≤120 | 每关节 `min/max_position`、`max_velocity`、`max_acceleration` |
| `joint_count` | 6 | 关节数量 |
| `queue_capacity` | 16 | 等待队列容量（不含正在执行的一条） |
| `command_timeout` | 5.0s | 排队命令从入队到开始执行的最长等待 |
| `default_speed` | 10.0 | 参考速度；模拟耗时 = 最大关节位移 / 请求速度 |
| 执行器时间片 | 0.01s（`actuator._TICK_SECONDS`） | 急停响应粒度/位置插值精度 |

## 拒绝原因对照（`RejectReason`）

| 原因 | 触发条件 |
| --- | --- |
| `BAD_REQUEST` | 空目标/未知关节/缺关节/NaN、Inf |
| `POSITION_OUT_OF_RANGE` | 任一关节目标位置超出限位 |
| `VELOCITY_OUT_OF_RANGE` | 速度 ≤0 或超过涉及关节最小速度上限 |
| `ACCELERATION_OUT_OF_RANGE` | 加速度 ≤0 或超过涉及关节最小加速度上限 |
| `MOTION_CONFLICT` | 运动执行中收到 blocking 请求 |
| `QUEUE_FULL` | 队列已满时提交排队请求 |
| `QUEUE_TIMEOUT` | 排队命令出队时已超过 `command_timeout` |
| `NOT_RESET` | 急停/故障态未复位时下发运动 |
| `DUPLICATE_REQUEST` | （仅日志）重复 ID 返回首次记录，不二次执行 |

## HTTP 接口（FastAPI）

| 方法/路径 | 说明 |
| --- | --- |
| `POST /commands` | 提交运动；返回 `state`/`reject_reason`/`reject_detail`（拒绝仍为 HTTP 200，按原因字段区分） |
| `GET /commands/{id}` | 查询单条命令（复位清空历史后返回 404） |
| `GET /status` | 控制器快照：状态/队列长度/当前命令/各关节位置/故障信息 |
| `POST /safety/emergency-stop` | 急停 |
| `POST /safety/external-fault` | 上报外部故障 |
| `POST /safety/reset` | 复位回初始状态 |

## 测试与日志

- 覆盖：越界请求按原因区分、幂等、运动冲突、运动中急停/故障/复位、
  20/50 并发提交串行化、队列溢出、排队超时、压力下不丢命令、HTTP 端到端。
- 测试日志通过 `robot_motion` logger 打印**请求参数与判定依据**；
  用 `uv run pytest --log-cli-level=INFO` 可实时查看。

更详细的设计见 [docs/architecture.md](docs/architecture.md)。
