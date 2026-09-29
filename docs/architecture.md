# 架构与设计说明

本文描述状态机、并发/执行模型、安全联锁、超时/溢出处理的具体实现约定，
便于审计“为什么结果是确定且可解释的”。

## 1. 模块职责

| 模块 | 职责 |
| --- | --- |
| `config.py` | `JointLimit`（单关节限位）与 `ControllerConfig`（限位表 + 队列容量 + 命令超时 + 参考速度）。`ControllerConfig.default()` 提供 6 关节默认配置 |
| `enums.py` | `ControllerState`（IDLE/RUNNING/EMERGENCY_STOP/FAULT）、`CommandState`（QUEUED/EXECUTING/COMPLETED/REJECTED/ABORTED/TIMEOUT）、`RejectReason`（全部可区分拒绝原因） |
| `models.py` | `MotionRequest`（冻结值对象 + 幂等键）、`CommandRecord`（生命周期状态 + 拒绝原因/依据 + 时间戳）、`ControllerSnapshot` |
| `exceptions.py` | `CommandRejectedError` / `ControllerNotResetError`（库式调用可用；HTTP 层把拒绝建模为 200 + reason 字段） |
| `validator.py` | 固定顺序的参数校验：结构合法性 → 位置 → 速度 → 加速度，返回 `(RejectReason, 判定依据)` |
| `actuator.py` | `SimulatedActuator`：时间片睡眠 + 线性插值模拟运动，`stop_event` 触发就地冻结，`reset()` 回零 |
| `controller.py` | `MotionController`：一把 Condition 保护状态机/FIFO/幂等表；内联执行 + 单 worker 串行队列；急停/故障/复位 |
| `api.py` | FastAPI 应用工厂 `create_app(controller)`：命令、状态、安全联锁端点 |

## 2. 并发模型（为什么不会有两段运动同时执行）

所有共享状态（状态机、当前命令、FIFO 队列、幂等记录表）都由**同一把
`threading.Condition`** 保护。执行权的取得在锁内决定，实际阻塞式运动在
锁外进行：

```
submit() 持锁:
  1. 查幂等表（重复直接返回首次记录）
  2. 联锁判定（急停/故障 -> NOT_RESET）
  3. 参数校验（validator 固定顺序）
  4. 若 current != None:
       blocking  -> MOTION_CONFLICT
       非阻塞且队列满 -> QUEUE_FULL
       非阻塞 -> 入队，返回 QUEUED
  5. 若空闲：当前线程 claim 执行权（current=记录, state=RUNNING,
     新建 stop_event），释放锁，调用 actuator.execute_move(...)
运动结束:
  重新持锁写终态（COMPLETED/ABORTED），current=None，
  若无并发急停/故障则 state=IDLE，notify_all

worker 单线程:
  等待 state==IDLE 且队列非空 -> 队首出队 ->
  超时判定 -> claim 执行权 -> 锁外执行 -> 落终态
```

**空闲竞态的处理**：多个线程同时在“控制器空闲”时提交，全部在锁内竞争，
只有一个线程看到 `current is None` 并内联取得执行权；其余线程随后看到
`current != None`，按 blocking/非阻塞分别得到冲突拒绝或入队。因此
**blocking 命令不会经历排队与排队超时**——它要么被拒，要么执行到结束。

## 3. 模拟执行器与运动耗时

`SimulatedActuator.execute_move()` 按时间片（默认 10ms）睡眠并对各关节做
起点→目标的线性插值：

- 运动时长 = `max_j |target_j - start_j| / velocity`（下限一个时间片）；
- 每个时间片检查 `stop_event`：被置位立即返回 `False`，关节位置停在最近
  一次插值点——即“就地冻结”，不会滑行到目标；
- 正常结束位置写成精确目标值，返回 `True`。

位置更新受执行器内部锁保护；`snapshot_positions()` 随时可读。

## 4. 状态机与安全联锁

### 4.1 控制器状态转移

| 起始 | 事件 | 结束 | 动作 |
| --- | --- | --- | --- |
| IDLE | 有效命令取得执行权 | RUNNING | current=命令，记录 EXECUTING |
| RUNNING | 运动正常结束 | IDLE | 记录 COMPLETED，唤醒等待者 |
| RUNNING | worker 取出排队项 | RUNNING | （不经过 IDLE 也可连续执行队列） |
| RUNNING/IDLE | `emergency_stop()` | EMERGENCY_STOP | stop_event.set()；队列作废 ABORTED；在跑运动 ABORTED |
| RUNNING/IDLE | `external_fault()` | FAULT | 同上，区别仅在状态名与故障信息 |
| EMERGENCY_STOP/FAULT | `reset()` | IDLE | 回零、清队列、清命令历史、清故障信息 |
| RUNNING | `reset()` | RUNNING（不变） | 运动中复位被忽略并记录警告 |

### 4.2 联锁规则（提交时的判定优先级）

```
重复 request_id  >  NOT_RESET  >  参数校验  >  忙时决策
```

1. **重复**请求永远短路返回首次记录（即使首次是 REJECTED/TIMEOUT）。
2. 急停/故障态下，除复位/查询外的一切提交一律 `NOT_RESET`（参数不再校验，
   因为安全状态优先）。
3. 然后才是位置、速度、加速度校验（顺序见 §5）。
4. 最后才考虑“忙”：冲突拒绝 / 满队 / 入队。

### 4.3 复位语义

`reset()` 的目标是一个**明确稳定的初始状态**：状态 IDLE、执行器各关节回到
零位（home，默认全 0）、FIFO 清空、幂等/历史记录表清空、故障信息清空。
因此复位后此前所有 `request_id` 的查询返回 404（HTTP）或 `None`（库），
这本身就是“已回到初始态”的可观察证据。运动中不允许复位，避免与在跑运动
竞争；必须先急停或等运动自然结束。

## 5. 参数校验顺序（可区分拒绝原因）

`validator.validate_request()` 严格按下列顺序短路返回，附带“判定依据”文本：

1. `BAD_REQUEST`：`positions` 为空/类型错、含未知关节、缺少关节（多关节
   运动必须覆盖全部关节）、任一数值为 NaN/Inf；
2. `POSITION_OUT_OF_RANGE`：任一关节目标超出 `[min_position, max_position]`；
3. `VELOCITY_OUT_OF_RANGE`：速度 ≤ 0，或大于涉及关节的 `min(max_velocity)`；
4. `ACCELERATION_OUT_OF_RANGE`：加速度 ≤ 0，或大于涉及关节的
   `min(max_acceleration)`。

速度/加速度取“涉及关节中最小上限”，保证多关节同步运动时所有关节都不越限。

## 6. 排队、溢出与超时

- **串行 FIFO**：只有非阻塞命令排队；单 worker 线程，队首出队后才可能执行。
- **溢出**：`len(queue) >= queue_capacity` 的新排队请求在提交瞬间被
  `QUEUE_FULL` 拒绝，确定、立即、不等待。
- **超时**：worker 取出队首时比较 `now - enqueue_time > command_timeout`：
  超时则直接置 `TIMEOUT`（带实际等待时长），**不启动运动**。超时基准是
  “入队到开始执行”，开始执行后的运动时间不计入超时。
- **不丢命令**：每个被接受的请求都创建 `CommandRecord`；溢出/超时/急停
  都有明确终态（REJECTED/TIMEOUT/ABORTED），可凭 `request_id` 查询到
  （复位前）。压力测试 `test_no_command_lost_under_pressure` 用 50 线程并发
  验证每个 ID 都有终态记录。

## 7. 幂等性

`_records: dict[request_id, CommandRecord]` 是幂等表。查询与插入在同一把锁
内完成，因此并发重复提交也只有一个赢家。重复请求：

- 不做任何校验/入队/执行；
- 返回**同一个** `CommandRecord` 对象（`is` 相等）；
- 首次记录的参数与终态不受二次提交内容影响。

复位清空幂等表，因此复位后同一 ID 可作为全新请求提交。

## 8. HTTP 约定

- 被领域规则拒绝时返回 HTTP **200** + `state="REJECTED"` +
  `reject_reason` + `reject_detail`，便于客户端用统一结构按原因分支；
  Pydantic 结构错误返回 422；未知 ID 查询返回 404。
- 安全端点：`POST /safety/emergency-stop`、`/safety/external-fault`、
  `/safety/reset` 均返回操作后的控制器快照。

## 9. 日志与可解释性

`robot_motion` logger 在以下时机输出**请求参数与判定依据**：

- 命令受理/入队：完整 `positions/velocity/acceleration/blocking` 与队列位置；
- 每一种拒绝：`request_id` + 完整参数 + `reject_reason` + `reject_detail`
  （如“关节 2 目标 999 超出限位 [-180,180]”、“排队 0.877s 超过 0.4s”）；
- 急停/故障/复位、运动开始/完成/中止。

测试用 `--log-cli-level=INFO` 运行即可在控制台看到完整判定链路。

## 10. 本地复现

```bash
uv run python examples/demo.py   # 8 个场景：正常/越界/幂等/冲突/溢出/急停复位/超时/并发压力
uv run pytest --log-cli-level=INFO
uv run python main.py            # http://127.0.0.1:8000/docs 手工触发
```
