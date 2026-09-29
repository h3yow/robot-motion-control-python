# 本地多关节机器人运动控制 — 设计说明

全部功能在**本地进程**内完成：执行器状态是模拟的，时间由一个确定性
**虚拟时钟（SimClock）**驱动，不依赖任何真实硬件、PLC 或外部服务。
可通过单元测试或本地启动 HTTP 服务完整复现所有行为（含急停、并发、越界）。

---

## 1. 模块结构

```
robot_motion/
  models.py      枚举与值对象：状态机状态、命令状态、拒绝原因、命令/快照模型
  config.py      关节硬限位、队列容量、排队超时、虚拟时钟节拍等配置
  clock.py       SimClock：按固定节拍推进的虚拟单调时钟（毫秒）
  actuator.py    SimulatedActuator：各关节位置 + 梯形速度规划，按拍推进/冻结
  interlock.py   SafetyInterlock：IDLE/RUNNING/ESTOPPED/FAULT 安全联锁
  engine.py      ExecutionEngine：单工作线程串行执行、排队、超时、急停清空
  controller.py  MotionController：校验、幂等去重、冲突检测、安全操作门面
  api.py         FastAPI HTTP 接口
main.py          uvicorn 入口
tests/           37 个测试：校验 / 幂等 / 并发 / 安全 / HTTP
```

调用关系：`api → controller → {interlock, engine → {actuator, clock}}`。

---

## 2. 状态机（安全联锁 SafetyInterlock）

四个状态，线程安全：

| 状态 | 含义 | 是否接受新运动 |
|------|------|----------------|
| `IDLE` | 初始/稳定空闲，执行器静止在确定位置 | 是 |
| `RUNNING` | 恰有一段运动在执行（全机同一时刻只允许一段） | 新命令排队/按规则拒绝 |
| `ESTOPPED` | 急停已锁存，运动已就地冻结、队列已清空 | **否（必须先复位）** |
| `FAULT` | 外部故障已锁存，处理同急停 | **否（必须先复位）** |

合法迁移：

```
IDLE    --begin_motion-->  RUNNING
RUNNING --end_motion---->  IDLE          （运动正常结束）
任意    --emergency_stop-> ESTOPPED      （运行中的运动立即冻结，排队全部取消）
任意    --external_fault--> FAULT        （同上）
ESTOPPED/FAULT --reset---> IDLE          （仅显式复位可恢复）
```

安全规则：

1. **急停/故障是锁存的**：重复触发不改变结果（幂等），未 `reset` 前状态不会自行离开。
2. **复位只能从 `ESTOPPED`/`FAULT` 进行**；`RUNNING` 下复位被拒绝——必须先停下，
   保证恢复时一定回到"无运动在途、队列为空"的确定 `IDLE`。
3. **跳闸与停动是原子的**：`emergency_stop()`/`external_fault()` 通过引擎在
   工作线程使用的同一把锁内完成「锁存联锁 + 冻结执行器 + 清空队列」，
   工作线程不可能在"已跳闸"与"已停动"之间再推进一拍。
4. **急停就地冻结**：执行器停在**上一拍已到达的位置**（不会再用新时刻外推），
   速度清零、目标清除。复位后该冻结位置就是后续运动的确定起点。
5. **联锁期间**所有新提交一律以 `INTERLOCK_ENGAGED` 拒绝（见下文拒绝优先级）。

复位后的稳定初始状态：`state=IDLE`、`current=None`、`queue=[]`、
各关节静止在冻结（或初始 0）位置。

---

## 3. 命令生命周期（CommandStatus）

```
PENDING → QUEUED → RUNNING → COMPLETED
                      │
                      ├─(急停)→ EMERGENCY_STOPPED   （运行中 / 排队中 同因取消）
                      ├─(故障)→ FAULTED
                      └─(排队超时)→ QUEUE_TIMEOUT
提交被拒 → REJECTED（带可区分 reject_reason，不进入执行）
```

- 同一时刻**全机最多一段运动**执行；其余已接受命令在 FIFO 队列等待。
- 一条命令**有且仅有一个终态**，状态只在引擎工作线程上变更，杜绝并发下的状态错乱。

---

## 4. 提交校验顺序与可区分拒绝原因

`controller.submit()` 按**固定顺序**判定，因此每个拒绝原因都明确、可解释，
并且判定依据会写入 `detail` 与日志（`joints=... reason=... basis=...`）：

| 顺序 | 拒绝原因 `RejectReason` | 判定依据 |
|------|------------------------|----------|
| 1 | `DUPLICATE_REQUEST` | `request_id` 已出现过：返回原记录，**绝不二次执行** |
| 2 | `INTERLOCK_ENGAGED` | 联锁为 `ESTOPPED`/`FAULT`，必须先复位 |
| 3 | `INVALID_PAYLOAD` | 非空列表、字段齐全且为数值、关节存在、同一请求内关节不重复、v/a>0 |
| 4 | `POSITION_OUT_OF_RANGE` | 目标位置超出该关节 `[min,max]`（边界值合法） |
| 4 | `VELOCITY_OUT_OF_RANGE` | 速度超过 `max_velocity` |
| 4 | `ACCELERATION_OUT_OF_RANGE` | 加速度超过 `max_acceleration` |
| 5 | `JOINT_CONFLICT` | 与**当前正在运行**的命令存在关节交集 |
| 6 | `QUEUE_FULL` | 等待队列已达 `queue_capacity`（不含正在运行的那条） |

设计要点：

- **幂等优先级最高**：无论原命令处于排队/运行/完成/拒绝，相同 `request_id`
  都返回同一份原记录（`DUPLICATE_REQUEST`），不会重新校验、不会再入队。
  被拒请求同样被记忆，避免"换个合法参数重提同一 id"被二次执行。
- **冲突只与正在运行的命令比较**：排队命令不参与冲突判定，统一由单工作线程
  串行化解决；目标命令只要不触碰正在运动的关节即可入队，待其完成后顺序执行。
- **越界先于冲突、冲突先于队列满**，使拒绝原因稳定、互不混淆。

---

## 5. 并发、串行化、队列溢出与超时

### 5.1 串行化

- 唯一的引擎工作线程负责：超时清理 → 推进/完成当前运动 → 从队首取下一条。
- 生产者（HTTP/测试线程）只在锁内把记录追加进 `deque`；所有记录、执行器、
  联锁的状态变更都发生在工作线程上。单一写者 + 短临界区 ⇒ 高并发下
  **不丢命令、不重复执行、状态不错乱**。
- 工作线程不在持锁时阻塞，故急停不会被工作线程延迟。

### 5.2 队列溢出（确定处理）

- 提交时若等待队列已满（`len(waiting) >= queue_capacity`），立即以
  `QUEUE_FULL` **同步拒绝**（非阻塞），消息含占用量如 `waiting queue full (4/4)`。
- 命令不丢：已接受的命令要么进入终态，要么可在 `/commands/{id}` 查到。
- 队列随命令完成而释放，之后新命令可再次被接受（可恢复）。

### 5.3 排队超时（确定处理）

- 引擎在每个虚拟时钟节拍检查：某条命令在队列中等待时间
  `now - submit_clock_ms > queue_timeout_ms` 时，将其取消为 `QUEUE_TIMEOUT`，
  并记录 `detail="queued for <n>ms > limit <m>ms"`。
- 判定基于虚拟时间，因而在测试中完全确定、可复现。

### 5.4 虚拟时钟与确定性

- `SimClock` 以 `tick_ms`（默认 10ms 虚拟时间）为步长，由后台线程推进；
  每拍真实睡眠 `clock_real_ms_per_tick`（默认 1ms，约 10 倍速）。
- 测试使用 `clock_real_ms_per_tick=0.05ms`，使千毫秒级运动仅需几毫秒真实时间，
  而**所有时序判定仍基于虚拟时钟** ⇒ 快且确定。

---

## 6. 配置项（config.MotionConfig）

| 配置项 | 含义 | 默认值 |
|--------|------|--------|
| `joint_limits[].min_position / max_position` | 关节位置硬限位（度，闭区间） | j0 ±180 / j1 ±90 / j2 ±120 |
| `joint_limits[].max_velocity` | 最大速度（度/秒） | 100 / 80 / 120 |
| `joint_limits[].max_acceleration` | 最大加速度（度/秒²） | 200 / 160 / 240 |
| `queue_capacity` | 等待队列容量（不含运行中的命令）；满则 `QUEUE_FULL` | 8 |
| `queue_timeout_ms` | 排队最大等待（虚拟毫秒）；超过则 `QUEUE_TIMEOUT` | 5000 |
| `clock_tick_ms` | 每个时钟拍推进的虚拟毫秒 | 10 |
| `clock_real_ms_per_tick` | 每拍真实睡眠毫秒（控制仿真倍速；0 最快） | 1.0 |

单位：位置=度，速度=度/秒，加速度=度/秒²，时间=毫秒。
配置为 frozen dataclass，运行期不变；测试构造自己的 `MotionConfig` 获得快速确定时序。
`GET /config` 可查看当前生效值。

---

## 7. HTTP 接口

| 方法/路径 | 说明 |
|-----------|------|
| `GET /health` | 存活检查 + 当前联锁状态 |
| `GET /config` | 当前关节限位与队列/时钟配置 |
| `POST /commands` | 提交运动命令（见下）；接受 202，拒绝 409，坏包 400/422 |
| `GET /commands/{request_id}` | 查询单条命令生命周期；未知 id 返回 404 |
| `GET /status` | 全量快照：状态、时钟、队列、当前命令、各关节位置、全部命令 |
| `POST /safety/estop` | 急停 |
| `POST /safety/fault` | 外部故障 |
| `POST /safety/reset` | 复位到 IDLE |

提交请求体：

```json
{
  "request_id": "uuid-or-idempotency-key",
  "joints": [
    {"joint_id": 0, "target_position": 60.0, "velocity": 50.0, "acceleration": 100.0}
  ]
}
```

拒绝响应（HTTP 409）示例：

```json
{
  "accepted": false,
  "request_id": "x",
  "status": "REJECTED",
  "reject_reason": "POSITION_OUT_OF_RANGE",
  "message": "joint 0: target_position 999.0 outside [-180.0, 180.0]",
  "command": { "...": "完整命令记录" }
}
```

---

## 8. 本地复现与验证

运行全部测试（日志会打印请求参数与判定依据）：

```bash
uv sync
uv run pytest -q
```

启动本地服务手动验证：

```bash
uv run python main.py            # http://127.0.0.1:8000
curl -s http://127.0.0.1:8000/health
curl -s -X POST http://127.0.0.1:8000/commands -H 'Content-Type: application/json' \
  -d '{"request_id":"demo","joints":[{"joint_id":0,"target_position":60,"velocity":50,"acceleration":100}]}'
curl -s http://127.0.0.1:8000/commands/demo
curl -s -X POST http://127.0.0.1:8000/safety/estop
curl -s -X POST http://127.0.0.1:8000/safety/reset
```

测试覆盖：

- `tests/test_validation.py`  位置/速度/加速度越界、坏包、边界值，按原因区分 + 日志断言
- `tests/test_idempotency.py` 排队/运行/完成/被拒各阶段重复提交均不二次执行
- `tests/test_concurrency.py` 多线程并发提交、运行区间不重叠、QUEUE_FULL、60 线程压力
- `tests/test_safety.py`     运动中急停就地冻结、清空队列、联锁拒绝、故障、复位、排队超时
- `tests/test_api.py`        HTTP 端到端正/负向流程
