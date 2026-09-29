# 本地多关节机器人运动控制（模拟）

一套**完全本地、无外部硬件/服务依赖**的多关节机器人运动命令与执行流程。
执行器状态为模拟数据，时间由确定性虚拟时钟驱动；并发下发、急停、参数越界、
排队溢出/超时的结果都可预期、可解释，并可通过测试或本地 HTTP 服务复现。

## 能力

- **命令提交与状态查询**：位置/速度/加速度越界、与运行中运动的关节冲突均被拒绝，
  拒绝原因可区分；同一 `request_id` 重复提交绝不执行两次（幂等）。
- **安全联锁与状态流转**：`IDLE/RUNNING/ESTOPPED/FAULT` 四态机；急停或外部故障
  立即就地冻结运动并清空队列，复位前拒绝一切新命令，复位后回到确定的初始 `IDLE`。
- **并发串行化与排队**：全机同一时刻仅一段运动执行；FIFO 队列、容量满同步拒绝
  （`QUEUE_FULL`）、排队超时确定取消（`QUEUE_TIMEOUT`）；单工作线程保证高压下
  不丢命令、不重复、状态不错乱。
- **可观测**：每次判定都记录请求参数与依据；`GET /status` 给出全量快照。

## 快速开始

```bash
uv sync
uv run pytest -q            # 37 个测试，约 1~2 秒
uv run python main.py       # 启动 HTTP 服务 http://127.0.0.1:8000
```

提交一条运动命令：

```bash
curl -s -X POST http://127.0.0.1:8000/commands \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"demo","joints":[{"joint_id":0,"target_position":60,"velocity":50,"acceleration":100}]}'
curl -s http://127.0.0.1:8000/commands/demo
curl -s -X POST http://127.0.0.1:8000/safety/estop
curl -s -X POST http://127.0.0.1:8000/safety/reset
```

## 拒绝原因（reject_reason）

`DUPLICATE_REQUEST` · `INTERLOCK_ENGAGED` · `INVALID_PAYLOAD` ·
`POSITION_OUT_OF_RANGE` · `VELOCITY_OUT_OF_RANGE` · `ACCELERATION_OUT_OF_RANGE` ·
`JOINT_CONFLICT` · `QUEUE_FULL`

命令终态：`COMPLETED` · `REJECTED` · `EMERGENCY_STOPPED` · `FAULTED` · `QUEUE_TIMEOUT`。

## 文档

详细状态机、安全规则、并发模型、配置项与复现方式见
[`docs/design.md`](docs/design.md)。
