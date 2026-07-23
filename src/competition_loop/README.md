# competition_loop

**负责人**: 黄誉铭（团队负责人、任务状态机、结果记录、系统集成）

## 模块目标

比赛闭环主节点：管理比赛状态机、计时、环境接口合规校验、结果记录和模块协调。
不实现真实探索决策或感知算法。

## 输入

| 输入 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/competition/state` | `CompetitionState` | 本模块发布 | 全局比赛状态（latch） |
| `/competition/start` | `std_srvs/Trigger` | 外部调用 | 手动启动比赛 |
| `/competition/finish` | `std_srvs/Trigger` | 外部调用或内部超时 | 主动完成比赛 |
| `team_scene_info.json` | 文件 | 场景生成器 | **唯一**允许读取的场景文件 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/state` | `CompetitionState` | 所有模块 | 全局状态广播 |
| `/competition/control_enabled` | `std_msgs/Bool` | `cmd_vel_guard_node` | 运动控制使能 |
| `/competition/emergency_stop` | `std_msgs/Bool` | `cmd_vel_guard_node` | 紧急停车 |
| `results/detected_danger.json` | 文件 | 裁判评估 | 最终检测结果 |

**本模块不直接发布 `/cmd_vel`。**

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~max_time` | float | 600.0 | 最大探索时间（秒） |
| `~start_delay` | float | 5.0 | 启动后等待传感器稳定的延迟 |
| `~auto_start` | bool | true | 是否自动启动比赛 |
| `~competition_mode` | bool | true | 正式比赛模式 / 测试模式 |
| `~result_path` | str | `results/detected_danger.json` | 结果输出路径 |

## 当前状态

**v0.1.0 — 骨架**

- [x] 状态机实现（WAITING → RUNNING → FINISHED/ERROR）
- [x] 时间管理器（基于 ROS 时间，支持 /clock）
- [x] 接口合规校验（forbidden topics / files）
- [x] 结果写入（原子替换、frame 校验）
- [x] 门/电梯服务封装
- [x] 安全信号发布（control_enabled / emergency_stop）
- [ ] 模块状态聚合与协调
- [ ] 与 perception 候选结果的实时集成
- [ ] 自动乘梯策略
- [ ] 完整端到端测试

**默认安全行为**：
- 始终从 `WAITING` 状态启动
- `WAITING` 状态下 `control_enabled=false`、`emergency_stop=true`
- 未满足依赖条件时拒绝进入 `RUNNING`

## 待实现 TODO

1. **模块状态聚合**：订阅所有 `/competition/*/status` 话题，汇总模块就绪状态
2. **候选收集**：订阅 `/competition/hazard_candidates`，实时交给 result_writer
3. **集成测试**：与真实传感器和控制器的联调
4. **自动计时启动**：检测到控制器进入 RL `/cmd_vel` 模式后自动开始计时

## 禁止事项

- 不得订阅 `/Odometry_gazebo`
- 不得订阅 `/ground_truth/*`
- 不得读取 `results/danger_truth.json`
- 不得读取 `generated_building/danger_truth.json`
- 不得调用 Gazebo get_model_state
- 不得直接发布 `/cmd_vel`
- 不得在骨架模式下发布虚假的 `valid=true` 或非零速度

## 最小运行命令

```bash
source devel/setup.bash

# 仅启动比赛闭环（不依赖 Gazebo，可用于测试）
rosrun competition_loop competition_loop_node \
  _competition_mode:=false \
  _auto_start:=false \
  _max_time:=60.0

# 手动触发开始
rosservice call /competition/start

# 手动触发完成
rosservice call /competition/finish
```

## 验收标准

- [x] `WAITING → RUNNING` 转换需通过接口合规检查
- [x] `RUNNING` 状态发布 `control_enabled=true`
- [x] `max_time` 到达 → `FINISHED`
- [x] `FINISHED`/`ERROR` → 请求安全停车
- [x] 结果输出格式严格匹配 `{"exploration_time": ..., "detected_danger_sources": [...]}`
- [x] 只接受 `frame_id="world"` 且 `position_valid=true` 的候选
- [x] 使用临时文件 + 原子替换写入 JSON
- [x] 不读取任何 forbidden 文件
- [ ] 完整工作区编译通过
- [ ] 端到端集成测试通过

## 需要向上下游成员交付的内容

- 纪雯瀚：`/competition/state` 用于了解比赛阶段；`/competition/start` 用于触发准备
- 鲍语菲：`/competition/hazard_candidates` 的反向接口（result_writer 消费感知输出）
- 杨渊浩：`/competition/control_enabled` 和 `/competition/emergency_stop` 用于安全控制
- 闫雨萱：`/competition/state` 用于了解比赛阶段
- FFFA：`/competition/state` 和 `results/detected_danger.json` 用于证据收集
