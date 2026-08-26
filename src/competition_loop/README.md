# competition_loop

**负责人**: 黄誉铭（团队负责人、任务状态机、结果记录、系统集成）

## 模块目标

比赛闭环主节点：管理比赛状态机、计时、环境接口合规校验、结果记录和模块协调。
不实现真实探索决策或感知算法。

## 输入

| 输入 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/competition/state` | `CompetitionState` | 本模块发布 | 全局比赛状态（latch + 周期性广播） |
| `/competition/start` | `std_srvs/Trigger` | 外部调用 | 手动启动比赛（须满足全部安全条件） |
| `/competition/finish` | `std_srvs/Trigger` | 外部调用或内部超时 | 主动完成比赛 |
| `team_scene_info.json` | 文件 | 场景生成器 | **唯一**允许读取的场景文件 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/state` | `CompetitionState` | 所有模块 | 全局状态广播（latch，10 Hz tick） |
| `/competition/control_enabled` | `std_msgs/Bool` | `cmd_vel_guard_node` | 运动控制使能 |
| `/competition/emergency_stop` | `std_msgs/Bool` | `cmd_vel_guard_node` | 紧急停车 |
| `results/detected_danger.json` | 文件 | 裁判评估 | 最终检测结果 |

**本模块不直接发布 `/cmd_vel`。**

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~max_time` | float | 600.0 | 最大探索时间（秒，必须为有限正数） |
| `~start_delay` | float | 5.0 | 校验通过后等待传感器初始化的延迟（秒） |
| `~auto_start` | bool | true | 延迟结束后是否自动进入 RUNNING |
| `~competition_mode` | bool | true | 正式比赛模式 / 测试模式 |
| `~workspace_root` | str | `/workspace/SimEnv` | 工作空间根目录（用于解析 team_scene_info.json） |
| `~result_path` | str | `results/detected_danger.json` | 结果输出路径 |

## 当前状态

**v0.1.0 — 生命周期修复版**

- [x] 状态机实现（WAITING → RUNNING → FINISHED/ERROR）
- [x] 时间管理器（基于 ROS 时间，支持 /clock，含时间回退检测）
- [x] 接口合规校验（显式状态标志 / 禁止话题检查 / 禁止文件警告）
- [x] 结果写入（原子替换、frame 校验）
- [x] 门/电梯服务封装
- [x] 安全信号发布（control_enabled / emergency_stop）
- [x] 显式校验状态标志（不再以"错误列表为空"代表"校验已通过"）
- [x] start_delay 真实语义（校验通过后计时，时钟未就绪前保持 WAITING）
- [x] /clock 未就绪行为（等待 ROS 时间有效后再开始延迟计时）
- [x] 仿真时间回退检测（WAITING 阶段重置 delay，RUNNING 阶段进入 ERROR）
- [x] /competition/state 持续广播（latch + 10 Hz tick，状态转换后立即发布）
- [x] workspace_root 参数（可预测路径，不再依赖 Path.cwd()）
- [x] Mock 节点正式/测试模式硬隔离
- [x] 参数合法性校验（无效参数进入安全失败状态）
- [ ] 模块状态聚合与协调
- [ ] 与 perception 候选结果的实时集成
- [ ] 自动乘梯策略
- [ ] 完整端到端测试

**默认安全行为**：
- 始终从 `WAITING` 状态启动
- `WAITING` 状态下 `control_enabled=false`、`emergency_stop=true`
- 未满足依赖条件时拒绝进入 `RUNNING`
- 参数非法时进入 `ERROR` 状态并保持安全信号

## 校验完成状态

`is_ready()` 不再等同于"错误列表为空"。显式状态标志：

| 标志 | 说明 |
|------|------|
| `scene_info_loaded` | team_scene_info.json 加载成功 |
| `forbidden_topic_check_completed` | 禁止话题检查已执行 |
| `forbidden_topic_check_passed` | 禁止话题检查已通过 |

`is_ready()` 要求 `scene_info_loaded=True` **且** `forbidden_topic_check_completed=True` **且** `forbidden_topic_check_passed=True`。

测试模式下，`check_forbidden_topics()` 显式记录 `completed=True, passed=True` 而不查询 ROS Master。

## start_delay 真实语义

1. 环境校验全部通过后才记录 `ready_since` 时间戳
2. 从 `ready_since` 开始计算 `start_delay`（秒）
3. 延迟期间 `control_enabled=false`, `emergency_stop=true`
4. 延迟结束后，若 `auto_start=true`，自动进入 `RUNNING`
5. 若 `/clock` 仍未就绪，不开始计时
6. 若 ROS 时间回退，重置 `ready_since` 重新计时

## /clock 未就绪行为

- `/clock` 未就绪时（`rospy.Time.now().is_zero()`），`_check_clock_ready()` 返回 `False`
- WAITING 状态下持续等待，不记录 `ready_since`，不启动延迟计时
- 手动 `/competition/start` 也会被拒绝（返回 `success=false`）

## 时间回退行为

- WAITING 阶段：重置 `ready_since` 和 `start_delay_done`，重新开始延迟计时
- RUNNING 阶段：立即进入 `ERROR`，急停，不再发布 `control_enabled=true`

## /competition/state 持续广播

- 通过 `rospy.Timer` 以 10 Hz tick 持续发布 `CompetitionState`
- 保持 `latch=True`，新订阅者可立即获取最新状态
- 状态转换后在同一 tick 中立即发布（不等待下一次定时触发）
- 正确更新 `state`、`elapsed_time`、`detected_count`、`detail`

## /competition/start 手动开始安全条件

`/competition/start` 服务不能绕过以下任何条件：

1. 当前状态必须为 `WAITING`
2. 场景信息必须已加载
3. 禁止话题检查必须已完成并通过
4. ROS 时间必须有效
5. start_delay 必须已完成

任一条件不满足时返回 `success=false`，并在 message 中说明缺失条件。

## 安全信号表

| 状态 | control_enabled | emergency_stop |
|---|---:|---:|
| WAITING | false | true |
| RUNNING | true | false |
| RETURNING | Return-mode planning is enabled only with `~return_on_first_detection=true`; path remains on `/competition/path` | Fresh valid localization + follower `SUCCEEDED` + persistent start-distance gate |
| FINISHED | false | true |
| ERROR | false | true |

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
  _max_time:=60.0 \
  _workspace_root:=/workspace/SimEnv

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
- [x] `is_ready()` 使用显式状态标志，不依赖错误列表为空
- [x] `start_delay` 在校验通过后正确计时
- [x] `/clock` 未就绪时保持 WAITING
- [x] 仿真时间回退时进入安全状态
- [x] `/competition/state` 通过 tick 持续广播
- [ ] 完整工作区编译通过
- [ ] 端到端集成测试通过

## 需要向上下游成员交付的内容

- 纪雯瀚：`/competition/state` 用于了解比赛阶段；`/competition/start` 用于触发准备
- 鲍语菲：`/competition/hazard_candidates` 的反向接口（result_writer 消费感知输出）
- 杨渊浩：`/competition/control_enabled` 和 `/competition/emergency_stop` 用于安全控制
- 闫雨萱：`/competition/state` 用于了解比赛阶段
- FFFA：`/competition/state` 和 `results/detected_danger.json` 用于证据收集
