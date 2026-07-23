# competition_control

**负责人**: 杨渊浩（运动控制与执行）

## 模块目标

安全地执行路径跟踪，并通过唯一 `/cmd_vel` 发布者 (`cmd_vel_guard_node`)
提供速度限幅、超时停车和 watchdog 保护。

**关键原则**：
1. 只有 `cmd_vel_guard_node` 可以发布 `/cmd_vel`
2. 所有其他控制代码只能发布 `/competition/raw_cmd_vel`
3. 路径跟踪代码不直接发布 `/cmd_vel`

## 节点

### 1. `cmd_vel_guard_node` — 安全网关（**本轮已实现**）

这是整个系统唯一允许发布 `/cmd_vel` 的节点。

**功能**：
- [x] 订阅 `/competition/raw_cmd_vel`（原始速度指令）
- [x] 订阅 `/competition/control_enabled`（运动使能）
- [x] 订阅 `/competition/emergency_stop`（紧急停车）
- [x] 独占发布 `/cmd_vel`
- [x] 默认 `control_enabled=false`
- [x] 默认 `emergency_stop=true`（安全初态）
- [x] 超时检测：超过 `cmd_vel_timeout` 无新 raw 指令 → 持续发布零速度
- [x] 参数化限幅：`linear.x`、`linear.y`、`angular.z`
- [x] 关闭时发布多次零速度
- [x] 不转发 Twist 中未使用的自由度
- [x] 单元测试覆盖：禁用、急停、超时、限幅、正常转发

### 2. `path_follower_node` — 路径跟踪（骨架）

仅占位，默认发布零速度到 `/competition/raw_cmd_vel`。

## 输入

| 输入 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/competition/raw_cmd_vel` | `Twist` | path_follower 或其他算法 | 原始速度指令 |
| `/competition/control_enabled` | `Bool` | competition_loop | 运动使能 |
| `/competition/emergency_stop` | `Bool` | competition_loop | 紧急停车 |
| `/competition/navigation_goal` | `PoseStamped` | planning | 导航目标 |
| `/competition/path` | `Path` | planning | 规划路径 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/cmd_vel` | `Twist` | junior_ctrl | **独占发布** |
| `/competition/raw_cmd_vel` | `Twist` | cmd_vel_guard | 原始速度（path_follower 输出） |
| `/competition/control/status` | `ModuleStatus` | competition_loop | 控制模块状态 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~cmd_vel_timeout` | float | 0.5 | raw_cmd_vel 超时（秒） |
| `~max_linear_x` | float | 0.5 | 前进速度限幅（m/s） |
| `~max_linear_y` | float | 0.3 | 横向速度限幅（m/s） |
| `~max_angular_z` | float | 1.0 | 转向速度限幅（rad/s） |

## 当前状态

**v0.1.0**

- [x] `cmd_vel_guard_node` 完整实现（本轮唯一功能节点）
- [x] 单元测试：禁用、急停、超时、限幅、正常转发、未使用 DOF 归零
- [x] `path_follower_node` 骨架占位（零速度）
- [ ] 路径跟踪算法（Pure Pursuit / DWA / MPC）
- [ ] 目标到达判断
- [ ] 路径阻塞检测与重规划请求
- [ ] 多楼层路径执行（开/关门 + 乘梯配合）

## 禁止事项

- 禁止 path_follower_node 直接发布 `/cmd_vel`
- 禁止绕过 `cmd_vel_guard` 的安全检查
- 禁止在骨架模式下发布非零速度
- 不得订阅 `/Odometry_gazebo` 或 `/ground_truth/*`

## 最小运行命令

```bash
source devel/setup.bash

# 仅启动安全网关
rosrun competition_control cmd_vel_guard_node

# 仅启动路径跟踪（骨架）
rosrun competition_control path_follower_node
```

## 验收标准

- [x] `cmd_vel_guard` 是唯一 `/cmd_vel` 发布者（代码审计确认）
- [x] 禁用时输出零速
- [x] 急停时输出零速
- [x] 超时时输出零速
- [x] 超限时正确限幅
- [x] 关闭时发布多次零速
- [x] 单元测试全部通过
- [ ] 与 real 控制器联调通过

## 需要向上下游成员交付的内容

- 闫雨萱（planning）：路径跟踪状态，用于重规划
- 黄誉铭（loop）：`/competition/control/status` 用于模块就绪检查
- 所有人：`/cmd_vel` 的唯一可靠发布者
