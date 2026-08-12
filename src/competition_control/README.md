# competition_control

**负责人**：杨渊浩（运动控制与执行）

## 模块目标

接收 planning 发布的路径和 localization 发布的里程计，生成原始速度，
再由安全网关统一检查并发布最终速度与控制模块权威状态。

```text
planning_node
  -> /competition/path
  -> path_follower_node
  -> /competition/raw_cmd_vel
  -> cmd_vel_guard_node
  -> /cmd_vel
```

## 唯一发布者约束

- `path_follower_node` 是
  `/competition/control/path_follower_status` 的唯一发布者。
- `cmd_vel_guard_node` 是 `/competition/control/status` 的唯一发布者。
- `cmd_vel_guard_node` 是 `/cmd_vel` 的唯一发布者。
- `path_follower_node` 只发布 `/competition/raw_cmd_vel`，不得发布
  `/cmd_vel`。
- `/competition/control/path_follower_status` 是
  `competition_control` 包内部接口。
- `/competition/control/status` 是提供给 `competition_loop` 等下游模块的
  控制模块权威状态。

未修改 `competition_interfaces/ModuleStatus.msg` 或其他冻结公共接口。

## 节点

### `path_follower_node`

订阅：

| 话题 | 类型 | 来源 |
|---|---|---|
| `/competition/path` | `nav_msgs/Path` | planning |
| `/competition/localization/odom` | `nav_msgs/Odometry` | localization |

发布：

| 话题 | 类型 | 消费者 | 说明 |
|---|---|---|---|
| `/competition/raw_cmd_vel` | `geometry_msgs/Twist` | `cmd_vel_guard_node` | 未经过安全网关的原始速度 |
| `/competition/control/path_follower_status` | `competition_interfaces/ModuleStatus` | `cmd_vel_guard_node` | 包内部路径跟踪状态 |

行为：

- 缺少路径或里程计时持续输出零速度和 `STARTING, valid=false`。
- 路径、里程计或内部计算超时时输出零速度和
  `TIMEOUT, valid=false`。
- 路径与里程计 `frame_id` 非空且一致时才执行；当前不进行 TF 转换。
- 拒绝 `NaN`、`Inf`、空坐标系和零范数四元数。
- 空路径代表撤销，立即清除旧路径并停车。
- 新路径安全替换旧路径，并跳过机器人附近已经到达的起始点。
- 正常跟踪时报告 `RUNNING, valid=true`。
- 到达路径终点时输出零速度并报告 `SUCCEEDED, valid=true`。

### `cmd_vel_guard_node`

订阅：

| 话题 | 类型 | 来源 |
|---|---|---|
| `/competition/raw_cmd_vel` | `geometry_msgs/Twist` | `path_follower_node` |
| `/competition/control/path_follower_status` | `competition_interfaces/ModuleStatus` | `path_follower_node` |
| `/competition/control_enabled` | `std_msgs/Bool` | `competition_loop` |
| `/competition/emergency_stop` | `std_msgs/Bool` | `competition_loop` |

发布：

| 话题 | 类型 | 消费者 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘控制器 | 最终速度，唯一发布者 |
| `/competition/control/status` | `competition_interfaces/ModuleStatus` | `competition_loop` 等 | 控制模块权威状态，唯一发布者 |

安全网关拒绝非有限速度，对使用的三个自由度进行限幅，并保证其他 Twist
自由度为零。节点关闭时会连续发布多次零速度。

## 参数

### `path_follower_node`

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `~max_linear` | `0.25` | 最大前进速度（m/s） |
| `~max_angular` | `0.8` | 最大角速度（rad/s） |
| `~goal_tolerance` | `0.2` | 航点到达容差（m） |
| `~path_timeout` | `5.0` | 路径接收超时（s） |
| `~odom_timeout` | `1.0` | 里程计接收超时（s） |

### `cmd_vel_guard_node`

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `~cmd_vel_timeout` | `0.5` | `/competition/raw_cmd_vel` 超时（s） |
| `~path_follower_status_timeout` | `1.0` | 内部路径跟踪状态超时（s） |
| `~max_linear_x` | `0.5` | `linear.x` 限幅（m/s） |
| `~max_linear_y` | `0.3` | `linear.y` 限幅（m/s） |
| `~max_angular_z` | `1.0` | `angular.z` 限幅（rad/s） |

上述参数必须为有限且大于零的数值。

## 状态优先级

`cmd_vel_guard_node` 按以下顺序决定最终速度和公共状态：

1. 急停：零速度，`BLOCKED, valid=false`。
2. 控制禁用：零速度，`READY, valid=false`。
3. 非法 `raw_cmd_vel`：零速度，`FAILED, valid=false`。
4. `raw_cmd_vel` 从未收到、超时或 ROS 时间回退：零速度，
   `TIMEOUT, valid=false`。
5. 从未收到路径跟踪状态：零速度，`STARTING, valid=false`。
6. 路径跟踪状态超时或 ROS 时间回退：零速度，
   `TIMEOUT, valid=false`。
7. 路径跟踪器报告 `FAILED`、`TIMEOUT`、`BLOCKED` 或
   `valid=false`：零速度，并汇总相应失败状态和 `detail`。
8. 路径跟踪器报告 `SUCCEEDED, valid=true`：零速度，并发布
   `SUCCEEDED, valid=true`。
9. 只有路径跟踪器报告 `RUNNING, valid=true` 且所有安全检查通过时，
   才限幅转发原始速度并发布 `RUNNING, valid=true`。

因此，路径跟踪器的 `RUNNING` 永远不能覆盖急停、控制禁用、非法速度或
安全网关超时。

## 自动测试

测试不需要启动 roscore：

```bash
python3 src/competition_control/test/test_cmd_vel_guard.py
python3 src/competition_control/test/test_path_follower.py
```

覆盖内容包括：

- 唯一发布者和内部状态订阅关系；
- 从未收到内部状态与内部状态超时；
- Gazebo/ROS 仿真时间回退；
- `RUNNING`、`SUCCEEDED`、失败状态和 `valid=false`；
- 急停、禁用、非法速度、raw 速度超时与限幅；
- 空路径撤销、坐标系不一致、非法数值和非法四元数；
- 正常跟踪和终点停车。

## ROS 联调验收

```bash
rostopic info /competition/control/status
rostopic info /competition/control/path_follower_status
rostopic info /competition/raw_cmd_vel
rostopic info /cmd_vel
```

预期：

- `/competition/control/status` 只有 `cmd_vel_guard_node` 发布。
- `/competition/control/path_follower_status` 只有
  `path_follower_node` 发布。
- `/competition/raw_cmd_vel` 由 `path_follower_node` 发布。
- `/cmd_vel` 只有 `cmd_vel_guard_node` 发布。

禁止正式算法订阅 `/Odometry_gazebo`、`/ground_truth/*`，或读取真值接口
绕过 localization。
