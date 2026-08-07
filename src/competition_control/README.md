# competition_control

**负责人**：杨渊浩（运动控制与执行）

## 模块目标

接收 Planning 发布的路径与状态，以及 Localization 发布的里程计与状态，
生成未经最终安全检查的原始速度。

原始速度和 PathFollower 内部状态再交给 `cmd_vel_guard_node`，
由安全网关统一决定最终 `/cmd_vel` 和控制模块权威状态
`/competition/control/status`。

```text
Planning
  ├─ /competition/path
  └─ /competition/planning/status
                │
                v
        path_follower_node
                ^
                │
Localization
  ├─ /competition/localization/odom
  └─ /competition/localization/status
                │
                v
  /competition/raw_cmd_vel
  /competition/control/path_follower_status
                │
                v
        cmd_vel_guard_node
          ^             ^
          │             │
/competition/       /competition/
control_enabled     emergency_stop
          │             │
          └──────┬──────┘
                 v
             /cmd_vel
             /competition/control/status
```

## 唯一发布者约束

* `path_follower_node` 是
  `/competition/control/path_follower_status` 的唯一发布者。
* `path_follower_node` 只发布 `/competition/raw_cmd_vel` 和
  `/competition/control/path_follower_status`。
* `path_follower_node` 不得发布 `/cmd_vel`。
* `path_follower_node` 不得发布 `/competition/control/status`。
* `cmd_vel_guard_node` 是 `/cmd_vel` 的唯一发布者。
* `cmd_vel_guard_node` 是 `/competition/control/status` 的唯一发布者。
* `/competition/control/path_follower_status` 是
  `competition_control` 包内部接口。
* `/competition/control/status` 是提供给 `competition_loop`
  等下游模块的控制模块权威状态。

未修改 `competition_interfaces/ModuleStatus.msg`
或其他冻结公共接口。

## 节点

### `path_follower_node`

订阅：

| 话题                                 | 类型                                    | 来源           |
| ---------------------------------- | ------------------------------------- | ------------ |
| `/competition/path`                | `nav_msgs/Path`                       | planning     |
| `/competition/planning/status`     | `competition_interfaces/ModuleStatus` | planning     |
| `/competition/localization/odom`   | `nav_msgs/Odometry`                   | localization |
| `/competition/localization/status` | `competition_interfaces/ModuleStatus` | localization |

发布：

| 话题                                          | 类型                                    | 消费者                  | 说明           |
| ------------------------------------------- | ------------------------------------- | -------------------- | ------------ |
| `/competition/raw_cmd_vel`                  | `geometry_msgs/Twist`                 | `cmd_vel_guard_node` | 未经过安全网关的原始速度 |
| `/competition/control/path_follower_status` | `competition_interfaces/ModuleStatus` | `cmd_vel_guard_node` | 包内部路径跟踪状态    |

#### 输入有效条件

只有同时满足以下条件时，PathFollower 才允许输出非零原始速度：

* 已收到且持续收到 `/competition/planning/status`；
* Planning 状态为 `valid=true`；
* 已收到非空且合法的 `/competition/path`；
* Path 未超时；
* Path 的 `header.frame_id` 为 `world`；
* Path 中每个 `PoseStamped.header.frame_id` 均为 `world`；
* 已收到且持续收到 `/competition/localization/status`；
* Localization 状态为 `valid=true`；
* 已收到合法的 `/competition/localization/odom`；
* Odom 未超时；
* Odom 的 `header.frame_id` 为 `world`；
* Odom 的 `child_frame_id` 为 `base`；
* 输入数据及控制计算中不存在 `NaN` 或 `Inf`；
* Odom 四元数合法且范数非零；
* ROS 时间未发生回退。

任一必要条件不满足时，都必须输出零
`/competition/raw_cmd_vel`，并报告非运行状态。

当前 PathFollower 不进行 TF 转换。

#### Path 行为

* 空 Path 表示显式撤销当前路径，立即清除旧 Path、重置目标点并停车。
* 收到新 Path 时先使旧 Path 失效，再校验和缓存新 Path。
* 新 Path 合法后，根据机器人当前位置跳过已经处于
  `goal_tolerance` 内的起始航点。
* 正常跟踪时报告 `RUNNING, valid=true`。
* 到达全部航点后输出零速度并报告
  `SUCCEEDED, valid=true`。

#### Planning 状态行为

* `/competition/planning/status` 为 `valid=false` 时立即停车。
* Planning 从正常状态进入 invalid 时，失效此前生成的旧 Path。
* Planning 持续 invalid 时，即使收到候选新 Path，也不得执行。
* Planning 恢复为 `valid=true` 后，只允许使用当前有效 generation
  中的新 Path，不得恢复使用失效前的旧 Path。
* Planning status 超时后，旧 Path 失效并停车。

#### Localization 状态行为

* `/competition/localization/status` 为 `valid=false` 时立即停车。
* Localization 从正常状态进入 invalid 时，失效此前的旧 Odom 和旧 Path。
* Localization 持续 invalid 时，新收到的数据只能作为候选数据，
  不得产生非零控制输出。
* Localization 恢复后，不得出现
  “新 Odom + 失效前旧 Path”继续运动的情况。
* Localization status 超时后，旧 Odom 和旧 Path 都失效并停车。

#### 超时和 ROS 时间回退

PathFollower 分别检查：

* Path 接收超时；
* Odom 接收超时；
* Planning status 接收超时；
* Localization status 接收超时。

发生 ROS 时间回退时，旧 generation 输入全部失效并停车。

### `cmd_vel_guard_node`

订阅：

| 话题                                          | 类型                                    | 来源                   |
| ------------------------------------------- | ------------------------------------- | -------------------- |
| `/competition/raw_cmd_vel`                  | `geometry_msgs/Twist`                 | `path_follower_node` |
| `/competition/control/path_follower_status` | `competition_interfaces/ModuleStatus` | `path_follower_node` |
| `/competition/control_enabled`              | `std_msgs/Bool`                       | `competition_loop`   |
| `/competition/emergency_stop`               | `std_msgs/Bool`                       | `competition_loop`   |

发布：

| 话题                            | 类型                                    | 消费者                  | 说明             |
| ----------------------------- | ------------------------------------- | -------------------- | -------------- |
| `/cmd_vel`                    | `geometry_msgs/Twist`                 | 底盘控制器                | 最终速度，唯一发布者     |
| `/competition/control/status` | `competition_interfaces/ModuleStatus` | `competition_loop` 等 | 控制模块权威状态，唯一发布者 |

安全网关拒绝非有限速度，对使用的三个自由度进行限幅，并保证其他 Twist
自由度为零。节点关闭时会连续发布多次零速度。

只有 PathFollower 报告 `RUNNING, valid=true`，
且 Guard 的所有安全检查都通过时，才允许输出非零 `/cmd_vel`。

## 参数

### `path_follower_node`

| 参数                             |    默认值 | 说明                          |
| ------------------------------ | -----: | --------------------------- |
| `~control_frequency`           | `10.0` | PathFollower 控制循环频率（Hz）     |
| `~max_linear`                  | `0.25` | 最大前进速度（m/s）                 |
| `~max_angular`                 |  `0.8` | 最大角速度（rad/s）                |
| `~goal_tolerance`              |  `0.2` | 航点到达容差（m）                   |
| `~path_timeout`                |  `5.0` | Path 接收超时（s）                |
| `~odom_timeout`                |  `1.0` | Odom 接收超时（s）                |
| `~planning_status_timeout`     |  `1.0` | Planning status 接收超时（s）     |
| `~localization_status_timeout` |  `1.0` | Localization status 接收超时（s） |

### `cmd_vel_guard_node`

| 参数                              |   默认值 | 说明                               |
| ------------------------------- | ----: | -------------------------------- |
| `~cmd_vel_timeout`              | `0.5` | `/competition/raw_cmd_vel` 超时（s） |
| `~path_follower_status_timeout` | `1.0` | 内部路径跟踪状态超时（s）                    |
| `~max_linear_x`                 | `0.5` | `linear.x` 限幅（m/s）               |
| `~max_linear_y`                 | `0.3` | `linear.y` 限幅（m/s）               |
| `~max_angular_z`                | `1.0` | `angular.z` 限幅（rad/s）            |

上述参数必须为有限且大于零的数值。

## 状态优先级

`cmd_vel_guard_node` 按以下顺序决定最终速度和公共状态：

1. 急停：零速度，`BLOCKED, valid=false`。
2. 控制禁用：零速度，`READY, valid=false`。
3. 非法 `raw_cmd_vel`：零速度，`FAILED, valid=false`。
4. `raw_cmd_vel` 从未收到、超时或 ROS 时间回退：
   零速度，`TIMEOUT, valid=false`。
5. 从未收到路径跟踪状态：
   零速度，`STARTING, valid=false`。
6. 路径跟踪状态超时或 ROS 时间回退：
   零速度，`TIMEOUT, valid=false`。
7. 路径跟踪器报告 `FAILED`、`TIMEOUT`、`BLOCKED`
   或 `valid=false`：
   零速度，并汇总相应失败状态和 `detail`。
8. 路径跟踪器报告 `SUCCEEDED, valid=true`：
   零速度，并发布 `SUCCEEDED, valid=true`。
9. 只有路径跟踪器报告 `RUNNING, valid=true`
   且所有安全检查通过时，
   才限幅转发原始速度并发布 `RUNNING, valid=true`。

因此，路径跟踪器的 `RUNNING` 永远不能覆盖急停、控制禁用、
非法速度或安全网关超时。

## 自动测试

测试不需要启动 roscore：

```bash
python3 src/competition_control/test/test_cmd_vel_guard.py
python3 src/competition_control/test/test_path_follower.py
```

当前测试覆盖包括：

* 节点正式订阅和发布关系；
* `/cmd_vel` 与 `/competition/control/status` 的唯一发布者约束；
* Planning status 缺失、invalid、timeout 和恢复；
* Localization status 缺失、invalid、timeout 和恢复；
* invalid/recovery 后旧 Path 不得错误复用；
* invalid 状态下候选新数据的缓存与恢复；
* Path timeout；
* Odom timeout；
* ROS 时间回退；
* 空 Path 撤销；
* Path `world` frame 校验；
* Path 中每个 PoseStamped 的 `world` frame 校验；
* Odom `world -> base` frame 校验；
* `NaN`、`Inf` 和非法四元数；
* 正常路径跟踪与终点停车；
* Guard 急停与控制禁用；
* 非法 raw velocity；
* raw velocity timeout；
* follower status timeout；
* 速度限幅与安全状态优先级。

当前回归结果：

```text
test_path_follower.py: 26 tests passed
test_cmd_vel_guard.py: 14 tests passed
```

## ROS 联调验收

可使用以下命令检查关键话题的发布者和订阅关系：

```bash
rostopic info /competition/control/status
rostopic info /competition/control/path_follower_status
rostopic info /competition/raw_cmd_vel
rostopic info /cmd_vel
```

预期：

* `/competition/control/status` 只有 `cmd_vel_guard_node` 发布。
* `/competition/control/path_follower_status` 只有
  `path_follower_node` 发布。
* `/competition/raw_cmd_vel` 由 `path_follower_node` 发布。
* `/cmd_vel` 只有 `cmd_vel_guard_node` 发布。

完整 Control 联调应使用真实：

* `path_follower_node`
* `cmd_vel_guard_node`

并由 mock Planning、mock Localization 和 mock Competition Loop
提供输入。

已验证场景包括：

* 正常跟踪时最终 `/cmd_vel.linear.x = 0.25`；
* Planning invalid 立即停车；
* Planning 恢复后旧 Path 不复活；
* Localization invalid 立即停车；
* Localization 恢复并收到新 Odom 后，旧 Path 不复活；
* 收到新完整数据后恢复运行；
* Path timeout；
* Odom timeout；
* Planning status timeout；
* Localization status timeout；
* 非法 Path frame；
* 非法 Odom frame；
* 非法 Odom `child_frame_id`；
* `control_enabled=false` 时最终速度为零；
* `emergency_stop=true` 时最终速度为零；
* 急停优先级高于控制禁用。

禁止正式算法订阅 `/Odometry_gazebo`、`/ground_truth/*`
或读取其他真值接口绕过 Localization。
