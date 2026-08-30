# competition_planning

**负责人**：闫雨萱
**开发分支**：`feat/yanyx/planning-contract-v2`

## 模块目标

Planning 根据 Mapping、Localization 的正式数据和模块状态完成 Frontier 探索与路径规划：

1. 检查两个上游模块状态及输入新鲜度；
2. 提取、聚类并筛选 Frontier；
3. 对障碍物进行安全膨胀；
4. 使用单源四邻域 BFS 构建可达性搜索树；
5. 为可达 Frontier 生成导航目标与完整 Path；
6. 上游或规划失效时立即撤销旧 Path。

## 正式输入接口

| 话题 | 类型 | 约束 |
|---|---|---|
| `/competition/map` | `nav_msgs/OccupancyGrid` | `header.frame_id=world` |
| `/competition/mapping/status` | `competition_interfaces/ModuleStatus` | `valid=true` 才允许使用 Map |
| `/competition/localization/odom` | `nav_msgs/Odometry` | `header.frame_id=world`、`child_frame_id=base` |
| `/competition/localization/status` | `competition_interfaces/ModuleStatus` | `valid=true` 才允许使用 Odometry |

Planning 只有在两个 status 均有效且未超时、当前 generation 已收到新 Map 和新 Odometry、数据合法且未超时、frame 和时间戳符合契约时才允许规划。

Localization 正式接口实测约 10 Hz。Planning 不读取 FAST-LIO 内部参数，不自行做 TF 转换，也不硬编码机器人起点。

## 正式输出接口

| 话题 | 类型 | 语义 |
|---|---|---|
| `/competition/navigation_goal` | `geometry_msgs/PoseStamped` | `frame_id=world`，不锁存 |
| `/competition/path` | `nav_msgs/Path` | Path 和所有 Pose 使用 `world`，允许锁存 |
| `/competition/planning/status` | `competition_interfaces/ModuleStatus` | 当前规划结果是否允许 Control 执行 |

### 输出安全约定

- 新 Path 完整替换旧 Path；
- 空 Path 表示立即撤销此前锁存的 Path；
- `planning/status.valid=false` 时 Control 必须停止跟踪；
- 失效时不发布新 navigation goal；
- 计算期间 generation 或输入发生变化时丢弃旧规划结果；
- 有效 Path 中的位姿方向根据路径切线计算。

## generation 规则

Mapping/Localization invalid、status timeout、输入 timeout 或 ROS 时间回退会使旧缓存失效并进入新的本地 generation。

恢复规划必须重新获得：

- Mapping status valid；
- Localization status valid；
- 当前 generation 的新 Map；
- 当前 generation 的新 Odometry。

四项全部满足后才允许恢复，失效前的数据不能自动复活。

每次 `generation += 1` 时记录 generation 起始时间边界
`generation_start_stamp`；只有 generation 真正增加时才更新边界，同一
active fault 的重复心跳不会移动边界。

Map / Odometry 写入当前 generation 缓存前必须满足
`header.stamp > generation_start_stamp`（严格大于，等于边界同样拒绝）。
`stamp <= generation_start_stamp` 的消息视为上一 generation 延迟到达的
旧数据，只丢弃：不更新缓存、不触发新的 generation。该判断与上游
status 当前是否 valid 无关——新 generation 中 Mapping 仍 invalid 时，
fresh 数据可以先行缓存为候选，待上游恢复 valid 后才能消费。

ModuleStatus 按流单调准入：每个 status 流的 `header.stamp` 必须严格
大于该流上一次已接受的时间戳，否则视为延迟/重复旧消息直接丢弃——
不更新存储的 status、不清除 fault、不开启 generation。因此延迟到达的
旧 `valid=true` status 不会覆盖更新的 invalid status，也不会触发恢复
或导致后续真实 invalid 心跳强制额外 generation。ROS 时间回退（新
时间线）会重置两个 status 流的基线。

## 时间保护

Planning 分别检查：

- `received_at`：本地接收 watchdog；
- `header.stamp`：零时间戳、明显过旧和明显未来时间戳；
- `generation_start_stamp`：上一 generation 延迟到达的数据不得进入
  当前 generation 缓存；
- `last_ros_time`：ROS `/clock` 回退。

时间异常时发布空 Path 和 `planning/status.valid=false`。ROS 时间回退后旧 Map、Odometry 和 Path 均不得继续使用。重复出现的独立 stale
消息在完全恢复后可以再次开启新 generation；message 级时间戳校验在
当前 generation 全部恢复正常后清除通用 fault。

## ROS 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `~frontier_min_size` | `5` | Frontier 聚类最少格子数 |
| `~replan_interval` | `2.0` | 重规划周期（秒） |
| `~inflation_radius` | `0.25` | 障碍物膨胀半径（米） |
| `~size_weight` | `1.0` | Frontier 大小权重 |
| `~distance_weight` | `1.0` | 距离权重 |
| `~obstacle_threshold` | `50` | 占用格阈值 |
| `~map_timeout` | `5.0` | Map 接收及消息时间戳超时（秒） |
| `~localization_timeout` | `5.0` | Odometry 接收及消息时间戳超时（秒） |
| `~mapping_status_timeout` | `3.0` | Mapping status 超时（秒） |
| `~localization_status_timeout` | `3.0` | Localization status 超时（秒） |
| `~future_stamp_tolerance` | `0.5` | 未来时间戳容差（秒） |
| `~status_heartbeat_interval` | `1.0` | Planning status heartbeat 周期（秒） |
| `~exploration_complete_confirmations` | `3` | 连续同一 generation 的无 Frontier 结果确认探索完成的次数 |
| `~visual_scan_enabled` | `true` | 是否在新探索目标处执行 bounded active visual scan |
| `~visual_scan_yaw_step` | `0.7853981634` | 扫描 heading 间隔（弧度，默认 45°） |
| `~visual_scan_yaw_tolerance` | `0.15` | heading 到达容差（弧度） |
| `~visual_scan_dwell` | `0.6` | 每个 heading 到达后的观察停留时间（秒） |
| `~visual_scan_heading_timeout` | `4.0` | 单个 heading 的到达超时（秒） |
| `~visual_scan_timeout` | `45.0` | 整个 scan episode 的最大时间（秒） |
| `~visual_scan_position_dedup_radius` | `1.0` | 已扫描位置的 2D 去重半径（米） |
| `~visual_scan_route_spacing` | `2.0` | 沿 frontier route 选择视觉扫描 checkpoint 的最小间距（米） |
| `~visual_scan_target_tolerance` | `0.35` | 判断探索目标已到达的距离容差（米） |
| `~visual_scan_max_heading_retries` | `1` | 单个 heading 的有限重试次数 |
| `~visual_scan_update_interval` | `0.1` | 非阻塞 scan 状态机更新周期（秒） |
| `~visual_scan_final_before_complete` | `true` | 无 Frontier 时是否先执行当前位置 final scan |

`localization_timeout` 按 Localization 跟进说明暂时保留为可配置默认值 `5.0`，待真实 Mapping → Planning → Control 联调后再统一调整。

## Active visual coverage

Frontier 是地图覆盖信号，不代表 RGB-D 相机已经覆盖了房间内部。探索模式下，
Planner 会先把首个 route 的首个自由位姿作为初始搜索 station；之后在新的 route 上
选择间隔至少为 `visual_scan_route_spacing` 的 checkpoint，机器人实际到达 checkpoint
后进入内部 `VISUAL_SCAN` 子状态；若后续 route 短于该间隔则使用 route 终点。
checkpoint 一旦为当前 generation 选定，就保持到机器人实际到达并处理该位置，或
generation/reset 清除它；地图重规划不会用抖动的 route 顶点替换尚未完成的视觉义务。
同时只发布到该 checkpoint 为止的 planner 路径前缀，并在重规划时刷新时间戳而保持几何
不变，避免 follower 反复重置路径进度。若机器人已经从 checkpoint 分配位置实际前进
至少一个 route-spacing 且仍未到达，则当前正式 odometry 位置会作为下一视觉站重新锚定，
而旧位置不会被标记为已扫描，避免控制器/定位偏差造成永久等待。
以当前位姿为中心发布一组同位置、不同 yaw 的短路径点。默认以 45° 间隔
覆盖 360°，每个 heading 到达后停留 0.6 秒；状态机由 timer 驱动，不阻塞规划
callback。每个 heading 和整个 episode 都有超时，失败后有限重试并继续 Frontier
探索，避免扫描形成新的死锁。

扫描位置以 2D 半径去重，附近目标不会反复触发完整扫描。若 Frontier 突然为空，
Planner 会先尝试一次当前位置的 final scan；只有该 scan 完成或有界结束，并且
连续达到 `exploration_complete_confirmations` 次同 generation 的 `NO_FRONTIER`
结果后，才锁存 `/competition/planning/exploration_complete=true`。新 generation
或任务状态切换会清空扫描位置、当前 episode 和 completion debounce。

扫描期间 `/competition/planning/status` 仍为 `RUNNING, valid=true`。Planner 不
读取危险源位置、Gazebo 真值或隐藏场景信息。Control 通过同位置连续 Path pose
识别 rotation-only waypoint 并执行原地 yaw；普通位置路径仍使用原有跟踪规则。

## 当前已实现

- [x] 四个正式输入话题接入
- [x] 上游 `ModuleStatus.valid` 门控
- [x] status、Map 和 Odometry 接收超时保护
- [x] 消息时间戳零值、过旧及未来值保护
- [x] generation 起始时间边界与延迟旧数据丢弃
- [x] 重复独立 stale 消息的 fault 生命周期
- [x] negative 规划结果（NO_FRONTIER / UNREACHABLE / 异常）跨 generation 提交保护
- [x] ROS 时间回退处理
- [x] 新 generation 数据隔离与恢复门控
- [x] 固定 `world/base` 契约检查
- [x] Planning status 1 Hz heartbeat
- [x] 空 Path 撤销旧锁存路径
- [x] Frontier 提取、聚类、过滤和多候选回退
- [x] 地图格子与世界坐标双向转换
- [x] 单源四邻域 BFS 及父节点路径重建
- [x] 单次 BFS 搜索树复用于所有 Frontier 候选
- [x] 障碍物膨胀与机器人起点安全恢复
- [x] 路径方向四元数计算
- [x] 区分探索完成与暂时不可达
- [x] 新探索目标处的 bounded active visual scan
- [x] 扫描位置去重、heading dwell、有限超时和 generation reset
- [x] final visual scan gate 与去抖后的 exploration_complete

## 状态语义

| 场景 | state | valid | Path |
|---|---:|---:|---|
| 等待 status 或当前 generation 数据 | `STARTING` | `false` | 空 |
| 生成当前可执行路径或执行 visual scan | `RUNNING` | `true` | 非空 |
| 没有 Frontier、探索完成 | `SUCCEEDED` | `false` | 空 |
| 上游 invalid 或 Frontier 不可达 | `BLOCKED` | `false` | 空 |
| status、Map 或 Odometry 超时 | `TIMEOUT` | `false` | 空 |
| frame、数据、参数或时间戳错误 | `FAILED` | `false` | 空 |

## 已完成验证

### 无 ROS 单元测试

```text
Ran 134 tests
OK
```

```bash
python3 -m unittest discover \
  -s src/competition_planning/test \
  -p 'test_*.py' \
  -v
```

覆盖 Frontier、坐标转换、单源 BFS、多候选回退、障碍膨胀、状态区分、路径方向、
active visual coverage 的 heading/dwell/timeout/去重/route checkpoint/completion gate，以及 20×36 m
和 400×400 地图性能。

### ROS 契约集成测试

```text
RESULT: SUCCESS
TESTS: 1
ERRORS: 0
FAILURES: 0
```

已覆盖四输入订阅、上游 status 门控、`world/base`、invalid 撤销、generation 恢复、frame 拒绝和 status timeout。

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
catkin_make -j
source devel/setup.bash
rostest competition_planning planning_node_ros.test --text
```

## 尚待真实联调

- [ ] 与真实 Mapping 的 Map/status 频率和延迟联调
- [ ] 与约 10 Hz Localization 的 Odometry/status 联调
- [ ] 核对 `world -> base` TF 与 Odometry 一致性
- [ ] 根据真实链路调整 timeout 参数
- [ ] 与 Control 验证 Path 替换、空 Path 和 `valid=false` 停车
- [ ] 验证定位跳变、短时丢失和动态重规划
- [ ] 验证真实 ROS `/clock` 回退场景
- [ ] 验证 20×36 m 场景覆盖率和真实 RGB-D 视觉覆盖率
- [ ] 实现多楼层、电梯和返回起点策略

## 禁止事项

- 不读取 `layout_metadata.json`；
- 不读取 `danger_truth.json`；
- 不订阅 `/ground_truth/*`；
- 不使用裁判真值作为规划输入；
- 不保存 Localization 内部 FAST-LIO 或传感器外参参数；
- 不在无效状态下保留旧 Path。

测试 Mock 只能放在 `competition_testing`、Planning test 或测试 launch 中，不得进入正式比赛 launch。

## 团队协作接口

- Mapping：提供 `/competition/map` 和 `/competition/mapping/status`
- Localization：提供 `/competition/localization/odom` 和 `/competition/localization/status`
- Control：使用 `/competition/navigation_goal`、`/competition/path` 和 `/competition/planning/status`
- competition_loop：使用 `/competition/planning/status`
