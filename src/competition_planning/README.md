# competition_planning

**负责人**：闫雨萱
**开发分支**：`feat/yanyx/planning`

## 模块目标

根据占据栅格地图和机器人定位结果：

1. 提取未知区域与空闲区域交界处的 Frontier；
2. 将相邻 Frontier 格子聚类并过滤小区域；
3. 综合区域大小和距离选择探索目标；
4. 对障碍物进行安全膨胀；
5. 使用单源四邻域 BFS 一次构建可达性搜索树，复用至所有 Frontier 候选点，通过父节点表重建最短路径；
6. 向控制模块发布导航目标和路径；
7. 在输入失效、坐标系不一致或规划失败时撤销旧路径。

## 输入

| 输入话题 | 类型 | 提供模块 | 说明 |
|---|---|---|---|
| `/competition/map` | `nav_msgs/OccupancyGrid` | mapping | 占据栅格地图 |
| `/competition/localization/odom` | `nav_msgs/Odometry` | localization | 机器人当前位置 |

### 坐标系约束

`/competition/map` 与 `/competition/localization/odom` 的
`header.frame_id` 必须均为非空值且完全相同。

当前版本不执行 TF 坐标转换。若两个坐标系不一致，规划模块将：

- 发布 `ModuleStatus(state=FAILED, valid=false)`；
- 发布空的 `/competition/path`，撤销此前锁存的路径；
- 不发布新的导航目标；
- 不继续进行路径规划。

## 输出

| 输出话题 | 类型 | 使用模块 | 说明 |
|---|---|---|---|
| `/competition/navigation_goal` | `geometry_msgs/PoseStamped` | control | 当前 Frontier 导航目标 |
| `/competition/path` | `nav_msgs/Path` | control | A* 规划路径 |
| `/competition/planning/status` | `competition_interfaces/ModuleStatus` | competition_loop | 规划模块状态 |

### 输出安全约定

- `/competition/navigation_goal` 不锁存，避免新启动的控制节点收到旧目标；
- `/competition/path` 使用锁存，但规划失效时会发布空路径撤销旧路径；
- 控制模块收到空路径或 `planning/status.valid=false` 时应停止路径跟踪；
- 有效路径中的每个位姿均根据路径切线设置行进方向四元数；
- 最后一个路径位姿沿用最后一段路径的方向。

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---:|---|
| `~frontier_min_size` | int | `5` | Frontier 聚类最少格子数 |
| `~replan_interval` | float | `2.0` | 重规划周期，单位为秒 |
| `~map_timeout` | float | `5.0` | 地图消息最长允许未更新时间，单位为秒 |
| `~localization_timeout` | float | `5.0` | 定位消息最长允许未更新时间，单位为秒 |
| `~inflation_radius` | float | `0.25` | 障碍物安全膨胀半径，单位为米 |
| `~size_weight` | float | `1.0` | Frontier 区域大小评分权重 |
| `~distance_weight` | float | `1.0` | 机器人到目标距离评分权重 |
| `~obstacle_threshold` | int | `50` | 判定占用格子的阈值，合法范围为 `[1, 100]` |

### 参数安全约束

节点启动时会检查：

- `frontier_min_size >= 1`；
- `replan_interval > 0`；
- `map_timeout > 0`；
- `localization_timeout > 0`；
- `inflation_radius >= 0`；
- `size_weight >= 0`；
- `distance_weight >= 0`；
- `1 <= obstacle_threshold <= 100`；
- 所有浮点参数必须是有限值，拒绝 `NaN` 和 `Inf`。

## 当前已实现

- [x] 从 `OccupancyGrid` 提取 Frontier 格子
- [x] 采用八邻域进行 Frontier 聚类
- [x] 使用 `frontier_min_size` 过滤零散噪声
- [x] 选择距离聚类中心最近的真实 Frontier 格子
- [x] 综合区域大小和距离选择 Frontier 簇
- [x] 同簇代表点不可达时继续尝试同簇其他候选点
- [x] 整个 Frontier 簇不可达后尝试下一片 Frontier
- [x] 地图格子坐标与世界坐标双向转换
- [x] 支持带旋转的地图原点
- [x] 四邻域 A* 路径规划
- [x] 单源 BFS 可达性搜索树在所有 Frontier 候选间复用
- [x] 障碍物安全膨胀
- [x] 膨胀覆盖机器人起点时，仅恢复原本空闲的起点格
- [x] 禁止路径穿过未知区域和障碍物
- [x] 接入地图和定位 ROS 话题
- [x] 强制检查地图与定位坐标系一致
- [x] 地图或定位输入超时后撤销旧路径并发布 `TIMEOUT`
- [x] 发布导航目标、路径和模块状态
- [x] 规划失效时发布空路径撤销旧锁存路径
- [x] 导航目标不锁存
- [x] 根据路径切线生成路径位姿方向四元数
- [x] 区分“探索完成”和“暂时不可达”
- [x] 启动时完整校验参数并拒绝 `NaN`、`Inf` 和非法范围
- [x] 62 项无 ROS 依赖的自动测试通过
- [x] ROS 自动集成测试通过

## 规划状态

| 场景 | `ModuleStatus.state` | `valid` | 路径输出 |
|---|---:|---:|---|
| 等待地图或定位 | `STARTING` | `false` | 空路径 |
| 正常生成路径 | `RUNNING` | `true` | 有效路径 |
| 探索完成、没有 Frontier | `SUCCEEDED` | `false` | 空路径 |
| 存在 Frontier 但均不可达 | `BLOCKED` | `false` | 空路径 |
| 地图或定位输入过期 | `TIMEOUT` | `false` | 空路径 |
| 参数、数据或坐标系错误 | `FAILED` | `false` | 空路径 |

## 已完成验证

### 无 ROS 单元测试

已验证：

- Frontier 提取、聚类和过滤；
- Frontier 代表点及簇评分；
- 同簇多候选点回退；
- 地图格子坐标与世界坐标转换；
- 带旋转地图原点；
- 四邻域 A*；
- 障碍物膨胀；
- 机器人起点被膨胀覆盖时的安全恢复；
- 探索完成与不可达状态区分；
- 20×36 m 和 400×400 地图规划性能；
- 路径方向计算；
- 单源 BFS 可达性搜索在所有 Frontier 候选间复用；
- 同簇代表点不可达时回退至同簇其他候选（含真实网格验证）；
- 重建路径完全由合法空闲格组成；
- 起点等于目标、起点越界、起点被障碍占用的边界情况。<｜end▁of▁thinking｜>

<｜｜DSML｜｜parameter name="new_string" string="true">- 四邻域 A*；
- 障碍物膨胀；
- 机器人起点被膨胀覆盖时的安全恢复；
- 探索完成与不可达状态区分；
- 20×36 m 和 400×400 地图规划性能；
- 路径方向计算；
- 单源 BFS 可达性搜索在全部 Frontier 候选间复用；
- 同簇代表点不可达时回退至同簇其他候选（含真实网格验证）；
- 重建路径全部由合法空闲格组成；
- 起点等于目标、起点越界、起点被障碍占用的边界情况。

当前预期结果：

```text
Ran 62 tests
OK
```

### ROS 自动集成测试

已验证：

- 节点能根据 mock 地图和定位生成有效路径；
- 有效路径中的四元数与路径前进方向一致；
- 坐标系不一致时发布 `FAILED` 和空路径；
- Frontier 不可达时发布 `BLOCKED` 和空路径；
- 输入超时后发布 `TIMEOUT` 和空路径；
- 成功规划后再次失效时，旧锁存路径被清空；
- 新订阅者不会收到过期的锁存导航目标。

当前预期结果：

```text
RESULT: SUCCESS
TESTS: 1
ERRORS: 0
FAILURES: 0
```

### Docker 与 ROS 环境

已在个人 Docker 环境中完成：

- ROS Noetic 环境检查；
- `catkin_make` 编译；
- `rospack find competition_planning` 检查；
- 62 项单元测试；
- ROS 自动集成测试；
- headless Gazebo 启动检查；
- mock 地图与定位话题验证；
- 导航目标、路径和模块状态输出检查。

## 自动测试

在仓库根目录执行：

```bash
python3 -m unittest discover \
  -s src/competition_planning/test \
  -p 'test_*.py' \
  -v
```

## ROS 编译与运行

在团队统一的 ROS Noetic Docker 环境中执行：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
catkin_make -j
source devel/setup.bash
rosrun competition_planning planning_node
```

## ROS 自动集成测试

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
catkin_make -j
source devel/setup.bash
rostest competition_planning planning_node_ros.test --text
```

## 尚待团队联调

- [ ] 使用 mapping 模块真实发布的 `/competition/map` 联调
- [ ] 使用 localization 模块真实发布的 `/competition/localization/odom` 联调
- [ ] 与 control 模块确认空路径和 `valid=false` 时立即停止跟踪
- [ ] 验证 control 模块是否使用路径姿态
- [ ] 在真实 Gazebo 地图中验证动态重规划
- [ ] 验证20×36 m 场景中的可达房间覆盖率
- [ ] 完善目标到达和重复目标抑制
- [ ] 实现多楼层、电梯和返回起点策略

## 禁止事项

规划模块不得：

- 读取 `layout_metadata.json` 获取房间或目标位置；
- 读取 `danger_truth.json` 获取危险源位置；
- 订阅 `/ground_truth/*`；
- 使用裁判真值作为规划输入；
- 在无效状态下继续保留旧路径。

## 团队协作接口

- 马跃（mapping）：提供 `/competition/map`
- 纪雯瀚（localization）：提供 `/competition/localization/odom`
- 杨渊浩（control）：使用 `/competition/navigation_goal` 和 `/competition/path`
- competition_loop：使用 `/competition/planning/status`