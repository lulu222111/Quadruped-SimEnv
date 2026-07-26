# competition_planning

**负责人**：闫雨萱
**开发分支**：`feat/yanyx/planning`

## 模块目标

根据占据栅格地图和机器人定位结果：

1. 提取未知区域与空闲区域交界处的 Frontier；
2. 将相邻 Frontier 格子聚类并过滤小区域；
3. 综合区域大小和距离选择探索目标；
4. 对障碍物进行安全膨胀；
5. 使用 A* 规划无碰路径；
6. 向控制模块发布导航目标和路径。

## 输入

| 输入话题 | 类型 | 提供模块 | 说明 |
|---|---|---|---|
| `/competition/map` | `nav_msgs/OccupancyGrid` | mapping | 占据栅格地图 |
| `/competition/localization/odom` | `nav_msgs/Odometry` | localization | 机器人当前位置 |

## 输出

| 输出话题 | 类型 | 使用模块 | 说明 |
|---|---|---|---|
| `/competition/navigation_goal` | `geometry_msgs/PoseStamped` | control | 当前 Frontier 导航目标 |
| `/competition/path` | `nav_msgs/Path` | control | A* 规划路径 |
| `/competition/planning/status` | `competition_interfaces/ModuleStatus` | competition_loop | 规划模块状态 |

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---:|---|
| `~frontier_min_size` | int | `5` | Frontier 聚类最少格子数 |
| `~replan_interval` | float | `2.0` | 重规划周期，单位为秒 |
| `~inflation_radius` | float | `0.25` | 障碍物安全膨胀半径，单位为米 |
| `~size_weight` | float | `1.0` | Frontier 区域大小评分权重 |
| `~distance_weight` | float | `1.0` | 机器人到目标距离评分权重 |
| `~obstacle_threshold` | int | `50` | 判定占用格子的阈值 |

## 当前已实现

- [x] 从 `OccupancyGrid` 提取 Frontier 格子
- [x] 采用八邻域进行 Frontier 聚类
- [x] 使用 `frontier_min_size` 过滤零散噪声
- [x] 选择距离聚类中心最近的真实 Frontier 格子
- [x] 综合区域大小和距离选择候选目标
- [x] 地图格子坐标与世界坐标双向转换
- [x] 支持带旋转的地图原点
- [x] 四邻域 A* 路径规划
- [x] 障碍物安全膨胀
- [x] 禁止路径穿过未知区域和障碍物
- [x] 当前候选目标不可达时尝试下一片 Frontier
- [x] 接入地图和定位 ROS 话题
- [x] 发布导航目标、路径和模块状态
- [x] 46 项无 ROS 依赖的自动测试通过

## 当前状态说明

模块已经完成个人分支中的核心算法、ROS 话题接入以及标准个人容器内的独立验证。

已完成的验证：

- Python 语法检查通过；
- 46 项 `unittest` 自动测试在标准个人 Docker 容器中通过；
- ROS Noetic 工作区 `catkin_make` 编译通过；
- `rospack find competition_planning` 能够正确识别规划包；
- `planning_node` 能够在真实 ROS Master 中启动；
- 缺少地图和定位输入时，模块保持 `STARTING`、`valid=false`，不发布伪造规划结果；
- `GUI=false` 无界面 Gazebo、Unitree A1、传感器和控制器启动通过；
- 使用合规 mock `OccupancyGrid` 和 `Odometry` 完成 ROS 话题闭环验证；
- mock 验证中检测到 12 个 Frontier 格子，发布目标 `(1.5, 2.5)` 和包含 2 个位姿的路径；
- `/competition/navigation_goal` 类型为 `geometry_msgs/PoseStamped`；
- `/competition/path` 类型为 `nav_msgs/Path`；
- 导航目标和路径的坐标系均为 `map`；
- Git 差异格式检查通过。

尚待团队联调和性能验证：

- 使用 mapping 模块提供的真实 `/competition/map` 联调；
- 使用 localization 模块提供的真实 `/competition/localization/odom` 联调；
- 与 control 模块验证目标、路径消费和重规划行为；
- 在真实 Gazebo 地图中验证路径安全性、不可达目标处理和动态重规划；
- 完成 20×36 m 场景性能和覆盖率测试；
- 按需使用 RViz 和 Gazebo GUI 完成可视化验证。

## 自动测试

在仓库根目录执行：

```bash
python3 -m unittest discover \
  -s src/competition_planning/test \
  -p 'test_*.py' \
  -v

```

当前预期结果：

```text
Ran 46 tests
OK
```

## ROS 编译与运行

需要在团队统一的 ROS Noetic Docker 环境中执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source devel/setup.bash
rosrun competition_planning planning_node
```

启动节点本身不会启动 Gazebo，但正式联调应由负责人统一安排。

## 状态行为

- 地图或定位尚未收到：
  - `state=STARTING`
  - `valid=false`
- 成功生成目标和路径：
  - `state=RUNNING`
  - `valid=true`
- 没有可达 Frontier：
  - `state=BLOCKED`
  - `valid=false`
- 输入数据或参数无效：
  - `state=FAILED`
  - `valid=false`

## 安全与禁止事项

本模块：

- 不读取 `layout_metadata.json`；
- 不读取 `danger_truth.json`；
- 不订阅 `/ground_truth/*`；
- 不直接发布 `/cmd_vel`；
- 不把未知区域当作可通行区域；
- 不伪造地图、定位、目标或路径数据。

## 待完成事项

- [ ] Docker 中完成 ROS 编译验证
- [ ] 与真实 mapping 输出联调
- [ ] 与 localization 输出联调
- [ ] 与 control 路径消费者联调
- [ ] 区分“探索完成”和“暂时不可达”
- [ ] 完善目标到达与重复目标抑制
- [ ] 多楼层、电梯和返回起点策略
- [ ] 验证规划时间小于 500 ms
- [ ] 验证 20×36 m 场景中的可达房间覆盖率

## 团队协作接口

- 马跃（mapping）：提供 `/competition/map`
- 纪雯瀚（localization）：提供 `/competition/localization/odom`
- 杨渊浩（control）：使用 `/competition/navigation_goal` 和 `/competition/path`
- competition_loop：使用 `/competition/planning/status`