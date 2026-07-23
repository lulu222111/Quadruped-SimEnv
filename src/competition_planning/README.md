# competition_planning

**负责人**: 闫雨萱（前沿点提取与路径规划）

## 模块目标

从 OccupancyGrid 提取 Frontier 前沿点，选择最佳探索目标，规划无碰路径。

## 输入

| 输入 | 类型 | 说明 |
|------|------|------|
| `/competition/map` | `nav_msgs/OccupancyGrid` | 占据栅格地图 |
| `/competition/localization/odom` | `nav_msgs/Odometry` | 机器人当前位置 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/navigation_goal` | `geometry_msgs/PoseStamped` | control | 当前导航目标 |
| `/competition/path` | `nav_msgs/Path` | control | 规划路径 |
| `/competition/planning/status` | `ModuleStatus` | competition_loop | 规划模块状态 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~frontier_min_size` | int | 5 | 最小前沿点数（cells） |
| `~replan_interval` | float | 2.0 | 重规划间隔（秒） |
| `~goal_tolerance` | float | 0.5 | 目标到达容差（m） |

## 当前状态

**v0.1.0 — 骨架**

- [x] 占位节点只发布 `ModuleStatus(state=STARTING, valid=false)`
- [x] 不发布伪造目标或路径
- [ ] Frontier 前沿点检测
- [ ] 信息增益评估与目标选择
- [ ] A* / Dijkstra 全局路径规划
- [ ] 无前沿时返回起点（RETURNING）
- [ ] 不可达目标重规划

**默认安全行为**：
- 不发布导航目标（无 PoseStamped）
- 不发布路径（无 Path）

## 待实现 TODO

1. **Frontier 提取**：从 OccupancyGrid 边界处检测未知-空闲交界
2. **目标选择**：最近前沿、最大信息增益、或基于房间的启发式
3. **全局路径**：A* 或 Dijkstra 在 OccupancyGrid 上搜索
4. **无前沿处理**：所有前沿已探索 → 发布 RETURNING 信号
5. **不可达目标**：检测规划失败 → 标记目标为不可达 → 选择下一个
6. **多楼层**：到达电梯口 → 通过 competition_loop 进行乘梯

## 禁止事项

- 不得读取 `layout_metadata.json` 获取房间布局
- 不得读取 `danger_truth.json` 获取危险源位置
- 不得订阅 `/ground_truth/*`

## 最小运行命令

```bash
source devel/setup.bash
rosrun competition_planning planning_node
```

## 验收标准

- [x] 启动后只发布 `valid=false` 的状态
- [x] 不订阅禁止话题
- [x] 不读取禁止文件
- [ ] 正确提取 OccupancyGrid 中的 Frontier
- [ ] 能生成可达目标的无碰路径
- [ ] 无前沿时发出明确信号
- [ ] 路径规划时间 < 500 ms
- [ ] 在 20×36 m 场景中不遗漏可达房间

## 需要向上下游成员交付的内容

- 杨渊浩（control）：`/competition/navigation_goal` + `/competition/path`
- 马跃（mapping）：依赖 `/competition/map`
- 纪雯瀚（localization）：依赖 `/competition/localization/odom`
