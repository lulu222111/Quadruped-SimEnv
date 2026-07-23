# competition_mapping

**负责人**: 马跃（地图构建与维护）

## 模块目标

将 Livox 点云和合法位姿估计融合为 `nav_msgs/OccupancyGrid`，供规划模块使用。

## 输入

| 输入 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/competition/localization/odom` | `nav_msgs/Odometry` | — | 合法定位 |
| `/scan` | `sensor_msgs/PointCloud` | 10 Hz | Livox 原始点云 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | ~10 Hz | 转换后点云 |
| TF `odom -> base` | transform | — | 坐标变换 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/map` | `nav_msgs/OccupancyGrid` | planning | 占据栅格地图 |
| `/competition/mapping/status` | `ModuleStatus` | competition_loop | 地图模块状态 |

## 地图值语义

| 值 | 含义 |
|----|------|
| -1 | Unknown（未探索） |
| 0 | Free（空闲可通行） |
| 100 | Occupied（被占据） |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~resolution` | float | 0.1 | 栅格分辨率（m/cell） |
| `~map_width` | int | 400 | 地图宽度（cells） |
| `~map_height` | int | 400 | 地图高度（cells） |
| `~update_rate` | float | 2.0 | 地图发布频率（Hz） |
| `~sensor_max_range` | float | 20.0 | 传感器最大有效距离（m） |

## 当前状态

**v0.1.0 — 骨架**

- [x] 占位节点只发布 `ModuleStatus(state=STARTING, valid=false)`
- [x] 不发布伪造 OccupancyGrid
- [ ] 点云滤波和下采样
- [ ] 点云到 OccupancyGrid 的投影
- [ ] 概率占据更新（log-odds）
- [ ] 基于合法定位的全局地图维护
- [ ] 地板去除（ground removal）

**默认安全行为**：
- 不发布地图（`valid=false`）

## 待实现 TODO

1. 点云预处理：体素滤波、离群点去除
2. 地面分割：分离地板点云
3. 概率占据：基于 ray-casting 的 log-odds 更新
4. 多层地图：支持多楼层（可能需 3D 地图或按楼层切换）
5. 动态更新：关门/开门后更新地图

## 禁止事项

- 不得使用 `/ground_truth/*` 作为位姿输入
- 不得读取 `layout_metadata.json` 获取建筑布局
- 不得读取 `scene_manifest.json` 获取地图先验
- 不得调用 Gazebo service 获取建筑几何

## 最小运行命令

```bash
source devel/setup.bash
rosrun competition_mapping mapping_node
```

## 验收标准

- [x] 启动后只发布 `valid=false` 的状态
- [x] 不订阅禁止话题
- [x] 不读取禁止文件
- [ ] 正确订阅 `/competition/localization/odom` 和点云
- [ ] OccupancyGrid 正确填充未知/空闲/占据
- [ ] 地图更新率 ≥ 2 Hz
- [ ] 静态环境下重复建图一致性良好
- [ ] 地图可在 RViz 中正常显示

## 需要向上下游成员交付的内容

- 闫雨萱（planning）：`/competition/map` + 地图元数据
- 纪雯瀚（localization）：依赖合法位姿和 TF
