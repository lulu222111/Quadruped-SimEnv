# competition_localization

**负责人**: 纪雯瀚（定位与状态估计）

## 模块目标

只使用允许的 Livox 点云、IMU 和公开场景信息估计机器人位姿（`world` 系）。
严禁使用 Gazebo 真值话题或服务。

## 输入

| 输入 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/trunk_imu` | `sensor_msgs/Imu` | 1000 Hz | 躯干 IMU |
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | 雷达内置 IMU |
| `/scan` | `sensor_msgs/PointCloud` | 10 Hz | Livox 原始点云 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | ~10 Hz | 转换后点云 |
| `team_scene_info.json` | 文件 | 一次性 | 起点信息用于初始对齐 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/localization/odom` | `nav_msgs/Odometry` | mapping, planning | 估计位姿 |
| `/competition/localization/status` | `ModuleStatus` | competition_loop | 定位模块状态 |
| TF `odom -> base` | transform | 全系统 | 坐标变换 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~publish_tf` | bool | true | 是否发布 TF |

## 当前状态

**v0.1.0 — 骨架**

- [x] 占位节点只发布 `ModuleStatus(state=STARTING, valid=false)`
- [x] 不发布伪造 Odometry
- [ ] LiDAR/IMU 里程计算法实现
- [ ] 基于 team_scene_info.json 起点的初始对齐
- [ ] TF 发布
- [ ] 回环检测与优化

**默认安全行为**：
- 不发布任何位姿估计（`valid=false`）
- 不发布 TF

## 待实现 TODO

1. IMU 预积分
2. Livox 点云帧间匹配（ICP / NDT）
3. 紧耦合或松耦合 LiDAR-IMU 融合（LIO-SAM / FAST-LIO 风格）
4. 起点与世界坐标系对齐
5. 位姿不确定性估计
6. 摔倒时的位姿重置

## 禁止事项

- 不得订阅 `/Odometry_gazebo`
- 不得订阅 `/ground_truth/*`
- 不得调用 Gazebo get_model_state 获取机器人真值位姿
- 不得读取 `results/danger_truth.json` 或 `generated_building/danger_truth.json`
- 不得读取 `layout_metadata.json` 或任何其他 forbidden 文件

## 最小运行命令

```bash
source devel/setup.bash
rosrun competition_localization localization_node
```

## 验收标准

- [x] 启动后只发布 `valid=false` 的状态
- [x] 不订阅禁止话题（代码审查确认）
- [x] 不读取禁止文件
- [ ] 合法定位 `/competition/localization/odom` 发布（精度待定）
- [ ] TF `odom -> base` 发布
- [ ] 在连续行走 100 m 后漂移 < 1 m
- [ ] ModuleStatus.valid 根据定位质量动态更新

## 需要向上下游成员交付的内容

- 马跃（mapping）：`/competition/localization/odom` + TF
- 闫雨萱（planning）：`/competition/localization/odom` + TF
- 黄誉铭（loop）：定位状态 `valid` 标志用于判断模块就绪
