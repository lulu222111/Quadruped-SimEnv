# competition_testing

**负责人**: 黄誉铭（团队负责人）

## 模块目标

纯合成 Mock 数据和接口联调。所有 Mock 数据来自硬编码参数或固定人工数据，
**严禁读取任何真值文件或订阅真值话题**。

## Mock 节点

| 节点 | 发布的话题 | 数据来源 |
|------|-----------|---------|
| `mock_localization_node` | `/competition/localization/odom` | 硬编码圆形轨迹 |
| `mock_mapping_node` | `/competition/map` | 硬编码 OccupancyGrid |
| `mock_perception_node` | `/competition/hazard_candidates` | 硬编码候选列表 |
| `mock_planning_node` | `/competition/navigation_goal`, `/competition/path` | 硬编码目标点 |

## 安全约束

- 所有 Mock 节点必须在 `test_mode=true` 时才运行
- `test_mode=false` 时立即退出并报错
- Mock 输出 topic 与正式模块一致，便于替换

## 禁止事项

- 不得订阅 `/Odometry_gazebo`
- 不得订阅 `/ground_truth/*`
- 不得读取 `danger_truth.json`（results/ 或 generated_building/）
- 不得读取 `competition_scene.world`
- 不得读取 `layout_metadata.json`
- 不得调用 Gazebo get_model_state
- 不得根据危险源模型名称生成检测结果

## 最小运行命令

```bash
source devel/setup.bash

# 仅启动 Mock 系统
roslaunch competition_testing mock_system.launch

# 或在 bringup 中以测试模式启动
roslaunch competition_bringup competition_system.launch \
  competition_mode:=false \
  use_mocks:=true
```

## 验收标准

- [x] test_mode=false 时所有 Mock 节点拒绝启动
- [x] 所有数据来自硬编码（代码审计确认）
- [x] Mock 输出 topic 与正式模块一致
- [x] 不依赖任何真值话题或文件
- [ ] 集成联调：Mock 输入 → competition_loop 正常完成状态机流转
- [ ] 集成联调：cmd_vel_guard 正常接收 raw_cmd_vel 并转发
