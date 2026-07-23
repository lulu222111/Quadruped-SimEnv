# competition_interfaces

**负责人**: 黄誉铭

**职责**: 维护跨模块 ROS 消息和固定接口。任何成员不得自行修改公共接口后直接提交；
所有公共消息变更必须单独提出 PR 并通知全组。

## 消息列表

| 消息 | 文件 | 用途 |
|------|------|------|
| `ModuleStatus` | [msg/ModuleStatus.msg](msg/ModuleStatus.msg) | 各模块状态报告 |
| `HazardCandidate` | [msg/HazardCandidate.msg](msg/HazardCandidate.msg) | 单个危险源候选 |
| `HazardCandidateArray` | [msg/HazardCandidateArray.msg](msg/HazardCandidateArray.msg) | 危险源候选批量 |
| `CompetitionState` | [msg/CompetitionState.msg](msg/CompetitionState.msg) | 全局比赛状态 |

## 当前状态

v0.1.0 — 第一版消息定义已冻结。后续修改需走变更流程。

## 输入 / 输出

- **输入**: 无（纯消息定义包，不包含运行时节点）
- **输出**: ROS 消息头文件供所有 competition_* 包使用

## 参数

无

## 禁止事项

- 禁止在此包中添加执行逻辑或节点
- 禁止为已有 ROS 标准消息重复创建自定义消息（使用 `nav_msgs/Odometry`、`geometry_msgs/Twist` 等标准消息）
- 禁止绕过团队负责人直接修改公共接口

## 验收标准

- [x] 4 种消息定义完成
- [x] package.xml 依赖正确（message_generation / message_runtime / std_msgs / geometry_msgs）
- [x] CMakeLists.txt 正确配置 generate_messages
- [ ] catkin_make 编译通过（待完整工作区编译）
- [ ] 生成的头文件可被其他包 include

## 最小运行命令

```bash
# 编译此包（需完整工作区）
cd /workspace/SimEnv && source devel/setup.bash
catkin_make --pkg competition_interfaces

# 查看生成的消息
rosmsg show competition_interfaces/ModuleStatus
rosmsg show competition_interfaces/HazardCandidate
rosmsg show competition_interfaces/HazardCandidateArray
rosmsg show competition_interfaces/CompetitionState
```

## 需要向上下游成员交付的内容

- 黄誉铭（competition_loop）：CompetitionState 用于状态广播
- 纪雯瀚（competition_localization）：ModuleStatus 用于定位状态
- 鲍语菲（competition_perception）：HazardCandidate/HazardCandidateArray 用于候选输出
- 马跃（competition_mapping）：ModuleStatus 用于地图状态
- 闫雨萱（competition_planning）：ModuleStatus 用于规划状态
- 杨渊浩（competition_control）：ModuleStatus 用于控制状态
