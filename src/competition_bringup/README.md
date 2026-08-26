# competition_bringup

**负责人**: 黄誉铭（团队负责人）

## 模块目标

统一启动文件、参数加载、正式模式与测试模式隔离。

## 启动方式

### 正式比赛模式

```bash
source devel/setup.bash

# 推荐环境变量
export ENABLE_GROUND_TRUTH=false
export ENABLE_REFEREE_ODOM=false
export WRITE_GENERATED_TRUTH_COPY=0
export UNITREE_AUTO_CMD_VEL=1
export CONTROLLER_FOREGROUND=0

roslaunch competition_bringup competition_system.launch
```

### 测试模式（使用 Mock）

```bash
roslaunch competition_bringup competition_system.launch \
  competition_mode:=false \
  use_mocks:=true
```

## 参数

见 [config/default.yaml](config/default.yaml)。所有参数可通过 ROS parameter server 覆写。
Planning 的 `visual_scan_*` 参数控制 bounded active visual coverage；Control 的
`path_follower_orientation_tolerance` 将同位置、不同朝向的 Planning waypoint
映射为原地转向容差。正式默认开启 visual scan；可在诊断 launch 中显式设置
`planning_visual_scan_enabled:=false` 做兼容性 A/B 对照。

## 模式隔离

| 属性 | 正式模式 | 测试模式 |
|------|---------|---------|
| `competition_mode` | true | false |
| `use_mocks` | false | true |
| 定位节点 | competition_localization | mock_localization |
| 感知节点 | competition_perception | mock_perception |
| 地图节点 | competition_mapping | mock_mapping |
| 规划节点 | competition_planning | mock_planning |
| 接口校验 | 严格（检查 forbidden topics） | 宽松 |
| 真实传感器 | 需要 | 不需要 |

**正式 launch 默认不启动 `competition_testing`。**

## 输入 / 输出

- **输入**: 无（纯启动/配置包）
- **输出**: 启动所有 competition_* 节点，加载参数

## 当前状态

**v0.1.0**

- [x] 统一 launch 文件
- [x] 默认参数 YAML
- [x] 正式/测试模式隔离
- [ ] 与 Gazebo 场景的集成启动
- [ ] 启动顺序依赖管理（wait-for 逻辑）

## 禁止事项

- 不得在此包中实现算法逻辑
- 正式模式不得 include mock launch

## 验收标准

- [x] launch 文件语法正确
- [x] 正式模式下不启动 testing 节点
- [x] 测试模式下 mock 替换正式节点
- [ ] 完整工作区编译通过
- [ ] launch 后所有节点正常启动

## 需要向上下游成员交付的内容

- 所有人：一键启动命令
- FFFA：启动日志和运行证据
