# competition_mapping

负责人：马跃。

本包使用合法 Localization 和 Livox 点云生成固定尺寸的二维
`nav_msgs/OccupancyGrid`。当前实现是最小可用的确定性二值栅格建图，不读取
Gazebo truth、world/layout 文件，不订阅真值接口，也不发布 `/cmd_vel`。

## 冻结接口

输入：

- `/competition/localization/status`：`competition_interfaces/ModuleStatus`；
- `/competition/localization/odom`：`nav_msgs/Odometry`，必须为 `world/base`；
- `/livox/Pointcloud2`：`sensor_msgs/PointCloud2`，必须为 `laser_livox`。

`ModuleStatus.valid` 是 Localization 可用性的权威门控：`valid=true` 即视为
可用，不要求 `state==RUNNING`；`state` 只用于解释原因，并把 Localization
的 TIMEOUT / FAILED 语义映射到 Mapping 的 fault state。

TF：

- 动态 `world -> base(t_cloud)`；
- 静态 `base -> laser_livox`。

Mapping 按 `PointCloud2.header.stamp` 查询这两段 TF，不使用最新 Odometry
代替历史位姿，不使用 `rospy.Time(0)`，也不在本包中硬编码 Livox 外参。

输出：

- `/competition/map`：`nav_msgs/OccupancyGrid`，frame 为 `world`，latched；
- `/competition/mapping/status`：`competition_interfaces/ModuleStatus`，latched。

`mapping/status.valid=true` 才表示当前地图可供 Planning 使用。状态变为
`valid=false` 后，旧 latched map 仍可能存在，Planning 必须停止使用旧地图并
撤销旧路径；后者由 Planning/Loop 的独立 PR 负责。

## 关键行为

### 地图原点

- `origin_mode: initial_pose`（正式默认）：第一组本 generation 的合法
  Localization status 和 Odometry 到达后，将机器人初始 `(x,y)` 放在地图中央；
  此前不产生有效地图，状态保持 `valid=false`。
- `origin_mode: fixed`：使用 `origin_x/origin_y`，用于确定性测试和调试。

### latest-only 融合

点云回调只校验 frame 和时间戳并保存一帧 pending cloud（latest-only，最多
一帧）；新点云覆盖未处理的旧点云。融合定时器按 `integration_rate` 查询
时间戳 TF、解码点云并更新地图，避免在真实 Livox 点云下阻塞 Localization
回调。等待 Localization 恢复期间，融合定时器保留仍合法的 candidate cloud，
不会无条件消耗它。

### 失效、generation 与恢复

Localization 失效、status/Odometry 超时或非法、位姿跳变、TF 不可用、ROS
时间回退等情况会立即使 Mapping 无效。失效/恢复采用本地 generation 语义
（内部输入 epoch，不引入 `epoch_id` 等公共字段）：

- 仅在“进入新的失效”时切换一次新 generation，失效前旧 generation 缓存
  数据立即不可使用；
- 持续 invalid heartbeat 只更新 fault state / detail / 接收时间，
  不会不断创建新 generation，也不会反复失效 current-generation 候选数据；
- invalid 期间允许缓存 current-generation 候选 Odom 与最新一帧点云，
  但绝不融合点云、不发布有效地图、不恢复 `valid=true`；
- 恢复顺序无关：先到新 Odom / 新点云再到 `valid=true`，或先到
  `valid=true` 再到新 Odom / 新点云，均可恢复；
- pending 点云在真正融合前会重新校验 frame / generation / 时间戳
  （非零、非未来、不过旧，阈值 `pointcloud_timeout`）与 TF 可用性，
  接收时 fresh 不等于以后永远可用；
- 恢复 `valid=true` 必须同时满足：

1. 本 generation 的新合法 Localization status；
2. 本 generation 的新合法 `world/base` Odometry；
3. 本 generation 的新 `laser_livox` 点云；
4. 点云时刻两段 TF 查询成功；
5. 至少成功融合一帧且产生有效栅格更新。

成功融合点云的超时以 `last_cloud_integrated_at` 为准，而不是任意点云的接收
时间。零、过旧、未来、重复或乱序时间戳会被拒绝；ROS 时间回退会清空时间
基线、pending cloud、输入 epoch 有效标记和地图，并可在新输入到达后恢复。

同一 fault generation 内，新的 Odom fault（非法时间戳、非法
frame/quaternion/非有限数值、位姿跳变、Odom 超时）会强制开启新
generation，立即失效此前缓存的 Odom candidate；而持续的 Localization
`valid=false` heartbeat 不会反复开启新 generation。fault state 与 detail
始终作为一对，反映最近一次明确的 fault 报告；没有新的 fault event 时两者
保持不变。

### 栅格语义和限制

- `unknown=-1`、`free=0`、`occupied=100`；
- 同一扫描先汇总 free cells，最后写 occupied endpoints，保证 occupied 优先；
- 新扫描的 free ray 仍可清除旧扫描留下的障碍；
- 当前不是概率/log-odds 地图，没有占据概率累积、衰减或动态障碍跟踪；
- 当前高度过滤只是固定高度切片，不是地面分割；
- 当前只支持单楼层、固定二维地图。

高度阈值、融合频率、地图尺寸、跳变阈值和 timeout 仍需在真实
Localization + Livox 连续运动联调后确认。

## 文件结构

- `src/competition_mapping/occupancy_grid_core.py`：ROS 无关的栅格核心；
- `scripts/mapping_node`：订阅、时间同步、TF、generation、状态和发布；
- `config/mapping.yaml`：正式默认参数；
- `launch/mapping.launch`：正式启动文件，不启动 Mock；
- `test/test_occupancy_grid_core.py`：纯 Python、ROS 无关的栅格核心单元测试；
- `test/test_mapping_node_safety.py`：确定性单元测试（校验 / 竞态 / 恢复），
  不需要 roscore，但依赖 ROS Noetic Python 包和已生成的
  `competition_interfaces` 消息（需要 sourced 工作空间）；
- `test/mapping_integration.test`：Mock Localization、PointCloud 和 TF 的 Rostest；
- `test/mapping_faults.test`：故障注入与自动恢复 Rostest；
- `test/mapping_clock_rollback.test`：模拟时间回退 Rostest。

## 测试

### 1. 单元测试

`test_occupancy_grid_core.py` 是纯 Python、ROS 无关的测试，可在任意
Python 3 环境直接运行，不需要安装或 source ROS：

```bash
cd "$HOME/SimEnv"
python3 -m unittest -v \
  src/competition_mapping/test/test_occupancy_grid_core.py
```

`test_mapping_node_safety.py` 是确定性单元测试，**不需要 roscore**，但
依赖 ROS Noetic 的 Python 包（`rospy`、`nav_msgs`、`sensor_msgs`、
`geometry_msgs`、`std_msgs`）以及已生成的 `competition_interfaces`
Python 消息，因此需要 `source /opt/ros/noetic/setup.bash` 并 source
含 `competition_interfaces` 的 catkin 工作空间（或等价设置
`PYTHONPATH`）：

```bash
cd "$HOME/SimEnv"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
PYTHONPATH=src/competition_mapping/src:$PYTHONPATH \
python3 -m unittest -v \
  src/competition_mapping/test/test_mapping_node_safety.py
```

`test_occupancy_grid_core.py` 覆盖 unknown/free/occupied 语义、occupied
endpoint 优先、free ray 清除、距离/高度过滤和 ROS 无关的坐标变换。
`test_mapping_node_safety.py` 覆盖 frame 校验、零/未来/过旧/乱序时间戳、
ROS 时间回退、map 快照竞态、状态发布原子性，以及失效 generation、候选
缓存、恢复顺序无关、pending cloud freshness 重校验、valid-only usability、
fault state/detail 与最近一次 fault 报告的对应，以及 Odom fault（非法时间
戳、非法 frame/quaternion/非有限数值、位姿跳变、Odom 超时）强制新
generation 并失效缓存 Odom candidate 的回归。

### 2. 编译和 Rostest

在个人容器中执行：

```bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash

python3 -m py_compile \
  src/competition_mapping/src/competition_mapping/*.py \
  src/competition_mapping/scripts/mapping_node \
  src/competition_mapping/test/*.py

catkin_make --pkg competition_mapping
source devel/setup.bash
rostest competition_mapping mapping_integration.test
rostest competition_mapping mapping_faults.test
rostest competition_mapping mapping_clock_rollback.test
```

- `mapping_integration.test`：正式 `world/base/laser_livox` 契约、静态和动态
  TF；其 Odometry 位置故意与动态 TF 不同，用于验证建图实际采用点云时刻 TF；
  并验证 `initial valid -> localization invalid -> 无新点云不能恢复 -> 新点云
  融合后恢复`；
- `mapping_faults.test`：TF 缺失、错误点云 frame、status/Odometry/点云超时
  及自动恢复；
- `mapping_clock_rollback.test`：ROS `/clock` 回退后的失效与恢复。

合并门槛为 `0 errors / 0 failures / 0 timeouts`。

### 3. 真实仿真联调

只启动一套共享 Gazebo，并关闭真值接口。确认上游后再启动 Mapping：

```bash
rostopic hz /livox/Pointcloud2
rostopic echo -n 1 /livox/Pointcloud2/header
rostopic echo -n 1 /competition/localization/odom/header
rostopic echo -n 1 /competition/localization/odom/child_frame_id
rosrun tf tf_echo world base
rosrun tf tf_echo base laser_livox
roslaunch competition_mapping mapping.launch
```

RViz 的 Fixed Frame 设为 `world`，添加 `/competition/map` 和
`/livox/Pointcloud2`。重点确认地图不随机器人旋转/漂移、墙体连续、机器人周围
存在 free、未观测区域仍为 unknown，并记录真实点云 z 分布后再调整高度阈值。

## 合规检查

```bash
rg -n \
  '/Odometry_gazebo|/ground_truth/|get_model_state|danger_truth.json|layout_metadata.json|competition_scene.world|scene_manifest.json|/cmd_vel' \
  src/competition_mapping
```

可执行 Python、launch 和 config 不应命中任何禁用接口；README 中的说明文字可
能命中。

## 多楼层地图与电梯过渡

### 地图模型

`FloorMapManager` 为每个已访问楼层保存独立的：

- `OccupancyGridCore`；
- 是否已经形成有效观测；
- `map_load_time`；
- 最近一次扫描统计。

所有楼层共享 `world` 下相同的分辨率、尺寸和 XY 原点。切到新楼层时创建
全 unknown 地图；切回旧楼层时恢复原对象和全部历史栅格，不复制、不清空。

### 临时兼容接口

在团队正式冻结 `FloorState` 消息前，本包使用一个参数化的 `std_msgs/Int32`
适配层：

| 方向 | Topic | 类型 | 语义 |
|---|---|---|---|
| 输入 | `/competition/floor_state` | `std_msgs/Int32` | `-1`=电梯过渡；`>=0`=已稳定到达该楼层 |
| 输出 | `/competition/mapping/current_floor` | `std_msgs/Int32` | 当前激活楼层，latched |

正确时序必须是：

```text
进入电梯前发布 -1
→ Mapping 立即 valid=false、停止融合并清除 pending cloud
→ 执行电梯运动
→ 到达并稳定后发布目标 floor_id
→ Mapping 切换/恢复目标层
→ 收到新一代 Localization、Odometry、PointCloud 后恢复 valid=true
```

这个适配层不读取 Gazebo、world、layout 或楼层高度。正式 `FloorState` 接口确定
后，只替换 `mapping_node` 的订阅回调；`FloorMapManager` 不需要修改。

手工 smoke（只验证 Mapping 门控，不代表正式电梯联调）：

```bash
rostopic pub -1 /competition/floor_state std_msgs/Int32 "data: -1"
rostopic echo -n 1 /competition/mapping/status

rostopic pub -1 /competition/floor_state std_msgs/Int32 "data: 1"
rostopic echo -n 1 /competition/mapping/current_floor
```

### 多楼层测试

```bash
PYTHONPATH=src/competition_mapping/src:$PYTHONPATH \
python3 src/competition_mapping/test/test_floor_map_manager.py
```

测试覆盖楼层隔离、首次进入全 unknown、历史层恢复、当前层 reset、全层 reset、
时间线清空和共享地图几何重建。`test_mapping_node_safety.py` 另覆盖 transition
期间拒绝点云、切层推进 input epoch、新楼层必须使用新一代输入以及旧楼层地图
快照不得跨层发布。
