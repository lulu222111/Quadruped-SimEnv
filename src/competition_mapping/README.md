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

- `origin_mode: initial_pose`（正式默认）：第一组本 epoch 的合法
  Localization status 和 Odometry 到达后，将机器人初始 `(x,y)` 放在地图中央；
  此前不产生有效地图，状态保持 `valid=false`。
- `origin_mode: fixed`：使用 `origin_x/origin_y`，用于确定性测试和调试。

### latest-only 融合

点云回调只校验 frame 和时间戳并保存一帧 pending cloud；新点云覆盖未处理的
旧点云。融合定时器按 `integration_rate` 查询时间戳 TF、解码点云并更新地图，
避免在真实 Livox 点云下阻塞 Localization 回调。

### 异常门控与 epoch 恢复

Localization 失效、status/Odometry 超时或非法、位姿跳变、TF 不可用、ROS
时间回退等情况会立即使 Mapping 无效并进入新输入 epoch。恢复 `valid=true`
必须同时满足：

1. 本 epoch 的新合法 Localization status；
2. 本 epoch 的新合法 `world/base` Odometry；
3. 本 epoch 的新 `laser_livox` 点云；
4. 点云时刻两段 TF 查询成功；
5. 至少成功融合一帧且产生有效栅格更新。

成功融合点云的超时以 `last_cloud_integrated_at` 为准，而不是任意点云的接收
时间。零、过旧、未来、重复或乱序时间戳会被拒绝；ROS 时间回退会清空时间
基线、pending cloud、输入 epoch 有效标记和地图，并可在新输入到达后恢复。

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
- `scripts/mapping_node`：订阅、时间同步、TF、epoch、状态和发布；
- `config/mapping.yaml`：正式默认参数；
- `launch/mapping.launch`：正式启动文件，不启动 Mock；
- `test/test_occupancy_grid_core.py`：纯 Python 单元测试；
- `test/mapping_integration.test`：Mock Localization、PointCloud 和 TF 的 Rostest。

## 替换现有包

在 SimEnv 仓库中先提交或备份当前修改，然后用本目录内容替换
`src/competition_mapping/`。如果仓库已有真实 maintainer 姓名和邮箱，请保留；
压缩包内 `package.xml` 的邮箱仍需由团队确认，未擅自编造。

```bash
chmod +x \
  src/competition_mapping/scripts/mapping_node \
  src/competition_mapping/test/test_mapping_integration.py \
  src/competition_mapping/test/test_occupancy_grid_core.py
```

## 测试

### 1. 纯 Python 单元测试

```bash
cd "$HOME/SimEnv"
PYTHONPATH=src/competition_mapping/src \
python3 -m unittest -v \
  src/competition_mapping/test/test_occupancy_grid_core.py
```

预期：`Ran 13 tests`、`OK`。

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
```

Rostest 使用正式 `world/base/laser_livox` 契约、静态和动态 TF；其 Odometry
位置故意与动态 TF 不同，用于验证建图实际采用点云时刻 TF。测试还验证
`initial valid -> localization invalid -> 无新点云不能恢复 -> 新点云融合后恢复`
的 epoch 行为。合并门槛为 `0 errors / 0 failures / 0 timeouts`。

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
