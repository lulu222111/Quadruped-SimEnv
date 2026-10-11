# competition_localization

**负责人**: 纪雯瀚（定位与状态估计）
**集成**: fix/huangym/localization-final-integration

## 模块目标

使用允许的 Livox 点云、IMU 和公开场景信息估计机器人位姿（`world` 系）。
严禁使用 Gazebo 真值话题或服务。

## 输入

| 输入 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | Livox 内置 IMU |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | ~10 Hz | 转换后点云 (`frame_id=laser_livox`) |
| `/Odometry` | `nav_msgs/Odometry` | ~100 Hz | FAST-LIO 输出 (`camera_init`→`body`) |
| `team_scene_info.json` | 文件 | 一次性 | 起点信息用于初始对齐 |
| TF `base -> laser_livox` | static TF | — | URDF + robot_state_publisher 提供 |
| TF `base -> livox_imu_link` | static TF | — | URDF + robot_state_publisher 提供 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/localization/odom` | `nav_msgs/Odometry` | mapping, planning, control | 估计位姿 (`frame_id=world`, `child_frame_id=base`) |
| `/competition/localization/status` | `ModuleStatus` | competition_loop | 定位模块状态 (`valid=true/false`) |
| TF `world -> base` | transform | 全系统 | **唯一**动态 `world -> base` TF 发布者 |

### 坐标系契约

- `odom` 帧已从全系统移除。定位模块是 `world -> base` 的唯一动态 TF 发布者。
- 动态 TF 的位姿与时间戳和 `/competition/localization/odom` 消息完全一致。
- `base -> laser_livox` / `base -> livox_imu_link` 静态外参仅由 URDF + `robot_state_publisher` 提供。

### 协方差

**当前帧变换时 covariance 统一标记为 UNKNOWN (`[0.0] * 36`)。**

FAST-LIO 的 covariance 表达在 IMU/body 帧中，直接复制到经过 frame 对齐和
IMU→base reference-point 变换后的 `/competition/localization/odom` 在数学上不正确。
在实现正确的 covariance propagation（$J \cdot C \cdot J^T$，包含旋转 Jacobian）之前，
**下游模块不得依赖定位协方差进行决策**。

### Twist

按照 v0.2.0 契约，当前 `/competition/localization/odom` 不提供 twist 估计（保持全零）。

---

## FAST-LIO 坐标系与 frames

```
FAST-LIO 发布:
  header.frame_id = "camera_init"    (第一个关键帧位姿)
  child_frame_id  = "body"           (IMU 帧, 即 livox_imu_link)

localization_node 转换链:
  1. 锁存第一帧 P0 (camera_init -> body at t0)
  2. 从 team_scene_info.json 读取 world 起点 T_start (world -> base)
  3. 计算 T_world_to_camera_init = T_start * T_base_to_imu * P0^{-1}
  4. 每帧: T_world_to_base = T_world_to_camera_init * T_camera_init_to_body * (T_base_to_imu)^{-1}
  5. 发布 world -> base TF
```

### 外参定义 (FAST-LIO)

FAST-LIO 的 `extrinsic_T` = `Lidar_T_wrt_IMU` = LiDAR 原点在 IMU 坐标系中的位置。

**源码依据**:
- `laserMapping.cpp:818`: `Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);`
- `IMU_Processing.hpp:343`: `P_compensate = offset_R_L_I * P_lidar + offset_T_L_I`

即点从 LiDAR 帧变换到 IMU 帧: p_imu = R_L_I * p_lidar + T_L_I。

**URDF 推导** (a1_description/xacro/robot.xacro):

```
joint base -> laser_livox:
  origin: xyz="0.2 0 0.08" rpy="0 0.785 0"
  = T_base_lidar = transl(0.2, 0, 0.08) * rot_y(45°)

joint laser_livox -> livox_imu_link:
  parent="laser_livox", child="livox_imu_link"
  origin: xyz="-0.011 -0.02329 0.04412" rpy="0 0 0"
  = T_lidar_imu = transl(-0.011, -0.02329, 0.04412) (rotation = I)
```

URDF joint 定义从 parent (LiDAR) 到 child (IMU) 的变换，即 IMU 原点在 LiDAR 帧:
T_lidar_imu = transl(-0.011, -0.02329, 0.04412)

FAST-LIO 需要 T_imu_lidar (LiDAR 原点在 IMU 帧):
T_imu_lidar = -T_lidar_imu = [0.011, 0.02329, -0.04412] (旋转为 I)

**最终 extrinsic_T: `[0.011, 0.02329, -0.04412]`**

> **官方比赛文档依据**：
> - `laser_livox`（父 `base`）：`xyz=(0.2, 0.0, 0.08)`，`rpy=(0.0, 0.785, 0.0)`（绕 Y 轴 45°）；
> - `livox_imu_link`（父 `laser_livox`）：`(-0.011, -0.02329, 0.04412)`，`(0.0, 0.0, 0.0)`。
> 与 URDF `robot.xacro` 一致，构成 `base_to_imu = [0.223404, -0.023290, 0.118985], Ry(45°)` 与 `extrinsic_T = [0.011, 0.02329, -0.04412]` 的权威基准。

> ⚠️ **已知 sign error 警示**：`fast_lio` 子模块 `config/mid360_sim.yaml` 默认 `extrinsic_T = [-0.011, -0.02329, 0.04412]`（把 IMU-in-LiDAR 当成 LiDAR-in-IMU，方向反了）。**生产必须经 `localization_bringup.launch` 覆盖为 `[0.011, 0.02329, -0.04412]`**；不要直接运行 FAST-LIO 自带 `mapping_mid360_sim.launch`。

---

## 启动 (生产链路)

### 标准链路 (生产模式)

```bash
# 生产模式: competition_system.launch 已 include localization_bringup.launch
roslaunch competition_bringup competition_system.launch
```

### 独立启动

```bash
# 标准启动 (FAST-LIO + 适配节点):
roslaunch competition_localization localization_bringup.launch rviz:=false

# 调试模式 (rviz 可视化):
roslaunch competition_localization localization_bringup.launch rviz:=true
```

### 腿式里程计校正实验链的合规状态

`localization_corrected.launch` 是 **EXPERIMENTAL / VALIDATION ONLY** 候选，尚未获准
进入正式比赛模式。它现在可由单条命令复现完整实验链：

```bash
roslaunch competition_localization localization_corrected.launch
```

该命令同时启动 `filter_livox.py`、FAST-LIO、`leg_odometry.py` 和
`localization_node`。腿式校正使用：

- `/a1_gazebo/joint_states`：本体关节状态，用于足端正运动学与速度估计；
- `/trunk_imu`：躯干姿态与角速度，用于身体旋转和世界/本体系转换。

在把该链标记为正式比赛生产定位方案之前，必须由赛题规则负责人明确确认：

1. `joint_states` 对应的数据是否属于机器人正常可用的本体状态；
2. `trunk_imu` 是否属于正式允许的机载传感器；
3. Gazebo 中这些 topic 是否包含实机不可获得的理想状态；
4. 实物迁移时是否存在语义和时序等价的输入。

因此当前结论仅为 `FORMAL_SENSOR_COMPLIANCE_CONFIRMED=False`，既不擅自判断
`LEGAL=True`，也不判断 `ILLEGAL=True`。正式 `competition_system.launch` 仍只 include
`localization_bringup.launch`，不会自动启用该实验链。

校正开启后是 required input：任一模式在 correction 从未到达、接收超时、source
stamp skew 超限、frame/pose/timestamp 非法时都停止发布 competition odom/TF，并令
`valid=false`。普通 timeout 会丢弃 latest measurement、保留相对 alignment offset；
只有新合法 correction 到达，且下一帧 FAST-LIO 与其时间差合规，才恢复 `RUNNING`。
correction timestamp 或 ROS `/clock` rollback 会进一步清除 alignment generation。

> FOLLOWUP: replace spherical near-field crop with robot self-mask evaluation.
> 本轮保持 `min_range=1.0m`、`max_range=40m` 及所有 FAST-LIO/腿里程计算法参数不变。

### Debug vs Competition 区别

| 项目 | 生产模式 | 调试模式 |
|------|---------|---------|
| `/competition_loop/competition_mode` | `true` | `false` |
| respawn (FAST-LIO + adapter) | `false` | 可手动配置 |
| PCD 保存 | `false` | 可手动打开 |
| 外参缺失 | FATAL + 终止 | 告警 + 降级 identity |
| 场景文件缺失/损坏 | FATAL + 终止 | 告警 + valid=false |

---

## 时间戳安全

- **Zero stamp**: 拒绝 time_sec ≤ 0 的输入
- **Stamp rollback**: 拒绝时间戳回跳的输入 (可恢复: 下一个合法消息恢复 RUNNING)
- **Future stamp**: 拒绝超过 1 秒的未来时间戳 (clock skew guard)
- **Stale data**: 拒绝超过 `input_timeout × 2` 的过时数据
- **ROS /clock rollback**: 检测到回跳后重置 last_input_stamp 并继续
- **Correction required**: `correction_enabled=true` 时所有 correction mode 均禁止 raw FAST-LIO 静默回退
- **Correction freshness**: correction 接收年龄不超过 `correction_timeout`，且与 FAST-LIO source stamp 差不超过 `max_correction_skew`

## 超时与状态

- **STARTING**: 等待首个合法 FAST-LIO 输入
- **RUNNING + valid=true**: 接收合法输入, 发布 world->base odom + TF
- **TIMEOUT + valid=false**: `input_timeout` 秒内无合法输入 (可恢复)
- **FAILED + valid=false**: 场景文件/外参配置错误 (不可恢复, 需重启)
- correction 非法 pose/stamp 为 **FAILED**，frame/skew 不匹配为 **BLOCKED**；新合法 generation 可恢复

---

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~publish_tf` | bool | true | 是否发布动态 TF `world -> base` |
| `~base_frame` | string | `base` | 机器人基坐标系 |
| `~imu_frame` | string | `livox_imu_link` | FAST-LIO `body` 对应 IMU 坐标系 |
| `~input_frame_id` | string | `camera_init` | FAST-LIO 输入 `header.frame_id` 期望值 |
| `~input_child_frame_id` | string | `body` | FAST-LIO 输入 `child_frame_id` 期望值 |
| `~input_timeout` | float | 0.5 | 看门狗: 超时无输入即 `TIMEOUT` (秒) |
| `~status_rate` | float | 10.0 | 状态发布频率 (Hz) |
| `~start_alignment_delay` | float | 2.0 | 对齐前的输入稳定等待时长 (秒) |
| `~odom_input_topic` | string | `/Odometry` | FAST-LIO 里程计话题 |
| `~base_to_imu_translation` | list[3] | — | `base -> livox_imu_link` 平移 [x,y,z] |
| `~base_to_imu_rotation_rpy` | list[3] | — | `base -> livox_imu_link` 旋转 RPY [rad] (与 quaternion 二选一) |
| `~base_to_imu_quaternion` | list[4] | — | 旋转的四元数表示 (与 rpy 二选一) |
| `/competition_loop/competition_mode` | bool | true | 正式模式: 严格校验、禁止静默降级 |
| `~team_scene_info_path` | string | `generated_building/team_scene_info.json` | 公开场景文件路径 |
| `~correction_enabled` | bool | false | 启用时 correction 成为 required input，不允许 raw fallback |
| `~correction_mode` | string | `full_pose` | `full_pose` / `yaw_only` / `planar_relative` |
| `~correction_timeout` | float | 0.5 | correction 最大接收年龄（秒） |
| `~max_correction_skew` | float | 0.20 | FAST-LIO 与 correction source stamp 最大绝对差（秒） |

实验 `leg_odometry.py` 另有 `~max_imu_age=0.5` 与
`~max_imu_joint_skew=0.05`。它拒绝 zero stamp、缺失 required joint、未准备/过期/错位
IMU；无可靠支撑脚时不积分且不发布 fresh correction。其 Odometry pose 位于
`c2r28_fastlio_local`，twist.linear 按 ROS 约定表达在 child `c2r28_base` 中。

---

## 验证

### 单元测试

```bash
# pytest
python3 -m pytest src/competition_localization/test/ -q

# 单个测试文件
python3 src/competition_localization/test/test_localization_node.py
python3 src/competition_localization/test/test_localization_extrinsic.py
python3 src/competition_localization/test/test_localization_scene_file.py
python3 src/competition_localization/test/test_leg_odometry.py
python3 src/competition_localization/test/test_corrected_launch.py
```

覆盖: 外参矩阵、首帧锁存、场景文件异常、静态稳定性、运动方向、TF 持续可查、
FAST-LIO/correction 双输入 fail-close、correction 恢复/rollback、腿里程计 freshness/
支撑脚/twist frame，以及 corrected launch 完整链与生产隔离。

### 集成烟雾测试

```bash
scripts/verify_localization.sh
```

该脚本启动私有 roscore + 标准 localization_bringup.launch，验证:
- `/competition/localization/odom` 类型为 `nav_msgs/Odometry`
- `/competition/localization/status` 类型为 `competition_interfaces/ModuleStatus`
- `frame_id=world`, `child_frame_id=base`
- TF `world -> base` 正常发布
- 输入超时后 `valid=false`
- 未使用禁止话题 (`/Odometry_gazebo`, `/ground_truth/*`, `/cmd_vel`)

---

## 禁止接口

- 不得订阅 `/Odometry_gazebo`
- 不得订阅 `/ground_truth/*`
- 不得调用 Gazebo `get_model_state` 获取机器人真值位姿
- 不得读取 `results/danger_truth.json` 或 `generated_building/danger_truth.json`
- 不得读取 `layout_metadata.json`、`building_config.json`、`scene_manifest.json`
- 不得读取 `results/` 下任何文件

---

## 当前状态

**v0.2.0 — FAST-LIO 适配器 (world/base 正式契约)**

- [x] 接收 FAST-LIO `/Odometry` (`camera_init` → `body`)
- [x] 首帧锁存 + 基于 P0 的起点对齐
- [x] `base -> livox_imu_link` 外参补偿 (`world -> base`)
- [x] 发布 `/competition/localization/odom` (`world`/`base`)
- [x] 动态 TF `world -> base` (与 odom 数值/时间戳一致)
- [x] 场景文件严格校验与安全失败 (无静默回退)
- [x] 正式模式外参缺失 FATAL + 终止
- [x] 输入 Frame 严格校验
- [x] 时间戳安全 (zero/rollback/future/stale)
- [x] 看门狗 (`TIMEOUT`/`RUNNING`/`FAILED` 状态转换)
- [x] covariance = unknown (全零, 下游不得依赖)
- [x] FAST-LIO extrinsic_T 正负号修正 (源码推导)
- [x] PCD 保存默认关闭 (生产模式)
- [x] respawn=false (防止 FAST-LIO 重启后对齐漂移)
- [x] 单元测试 + 集成烟雾测试
- [ ] 位姿协方差正确传播 (当前 unknown)
- [ ] 回环检测与优化
- [ ] 长距离漂移 < 1m / 100m

---

## 需要向上下游成员交付的内容

- 马跃 (mapping): `/competition/localization/odom` + TF `world -> base`
- 闫雨萱 (planning): `/competition/localization/odom` + TF `world -> base`
- 黄誉铭 (loop): 定位 `valid` 标志 + `ModuleStatus.state` 用于判断模块就绪
