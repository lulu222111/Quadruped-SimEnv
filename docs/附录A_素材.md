# 附录A 软件系统实现与代码组织素材

> 本文件为后续技术报告撰写所需的**事实素材**（非宣传稿）。
> 所有内容均取自本提交包内源码，可直接引用；涉及"是否已验证"的表述一律按
> 实际验证状态标注（`README_提交说明.md` 第 1、2 节为权威口径）。

---

## A.1 软件总体结构

### A.1.1 数据流（两种模式共同）

```text
传感器层
  Livox Mid-360 点云 /scan、Livox IMU /livox/imu、机体 IMU /trunk_imu、
  RealSense RGB /real_sense/rgb/image_raw、深度 /real_sense/depth/image_raw
  （以及两者的 CameraInfo）、A1 关节状态 /joint_states
        │
        ▼
定位层（competition_localization）
  filter_livox 裁剪点云 → FAST-LIO 里程 → localization_node 适配为 world→base
  （外参补偿 + 唯一动态 TF；可选腿式里程计 leg_odometry 做平面校正）
        │  /competition/localization/odom, /competition/localization/status
        ▼
建图层（competition_mapping）
  mapping_node：点云 + 位姿 → 固定尺寸 2D 占据栅格（代际/故障安全模型）
        │  /competition/map, /competition/mapping/status
        ▼
规划层（competition_planning）
  planning_node：Frontier 选择 / 目标管理 / 视觉覆盖扫描 / 返航路径
  纯算法：frontier.py、path_planner.py（A*）、planning_core.py、visual_coverage.py
        │  /competition/path（唯一路径来源）, /competition/planning/status
        ▼
感知层（competition_perception）
  perception_node：同步 RGB-D → perception_core 检测 → 三维反投影
        │  /competition/hazard_candidates, /competition/perception/status
        ▼
控制层（competition_control）
  path_follower_node：跟随路径 → /competition/raw_cmd_vel（期望速度）
  cmd_vel_guard_node：限幅 + 安全裁决 → /cmd_vel（唯一出口）
        │  /cmd_vel, /competition/control/status
        ▼
任务编排层（competition_loop）
  competition_loop_node：顶层 FSM + 计时 + 接口合规校验 + 危险源去重
  + 结果写入 + 安全信号（enabled / emergency_stop）
        │  /competition/state, /competition/enabled, /competition/emergency_stop
        ▼
输出层
  results/detected_danger.json（官方 schema，原子写入）
  地图 / 路径 / 状态 / 日志（ROS 日志与 Gazebo/控制器日志）
```

设计原则（贯穿全系统，可写入报告"体系结构"小节）：

1. **单一运动源**：`/cmd_vel` 只由 `cmd_vel_guard_node` 发布，跟随器只发
   `raw_cmd_vel`；模式 B 的电梯/门槛执行器通过 `/path_follower/execution_mode`
   （normal/hold/precision）仲裁运动所有权，而不是并列发速度。
2. **单一定位源**：`world→base` 动态 TF 只能有一个发布者；装配处用
   `localization_enabled` 显式选择由哪条链承担。
3. **失效安全（fail-safe）**：任何输入过期/时间回退/接口不合规/参数非法
   都趋向"不发速度或进入 ERROR"，而不是继续用陈旧数据。
4. **算法与 ROS 解耦**：`*_core.py`、`frontier.py`、`path_planner.py`、
   `visual_coverage.py`、`room_access.py`、`occupancy_grid_core.py` 等不含 ROS
   依赖，可用合成输入做确定性验证；`*_node` 只做话题/锁/状态机与发布纪律。
5. **合规**：生产链路不消费真值/裁判通道与非公开文件（见 A.6）。

### A.1.2 模式 A 与模式 B 的差异

| 维度 | 模式 A（单层完整闭环，已验证） | 模式 B（真实比赛完整流程，代码完成） |
|---|---|---|
| 启动入口 | `competition_system.launch`（生产）/ `tools/baselines/floor0_video_return_v1/floor0_full_replay.launch`（验证） | `competition_pipeline_full.launch`（wrapper） |
| `mission_mode` | `single_floor`（默认） | `multi_floor`（wrapper 强制） |
| 电梯执行器 | 不启动（`elevator_executor_enabled=false`） | 启动（`=true`） |
| 地图 | 单张固定尺寸栅格 | 每层一张栅格（`floor_map_manager.py`），切换楼层只切激活指针 |
| 任务 FSM | 顶层 FSM（WAITING/RUNNING/RETURNING/FINISHED/ERROR）+ 楼层完成门 | 顶层 FSM + 多楼层子 FSM（`floor_mission_fsm.py`）与楼层计划（`mission_plan.py`） |
| 危险源 registry | 单层去重 | 全局 registry，跨楼层不清空，仅改变去重作用域 |
| 返航 | 直接返航（RETURNING → ARRIVED_START） | 先 `RETURN_TO_START_FLOOR`（乘电梯回起始楼层）再 `RETURN_HOME` |
| 中投入口 | 仅 `localization_bringup` / `localization_corrected` | 增加 `elevator_locator`（规划侧）与 `door_elevator_executor`（控制侧） |
| 验证状态 | **runtime validated**（3 次 PASS） | **build + unit + static PASS；端到端未验证** |

---

## A.2 核心代码目录

模式 A（`baseline_single_floor/`）实际目录（`src/` 两层）：

```text
src/
├── competition_localization/     # 定位适配：FAST-LIO→world/base、点云裁剪、腿式里程计
├── competition_mapping/          # 占据栅格建图（ROS 封装 + 无 ROS 核心算法）
├── competition_planning/         # Frontier 探索、A* 寻路、视觉覆盖、房间识别
├── competition_perception/       # RGB-D 危险源检测与三维定位
├── competition_control/          # 路径跟随 + /cmd_vel 安全网关
├── competition_loop/             # 顶层 FSM、计时、合规校验、去重与结果写入
├── competition_interfaces/       # 比赛内部 msg 契约
├── competition_bringup/          # 统一 launch 装配与默认参数
├── competition_testing/          # mock 节点（联调用，运行时可被 include）
├── competition_recovery_testing/ # 直线段/传送恢复等受控实验工具
├── building_generator_core/      # 楼栋生成核心（场景几何）
├── building_generator_classic/   # 楼栋生成 + 门/电梯控制服务
├── building_generator_interfaces/# 门/电梯服务定义
├── building_obstacles/           # 场景生成脚本（generate_competition_scene.py）与评估脚本
├── fast_lio/                     # LiDAR-惯性里程计（子模块，已实体化）
├── livox_ros_driver/             # Livox 驱动（子模块，已实体化）
├── Mid360_imu_sim/               # Gazebo 激光仿真插件与 Mid-360 模型
└── unitree_guide/                # A1 四足模型/控制器（junior_ctrl，含 RL 策略权重）
```

模式 B（`baseline_full_competition/`）目录结构相同，差异在包内文件：

```text
competition_mapping/src/competition_mapping/floor_map_manager.py     # 多楼层地图管理
competition_loop/src/mission_plan.py                                  # 楼层计划（公开信息排序）
competition_loop/src/floor_mission_fsm.py                             # 多楼层子 FSM
competition_planning/scripts/elevator_locator.py                      # 电梯门口袋定位
competition_control/scripts/door_elevator_executor_node               # 电梯/门槛执行器
competition_control/scripts/elevator_executor_core.py
competition_control/scripts/elevator_service_sequence.py
competition_control/scripts/elevator_threshold_profile.py
competition_bringup/launch/competition_pipeline_full.launch          # MODE B 入口
```

各包一句话职责：

| 包 | 职责（一句话） |
|---|---|
| `competition_localization` | 把 FAST-LIO 输出适配成比赛定位接口，并维护 `world→base` 一致性 |
| `competition_mapping` | 维护占据栅格（模式 B 为每层独立栅格），负责地图生命周期与求解时刻安全 |
| `competition_planning` | 决定"下一步去哪、怎么走、在哪扫描"，并合法发现电梯厅门口 |
| `competition_perception` | 从 RGB-D 产生危险源观测候选（检测 + 三维定位） |
| `competition_control` | 唯一的 `/cmd_vel` 输出通道、路径跟随与执行器运动仲裁 |
| `competition_loop` | 任务状态机、多楼层编排、危险源去重与结果落盘 |
| `competition_interfaces` | 定义模块间消息契约（含 `ElevatorMission` / `ElevatorStatus`） |
| `competition_bringup` | 把各模块按 MODE A / MODE B 组装起来并集中暴露参数 |
| `building_generator_*` / `building_obstacles` | 场景（楼栋/房间/危险源/门/电梯）生成与控制服务、结果评估 |
| `unitree_guide` | A1 四足模型、Gazebo 接口与 RL/`cmd_vel` 控制器 |
| `Mid360_imu_sim` | Gazebo 激光仿真插件与 Mid-360 传感器模型 |
| `fast_lio` / `livox_ros_driver` | 激光惯性里程计与 Livox 数据驱动 |

---

## A.3 主要程序模块

### A.3.1 定位

| 项 | 内容 |
|---|---|
| 模块 | 定位适配（`competition_localization`） |
| 关键源文件 | `scripts/localization_node`（主）、`scripts/filter_livox.py`、`scripts/leg_odometry.py`、`launch/localization_bringup.launch`、`launch/localization_corrected.launch` |
| 主要职责 | 把 FAST-LIO 的 body（= IMU 帧）位姿转换为 `world→base`，补偿固定外参，作为该 TF 链唯一动态发布者；启动期对齐世界系（重力方向定水平、LiDAR 楼板回波定 z 原点） |
| 关键输入 | `/Odometry`（FAST-LIO）、`/livox/Pointcloud2`、`/trunk_imu`、`/joint_states`、公开场景文件中的起点 |
| 关键输出 | `/competition/localization/odom`、`world→base` TF、`/competition/localization/status` |
| 重要接口 | 校正源（腿式里程计）话题可配；校正模式 `planar_relative`（xy/yaw 取校正源，z/roll/pitch 取 FAST-LIO）；失败时不发布（fail-closed） |

### A.3.2 建图

| 项 | 内容 |
|---|---|
| 模块 | 2D 占据栅格建图（`competition_mapping`） |
| 关键源文件 | `scripts/mapping_node`、`src/competition_mapping/occupancy_grid_core.py`、`config/mapping.yaml` |
| 主要职责 | 点云 + 位姿 → 固定尺寸栅格；用"占据证据 vs 空闲证据"抑制单帧杂点；输入过期/时钟回退时进入故障并开启新代际 |
| 关键输入 | `/livox/Pointcloud2`、`/competition/localization/odom`、`/competition/localization/status` |
| 关键输出 | `/competition/map`（`nav_msgs/OccupancyGrid`）、`/competition/mapping/status` |
| 重要接口 | 分辨率 0.1 m、地图 400×400 格、高度窗 `min_height=-0.15`（排除 base 系地面回波） |

### A.3.3 Frontier 自主探索

| 项 | 内容 |
|---|---|
| 模块 | 探索规划（`competition_planning`） |
| 关键源文件 | `scripts/planning_node`（ROS 编排）、`scripts/frontier.py`（提取/聚类/选择）、`scripts/planning_core.py`（选点+寻路流水线）、`scripts/path_planner.py`（A* + 膨胀） |
| 主要职责 | 提取"自由空间与未知空间交界"→ 8 连通聚类 → 按尺寸/距离/朝向（+房间偏置）打分选簇 → 生成四连通路径；探索完成需连续多次确认（去抖） |
| 关键输入 | `/competition/map`、`/competition/localization/odom`、上游状态、比赛状态 |
| 关键输出 | `/competition/path`、导航目标、`/competition/planning/status`、`/competition/planning/exploration_complete` |
| 重要接口 | 唯一路径来源（供跟随器与执行器消费）；BLOCKED/UNREACHABLE 有明确状态码而非静默 |

### A.3.4 视觉覆盖（主动扫描）

| 项 | 内容 |
|---|---|
| 模块 | 有界主动视觉覆盖（`competition_planning`） |
| 关键源文件 | `scripts/visual_coverage.py`（状态机）、`scripts/room_access.py`（房间块识别）、`planning_node`（发布旋转路径与计时） |
| 主要职责 | 到达探索目标后，在原地按 `yaw_step` 生成一串朝向并逐次转到位/驻留，保证前向相机视野覆盖四周；只有"全部朝向都到位"才记为该位置已覆盖 |
| 关键输入 | 当前 yaw/时间、目标位置、房间块（由占据栅格的距离场在线识别，不读布局文件） |
| 关键输出 | 旋转型路径（仅含朝向的路径点）、扫描事件序列、覆盖/失败冷却状态 |
| 重要接口 | `planning_visual_scan_*` 参数族（步长/驻留/超时/重试/位置去重半径/路线间距/完成前最后一次扫描） |

### A.3.5 全局/路径规划与运动控制

| 项 | 内容 |
|---|---|
| 模块 | 路径生成与跟随（`competition_planning` + `competition_control`） |
| 关键源文件 | `planning_core.py`（`plan_to_frontier_with_status` / `plan_to_goal_with_status` / `plan_to_heading_goal_with_status`）、`path_follower_node`、`cmd_vel_guard_node` |
| 主要职责 | 在障碍膨胀后的栅格上搜索路径并输出带朝向的路径；跟随器做比例转向+前进，输出期望速度；网关限幅并做安全裁决后发布 `/cmd_vel` |
| 关键输入 | 栅格、位姿、上游状态、`/competition/path` |
| 关键输出 | `/competition/path`、`/competition/raw_cmd_vel`、`/cmd_vel`、`/competition/control/status`、`/competition/planning/stuck` |
| 重要接口 | 单一 motion owner；`/path_follower/execution_mode`（normal/hold/precision）供模式 B 执行器仲裁 |

### A.3.6 任务状态机与返航

| 项 | 内容 |
|---|---|
| 模块 | 任务编排（`competition_loop`） |
| 关键源文件 | `scripts/competition_loop_node`、`src/competition_state_machine.py`、`src/floor_lifecycle.py`、`src/time_keeper.py`、`src/interface_validator.py`、`src/door_controller.py`；（模式 B 另加 `src/floor_mission_fsm.py`、`src/mission_plan.py`） |
| 主要职责 | 维护 `WAITING/RUNNING/RETURNING/FINISHED/ERROR`；维护竞赛计时；校验接口合规；驱动返航与到达判定；发布使能/急停 |
| 关键输入 | `/competition/hazard_candidates`、定位与跟随状态、`/competition/planning/exploration_complete`、公开场景文件 |
| 关键输出 | `/competition/state`、`/competition/enabled`、`/competition/emergency_stop`、`results/detected_danger.json` |
| 重要接口 | 楼层完成门用"双证据 + 新鲜度"（布尔通道 + 任务状态通道）；返航到达需容差内连续多帧采样 |

### A.3.7 RGB-D 危险源识别、三维定位与空间去重

| 项 | 内容 |
|---|---|
| 模块 | 感知与结果聚合（`competition_perception` + `competition_loop`） |
| 关键源文件 | `perception_node`、`perception_core.py`、`hazard_candidate_router.py`、`result_aggregator.py`、`result_writer.py` |
| 主要职责 | 同步 RGB-D 并做观测准入（零/未来/过期/前代际/重复/乱序全部拒绝）→ HSV 掩膜与形态学 → 轮廓几何分类（红球=危险源；红方块/绿球=干扰源）→ 质心窗口深度中值 → 针孔反投影为三维点 → 世界变换 → 距离去重 → 每源聚合 → 原子写结果 |
| 关键输入 | `/real_sense/rgb/image_raw`、`/real_sense/depth/image_raw` 及 CameraInfo、位姿（用于世界变换） |
| 关键输出 | `/competition/hazard_candidates`、`/competition/perception/status`、`results/detected_danger.json` |
| 重要接口 | 参数名（launch/节点私参）：`hazard_allowed_labels`（默认 `["red_sphere"]`，即干扰源永不入结果）、`hazard_candidate_dedup_distance`（节点默认 0.30 m）、`hazard_cluster_enabled` / `hazard_cluster_radius`（默认 true / 1.0 m，每源聚合为一个代表点）、`hazard_source_radius`（0.15 m）与 `hazard_source_ground_z`（0.0 m）用于候选 z 归位 |

### A.3.8 模式 B 增量模块

| 模块 | 关键源文件 | 主要职责 | 关键输入 | 关键输出 |
|---|---|---|---|---|
| 多楼层地图管理 | `floor_map_manager.py` | 每层一张栅格、激活指针切换、世代失效 | `/competition/floor_state`（`-1` 表示过渡） | 激活楼层的栅格 |
| 楼层计划 | `mission_plan.py` | 由公开场景的电梯服务楼层集合升序确定访问顺序（无随机性） | `team_scene_info.json` | 楼层计划（确定性） |
| 多楼层子 FSM | `floor_mission_fsm.py` | `FLOOR_EXPLORATION→ELEVATOR_LOCATE→ELEVATOR_REQUEST→ELEVATOR_TRANSIT→FLOOR_SWITCH→…→ALL_FLOORS_COMPLETE→RETURN_TO_START_FLOOR` | 各模块状态、电梯状态 | 子相位 + 跨层指令 |
| 电梯门口定位 | `elevator_locator.py` | 用官方 `/set_door_state` 开厅门后走有界重观测路线，从栅格差异定位门口袋并生成进/出梯目标 | 栅格、电梯状态、门服务响应 | approach/cabin/exit 目标 |
| 电梯/门槛执行器 | `door_elevator_executor_node`、`elevator_executor_core.py`、`elevator_service_sequence.py`、`elevator_threshold_profile.py` | 以官方服务响应为唯一就绪证据驱动乘梯与门槛通过；期间通过 `execution_mode` 接管运动 | `/competition/elevator/mission` | `/competition/elevator/status`、transit 目标 |
| 全局危险源 registry | `hazard_candidate_router.py`（`set_floor()`） | 切层只改去重作用域，不清空历史结果 | 候选 + 当前楼层 | 跨层不丢失的已确认源 |

---

## A.4 启动方式

### A.4.1 模式 A

```bash
# 终端 1：环境（场景 + Gazebo + 控制器）
cd baseline_single_floor
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
SEED=77 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 \
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 \
GUI=true ENABLE_REALSENSE=1 ENABLE_SENSORS=1 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 ./auto.sh

# 终端 2：任务栈（验证运行使用的冻结 wrapper，逐字节一致）
roslaunch tools/baselines/floor0_video_return_v1/floor0_full_replay.launch max_time:=1800.0
```

生产默认入口（不含验证 wrapper）：

```bash
roslaunch competition_bringup competition_system.launch
```

### A.4.2 模式 B

```bash
# 终端 1：环境（多楼层场景）
cd baseline_full_competition
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
SEED=20260507 FLOOR_COUNT=3 ROOMS_PER_FLOOR=4 \
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 \
GUI=true ENABLE_REALSENSE=1 ENABLE_SENSORS=1 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 ./auto.sh

# 终端 2：真实比赛完整流程
roslaunch competition_bringup competition_pipeline_full.launch \
    max_time:=1800.0 start_floor:=0
```

构建（两模式相同）：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

---

## A.5 参数配置（仅列源码中真实存在的参数）

### A.5.1 场景/环境参数（`auto.sh` 环境变量，由场景生成脚本消费）

| 参数 | 默认值 | 来源 | 说明 |
|---|---|---|---|
| `SEED` | 空（自动随机） | `auto.sh` | 场景随机种子；验证运行固定 `77` |
| `FLOOR_COUNT` | `3` | `auto.sh` | 楼层数；模式 A 验证固定 `1` |
| `ROOMS_PER_FLOOR` | `4` | `auto.sh` | 每层房间数；验证固定 `4` |
| `BUILDING_WIDTH` / `BUILDING_LENGTH` | `20.0` / `36.0` | `auto.sh` | 楼栋尺寸（m） |
| `DANGER_COUNT` / `DISTRACTOR_COUNT` | `3:6` / `4:8` | `auto.sh` | 危险源/干扰源数量（`min:max`） |
| `GUI` / `PAUSED` | `true` / `false` | `auto.sh` | Gazebo GUI 与启动暂停 |
| `ENABLE_GROUND_TRUTH` | `0` | `auto.sh` | 真值话题开关（正式必须 0） |
| `ENABLE_REFEREE_ODOM` | `0` | `auto.sh` | `/Odometry_gazebo` 开关（正式必须 0） |
| `WRITE_GENERATED_TRUTH_COPY` | `1`（验证置 0） | `auto.sh` | 是否在生成目录写真值副本 |
| `ENABLE_SENSORS` / `ENABLE_LIVOX` / `ENABLE_REALSENSE` | `1` | `auto.sh` | 传感器数据总开关与单项开关 |
| `UNITREE_AUTO_CMD_VEL` | `0`（验证置 1） | `auto.sh` | 自动站立并进入 `/cmd_vel` 模式（仅仿真构建生效） |
| `CONTROLLER_FOREGROUND` | `1`（验证置 0） | `auto.sh` | 控制器前台/后台 |
| `ROBOT_X/Y/Z/YAW` | `0.0 / -3.2 / 0.6 / 1.5708` | `auto.sh` | 出生点（写入世界文件与公开信息） |

### A.5.2 任务栈参数

来源：`competition_bringup/config/default.yaml` + `competition_system.launch` 的
launch arg + 各节点私有参数。

| 参数 | 默认值 | 来源文件 | 说明 |
|---|---|---|---|
| `max_time` | 模式 A 600.0（launch 内 `<param>`，无 arg）；模式 B 有 `<arg>` 默认 600.0，wrapper 透传 1800.0 | `competition_system.launch` / `competition_pipeline_full.launch` | 任务时限；验证运行用 1800 s 看门狗 |
| `mission_mode` | `single_floor` | `competition_system.launch` | `single_floor` / `multi_floor` |
| `start_floor` | `0` | `competition_system.launch` | 起始楼层（模式 B） |
| `elevator_executor_enabled` | `false`（模式 B wrapper 强制 `true`） | `competition_system.launch` | 是否启动电梯/门槛执行器 |
| `locate_timeout` / `elevator_request_timeout` / `floor_switch_ready_timeout` / `max_elevator_retries` / `door_service_timeout` | `180.0 / 30.0 / 60.0 / 1 / 45.0` | `competition_pipeline_full.launch` | 模式 B 电梯流程超时与重试 |
| `workspace_root` | `/workspace/SimEnv` | `competition_system.launch`、`default.yaml` | 解析公开场景文件与结果路径的根 |
| `result_path` | `results/detected_danger.json` | 同上 | 结果文件相对路径 |
| `auto_start` / `start_delay` | `true` / `5.0` | `default.yaml` | 检查通过后自动开始任务 |
| `return_arrival_tolerance` / `return_arrival_samples` | `0.25` / `3` | `competition_system.launch` | 返航到达容差（m）与连续采样数 |
| `return_input_timeout` | `1.0` | `competition_system.launch` | 返航输入新鲜度门（s） |
| `cmd_vel_timeout` | `0.5` | `default.yaml` | 原始速度超时（s） |
| `max_linear_x/y` / `max_angular_z` | `0.5` / `0.3` / `1.0` | `default.yaml` | 速度限幅 |
| `mapping.resolution` / `map_width` / `map_height` | `0.1` / `400` / `400` | `default.yaml`、`mapping.yaml` | 栅格分辨率与窗口 |
| `mapping.min_height` | `-0.15` | `default.yaml` | 高度窗（排除地面回波） |
| `planning.frontier_min_size` | `5` | `default.yaml` | Frontier 簇最小格数 |
| `planning.inflation_radius` | `0.25` | `default.yaml` | 障碍膨胀半径（m） |
| `planning.replan_interval` | `2.0` | `default.yaml` | 重规划周期（s） |
| `planning.goal_tolerance` | `0.5` | `default.yaml` | 目标到达容差（m） |
| `planning.exploration_complete_confirmations` | `3` | `default.yaml` | 完成信号去抖次数 |
| `planning.visual_scan_*` | 见 `default.yaml` | `default.yaml` | 视觉覆盖步长/驻留/超时/重试/位置去重半径/路线间距等 |
| `localization.imu_frame` / `base_frame` | `livox_imu_link` / `base` | `default.yaml` | 坐标系契约 |
| `localization.base_to_imu_translation/rotation_rpy` | `[0.223404, -0.023290, 0.118985]` / `[0.0, 0.785, 0.0]` | `default.yaml` | 固定外参（由 URDF 链推导） |
| `perception.red_hue_min/max` / `min_confidence` | `0` / `10` / `0.5` | `default.yaml` | HSV 红色范围与最小置信度 |
| 验证 wrapper 参数 | `planning_room_mission_enabled=true`、`planning_room_scan_zones=[[1.1,9.5,7.9,21.9],[-9.5,-1.1,7.9,21.9],[1.1,9.5,21.9,35.9],[-9.5,-1.1,21.9,35.9]]`、`planning_visual_scan_route_spacing=4.0`、`correction_mode=planar_relative` | `tools/baselines/floor0_video_return_v1/floor0_full_replay.launch` | 验证运行使用的冻结选择项 |

---

## A.6 输入输出文件

| 文件 | 生成方 | 用途 | 可否作为算法输入 |
|---|---|---|---|
| `generated_building/team_scene_info.json` | 场景生成脚本 | 机器人起点、公开门/电梯 id、允许接口、结果文件路径 | **是**（唯一允许的公开场景文件） |
| `generated_building/competition_scene.world` | 场景生成脚本 | Gazebo 世界文件（由 `auto.sh` 加载） | 否（不作为算法输入） |
| `generated_building/layout_metadata.json` | 场景生成脚本 | 楼栋布局/房间/门/电梯/目标点元数据 | **否（禁止）** |
| `generated_building/building_config.json`、`scene_manifest.json` | 场景生成脚本 | 环境内部配置与本次场景 manifest | **否（禁止）** |
| `generated_building/door_config.yaml`、`elevator_config.yaml` | 场景生成脚本 | 门/电梯由环境服务读取 | **否（禁止）** |
| `results/danger_truth.json` | 场景生成脚本 | 危险源真值（裁判用） | **否（禁止）**，仅用于离线评估 |
| `results/detected_danger.json` | 参赛系统（`result_writer.py`，原子写入） | 最终输出 | 输出文件 |
| `results/evaluation_result.json` | `evaluate_danger.py` | 离线评估结果 | 评估产物 |
| 地图 `/competition/map` | `mapping_node` | 2D 占据栅格（`nav_msgs/OccupancyGrid`） | 运行时话题 |
| 路径 `/competition/path` | `planning_node` | 带朝向路径 | 运行时话题 |

`results/detected_danger.json` 格式（严格官方 schema）：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

说明：楼层信息仅用于内部去重与 registry，**不写入**输出文件（官方 schema 无该字段）；
输出前会做空间去重与每源聚合，重复观测不会产生重复条目。

---

## A.7 日志记录方式

| 类别 | 位置/方式 | 内容 |
|---|---|---|
| ROS 节点日志 | `ROS_HOME` 下按运行建立的目录（本仓库默认 `<WS>/artifacts/ros/log/<uuid>/`） | 每节点的 `.log` 与 stdout，含 FSM 相位变化、规划决策、感知统计、异常 |
| Gazebo / launch 日志 | `<WS>/logs/competition_gazebo.log`（由 `auto.sh` 重定向） | 世界加载、模型与控制器 spawn、插件报错 |
| 控制器日志 | `<WS>/logs/junior_ctrl.log` | 站立/`cmd_vel` 模式切换、RL 策略加载、`RL_HANDOFF_READY` 就绪标志 |
| 门/电梯服务日志 | `<WS>/logs/building_control.log` | 门开闭与电梯服务调用记录 |
| 状态事件 | ROS 话题 `/competition/state`、各 `ModuleStatus` | 任务相位与模块健康（可 rosbag 记录） |
| 结果文件 | `<WS>/results/detected_danger.json` | 最终危险源位置 |
| 评估产物 | `<WS>/results/evaluation_result.json` | 与真值的离线匹配结果（评估用） |

约定：判断一次运行是否成功，应只看**本次运行**的 ROS 节点日志与结果文件；
`artifacts/`、`reports/` 中的历史记录属于开发期材料，不作为运行判据。

---

## A.8 依赖软件及安装方式

| 类别 | 具体内容 | 安装方式 |
|---|---|---|
| OS | Ubuntu 20.04 | 基础镜像或本机系统 |
| ROS | ROS Noetic，`ros-noetic-desktop-full`（含 Gazebo Classic 11.15.1） | `apt install ros-noetic-desktop-full`（`docker/Dockerfile`） |
| 仿真 | Gazebo Classic 11 | 随 `desktop-full`；也可单独 `ros-noetic-gazebo-ros-pkgs` |
| 构建工具 | catkin（`catkin_make`）、`cmake ≥ 3.16`、`ninja-build`、g++ 9 | APT |
| 语言 | Python 3.8、C++14+ | 系统自带 / APT |
| LibTorch / CUDA | LibTorch 2.7.1+cu128（C++）到 `/opt/libtorch`；CUDA 12.8 工具链 | `docker/Dockerfile` 内 `wget` 下载 `libtorch-cxx11-abi-shared-with-deps-2.7.1+cu128.zip` 并解压到 `/opt`；`LIBTORCH_ROOT` 可覆盖 |
| 系统库 | `build-essential`、`pkg-config`、`libeigen3-dev`、`libarmadillo-dev`、`liblcm-dev`、`libyaml-cpp-dev`、`qtbase5-dev`、`libapr1-dev`、`mesa-utils`、`x11-apps` | APT |
| ROS 依赖 | 由各 `package.xml` 声明：`roscpp rospy std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs tf tf2_ros tf2_geometry_msgs cv_bridge image_transport message_filters pcl_ros gazebo_msgs gazebo_ros controller_manager joint_state_controller robot_state_publisher rviz message_generation message_runtime` | `rosdep install --from-paths src --ignore-src -y --rosdistro=noetic`（`docker/Dockerfile` 的实际做法） |
| Python 包 | `python3-numpy`、`python3-opencv`、`python3-yaml`、`python3-pip`、`python3-catkin-tools`、`python3-rosdep`、`python3-vcstool` | APT |
| 第三方库（源码内） | `pcl`、`Eigen`、`Boost`、`APR`、`rapidjson`（随 livox 驱动）、`quadProg++`（随 unitree_guide）、`pybind11`（随 unitree_legged_sdk，仅真机 Python 绑定使用，默认 `PYTHON_BUILD=OFF` 不编译） | 随源码或 APT |
| 子模块（已随包实体化，**无需** `git submodule update`） | `src/fast_lio`（模式 A：`bf5f051`；模式 B：`e6aeba1`）、`src/fast_lio/include/ikd-Tree`（`e2e3f4e`）、`src/livox_ros_driver`（模式 A：`24cf049`；模式 B：`3d240d5`） | 已包含在提交包内；如需对齐上游可自行 `git clone` |
| 可选 | Livox-SDK（仅真机 Livox 驱动）、NVIDIA 驱动与 `nvidia-container-toolkit`（仅 GUI/GPU 加速） | 见 `README_运行与复现.md` 1.2 / 4.3 节 |

容器方式（推荐）：

```bash
cd <WS>
docker compose build simenv
docker compose run --rm simenv bash
# 容器内工作目录为 /workspace/SimEnv，entrypoint 自动 source ROS 与 devel/setup.bash
```
