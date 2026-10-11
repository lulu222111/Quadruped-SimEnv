# DG-202602 核心代码提交说明

- 参赛单位：哈尔滨工业大学（深圳）
- 团队名称：危境智巡——面向高危未知场景的四足机器人自主探测导航系统
- 提交内容：核心代码（两个独立基线 + 复现说明 + 技术报告素材）

> 阅读顺序建议：本文件（了解提交包里有什么、验证到什么程度）→
> `README_运行与复现.md`（从解压开始怎么跑起来）→ `docs/附录A_素材.md` /
> `docs/附录B_素材.md`（技术报告写作素材）→ `docs/BASELINE_PROVENANCE.md`
> （来源与可追溯性）。

---

## 0. 本提交包包含两个相互独立的基线

本提交包**不是**一次 git merge，也**不是**把两套源码合成一个运行树。
包内是两套**完整、自包含、可分别构建与启动**的基线源码，用于表达同一套比赛
系统的两种提交口径：

| 目录 | 名称 | 一句话定位 | 运行验证状态 |
|---|---|---|---|
| `baseline_single_floor/` | **模式 A：单层完整闭环基线** | 已经过真实 Gazebo 端到端闭环验证的单层版本，是核心算法演示入口 | **runtime validated** |
| `baseline_full_competition/` | **模式 B：真实比赛完整流程基线** | 覆盖多楼层 / 电梯 / 跨层任务调度的完整比赛流程代码 | **code complete / build+unit+static validated；完整多楼层端到端 Gazebo 尚未验证** |

两个目录各自携带完整仓库内容（含 `fast_lio` / `livox_ros_driver` 子模块源码），
**互不依赖**：A 不引用 B 的任何文件，B 也不引用 A。评审可以只解压其中一个目录
单独构建与启动。

一体化说明：两套基线共享同一个仿真环境的模块划分（定位 / 建图 / 规划 / 感知 /
控制 / 任务编排 / 启动装配），差异集中在**任务编排层与执行器层**：

- 模式 A 有：单层探索 FSM、房间任务/可视化扫描、返航（RETURNING）；
- 模式 B 在 A 的能力之上增加：多楼层地图管理（`floor_map_manager.py`）、
  多楼层子 FSM（`floor_mission_fsm.py`）、楼层计划（`mission_plan.py`）、
  电梯定位（`elevator_locator.py`）与电梯/门槛执行器
  （`door_elevator_executor_node`、`elevator_executor_core.py`、
  `elevator_service_sequence.py`、`elevator_threshold_profile.py`）。

---

## 1. 模式 A：已验证单层完整闭环

**这是"可直接用于核心算法演示和单层闭环复现的已验证版本"。**

### 1.1 验证结论（3 次独立完整闭环）

在同一冻结源码状态下完成的 3 次端到端 Gazebo 运行（另 1 次因返程楔住作废，
已在来源材料中如实记录）：

| RUN | 结果 | 闭环耗时 | 返航落点误差 |
|---|---|---|---|
| `20260914-101516` | PASS | 828.80 s | 0.215 m |
| `20260914-121632` | PASS | 854.10 s | 0.093 m |
| `20260914-153840` | PASS | **783.50 s** | **0.090 m** |

3 次共同指标：

```text
HAZARDS                        = 6/6（唯一真值源匹配，1.0 m 口径）
FALSE_ALARMS                   = 0
MAX_TIME_REACHED               = false
SKIPS / UNREACHABLE            = 0 / 0
GROUND_TRUTH_FEEDBACK_COUNT    = 0
MANUAL_INTERVENTION_COUNT      = 0
任务状态序列                    = WAITING → RUNNING → CURRENT_FLOOR_COMPLETE
                                 → RETURNING → ARRIVED_START → FINISHED
```

模式 A 覆盖的完整自主闭环：**自主定位 → 在线建图 → Frontier 自主探索 →
可视化覆盖扫描 → 危险源 RGB-D 识别与三维定位 → 路径规划与运动控制 →
自主返航 → 结果输出**，全流程无人工干预、不接入任何真值通道。

### 1.2 核心算法演示入口

| 功能 | 源码位置（模式 A） | 启动方式 |
|---|---|---|
| 自主定位 | `src/competition_localization/scripts/localization_node`、`leg_odometry.py`、`filter_livox.py` | 由 `competition_system.launch` 或 `localization_corrected.launch` 启动 |
| 在线建图 | `src/competition_mapping/scripts/mapping_node`、`src/competition_mapping/src/competition_mapping/occupancy_grid_core.py` | `roslaunch competition_mapping mapping.launch` |
| Frontier 自主探索 | `src/competition_planning/scripts/planning_node`、`frontier.py` | `roslaunch competition_bringup competition_system.launch` |
| 视觉覆盖（主动扫描） | `src/competition_planning/scripts/visual_coverage.py`、`room_access.py` | 同上（参数 `planning_visual_scan_*`） |
| 路径规划 | `src/competition_planning/scripts/planning_core.py`、`path_planner.py` | 同上 |
| 运动控制 | `src/competition_control/scripts/path_follower_node`、`cmd_vel_guard_node` | 由 `competition_system.launch` 启动 |
| 危险源识别（RGB-D） | `src/competition_perception/scripts/perception_node`、`perception_core.py` | 同上 |
| 危险源三维定位 | `src/competition_perception/scripts/perception_core.py`（`project_to_3d` / `robust_depth`） | 同上 |
| 空间去重 | `src/competition_loop/src/hazard_candidate_router.py`、`result_aggregator.py` | 同上 |
| 任务状态机 | `src/competition_loop/src/competition_state_machine.py`、`floor_lifecycle.py` | 同上 |
| 自主返航 | `src/competition_planning/scripts/planning_node`（RETURNING 分支）+ `competition_loop`（到达判定） | 同上 |
| 结果输出 | `src/competition_loop/src/result_writer.py` → `results/detected_danger.json` | 同上 |

> 演示建议：先按 `README_运行与复现.md` 第 3 节启动模式 A，Gazebo GUI 中即可
> 观察机器狗自主探索、原地旋转扫描、接近红色球体危险源并返航的全过程；
> 结果文件与日志位置见该文件第 5 节。

---

## 2. 模式 B：真实比赛完整流程

**已实现、已构建通过、已通过单元与静态检查；完整多楼层端到端 Gazebo 尚未验证。**

### 2.1 状态标注（严格区分"已实现"与"已验证"）

```text
COMPETITION_CODE_COMPLETE              = true
BUILD                                  = PASS
UNIT_TESTS                             = 380/380 PASS（0 failure / 0 error）
STATIC / INTERFACE AUDIT               = PASS（9 条多楼层接口双向闭环）
END_TO_END_MULTI_FLOOR_GAZEBO          = NOT YET VALIDATED
```

请勿把本节理解为"多楼层比赛流程已经跑通"：多楼层端到端仿真运行、电梯行为
参数标定、跨楼层返航精度均属于**待验证项**。

### 2.2 覆盖的真实比赛流程

- 定位（FAST-LIO 适配 + 楼层复位信号）
- 多楼层建图（每层独立栅格、楼层切换与地图生命周期）
- Frontier 自主探索 + 有界主动视觉覆盖
- 路径规划（含电梯 approach / cabin / exit 的 transit 目标）与单一运动源控制
- RGB-D 危险源识别与三维定位
- 全局危险源 registry（跨楼层不丢失、按楼层去重）
- 电梯契约与执行器（官方 `/set_door_state`、`/call_elevator` 证据链驱动）
- 楼层切换（sentinel → 过渡 → 定位/规划复位 → 激活新楼层）
- return-to-start-floor（跨层返航）与 return-home（回到起点附近）
- 结果序列化（`detected_danger.json`，严格官方 schema、原子写入）
- 完整任务 FSM

### 2.3 模式 B 入口

```bash
roslaunch competition_bringup competition_pipeline_full.launch \
    max_time:=1800.0 \
    start_floor:=0
```

该入口为 wrapper，会强制：

```text
mission_mode                = multi_floor
elevator_executor_enabled   = true
```

任务流程（代码层相位）：

```text
WAITING → RUNNING → FLOOR_EXPLORATION → ELEVATOR_LOCATE → ELEVATOR_REQUEST
        → ELEVATOR_TRANSIT → FLOOR_SWITCH → （下一层…） → ALL_FLOORS_COMPLETE
        → RETURN_TO_START_FLOOR → RETURN_HOME（RETURNING） → ARRIVED_START → FINISHED
```

| 模式 B 增量功能 | 源码位置 |
|---|---|
| 多楼层地图管理 | `src/competition_mapping/src/competition_mapping/floor_map_manager.py` |
| 楼层计划（公开信息排序） | `src/competition_loop/src/mission_plan.py` |
| 多楼层子 FSM / 跨层调度 | `src/competition_loop/src/floor_mission_fsm.py` |
| 电梯门口定位 | `src/competition_planning/scripts/elevator_locator.py` |
| 电梯/门槛执行器 | `src/competition_control/scripts/door_elevator_executor_node`、`elevator_executor_core.py`、`elevator_service_sequence.py`、`elevator_threshold_profile.py` |
| 全局危险源 registry | `src/competition_loop/src/hazard_candidate_router.py`（`set_floor()`，跨层不丢历史） |
| return-to-start-floor / return-home | `floor_mission_fsm.py` + `competition_state_machine.py` |
| 结果序列化 | `src/competition_loop/src/result_writer.py` |

---

## 3. 目录结构

```text
DG-202602-core-code-submission-v1/
├── README_提交说明.md              # 本文件
├── README_运行与复现.md            # 从解压开始的完整运行与复现说明
├── baseline_single_floor/          # 模式 A：已验证单层完整闭环（完整仓库副本）
│   ├── src/                        # 仿真环境 + 参赛系统各 ROS 包（含子模块源码）
│   ├── tools/baselines/            # 基线复现入口（含本次验证运行所用 launch 与 harness）
│   ├── auto.sh                     # 环境一键启动（场景生成 + Gazebo + 控制器）
│   ├── docker/ compose.yaml        # 容器化复现
│   └── docs/ reports/ scripts/     # 环境文档、历史分析报告、辅助脚本
├── baseline_full_competition/      # 模式 B：真实比赛完整流程（完整仓库副本）
│   ├── src/                        # 同上，另含多楼层 / 电梯相关模块
│   ├── README.md / README_COMPETITION_CN.md
│   └── reports/                    # 代码完整性与可读性审计报告
├── docs/
│   ├── 附录A_素材.md               # 技术报告素材：软件系统实现与代码组织
│   ├── 附录B_素材.md               # 技术报告素材：系统运行及复现说明
│   └── BASELINE_PROVENANCE.md      # 两个基线的来源、提交号、冻结指纹与验证记录
└── manifest/
    ├── FILE_MANIFEST.txt           # 包内文件清单
    ├── SHA256SUMS.txt              # 包内文件 SHA256
    └── PACKAGE_AUDIT.md            # 打包审计报告（含逐项验收结论）
```

---

## 4. 输入与输出

### 4.1 主要输入（正式链路）

| 类别 | 内容 |
|---|---|
| 传感器 | Livox Mid-360 激光点云、机体 IMU、Livox IMU、RealSense RGB 与深度图（含各自 `CameraInfo`） |
| 关节/状态 | A1 `JointState`、控制器状态（用于腿式里程计与运动使能） |
| 公开场景信息 | `generated_building/team_scene_info.json`（机器人起点、公开门/电梯 id、允许接口、结果路径） |
| 运行参数 | launch arg 与 `competition_bringup/config/default.yaml` |
| 环境服务 | `/set_door_state`、`/call_elevator`（门与电梯的官方服务） |

### 4.2 主要输出

| 类别 | 内容 |
|---|---|
| 地图 | `/competition/map`（2D 占据栅格；模式 B 为每层独立栅格） |
| 路径 | `/competition/path`（唯一路径来源，保证单一运动源） |
| 指令 | `/competition/raw_cmd_vel` → `/cmd_vel`（经安全网关限幅后） |
| 任务状态 | `/competition/state`、各模块 `ModuleStatus` |
| 危险源候选 | `/competition/hazard_candidates` |
| 最终结果 | `results/detected_danger.json` |
| 日志 | Gazebo/控制器日志、ROS 节点日志、运行脚本日志（位置见 `README_运行与复现.md` 第 5 节） |

### 4.3 禁止输入（合规声明）

```text
GROUND_TRUTH_FEEDBACK = forbidden for production control
```

正式控制链路**不得**使用：`/Odometry_gazebo`、`/ground_truth/*`、
`/gazebo/model_states`、`/gazebo/link_states`、`/gazebo/get_model_state`，
以及 `layout_metadata.json`、`building_config.json`、`scene_manifest.json`、
`danger_truth.json`、`door_config.yaml`、`elevator_config.yaml` 等非公开文件。
Gazebo 真值仅用于**离线评估**算法输出（评估口径见环境文档 `docs/evaluation.md`），
**不是**本系统的生产输入。

---

## 5. 提交包审计摘要

```text
SOURCE_CODE_INCLUDED                     = true
SINGLE_FLOOR_BASELINE_INCLUDED           = true
FULL_COMPETITION_BASELINE_INCLUDED       = true
AUTONOMOUS_EXPLORATION_INCLUDED          = true
HAZARD_DETECTION_LOCALIZATION_INCLUDED   = true

CHINESE_COMMENTS                         = PASS
RUNTIME_ENVIRONMENT_DOCUMENTED           = true
TWO_COMPLETE_STARTUP_GUIDES              = true

TEST_CODE_INCLUDED                       = false
LOGIC_CHANGE_COUNT                       = 0（生产算法逻辑零改动）
PARAMETER_CHANGE_COUNT                   = 0
INTERFACE_CHANGE_COUNT                   = 0

SIMULATION_RUN                           = NOT RUN（本打包阶段未运行 Gazebo）
```

说明与例外（全部已在 `manifest/PACKAGE_AUDIT.md` 中逐条列出）：

1. 包内两个基线的**算法逻辑、参数、ROS 话题/服务、消息 schema、launch 运行语义、
   FSM、规划/控制/感知/定位行为**均未修改；仅补充中文注释与说明文档。
2. 按提交要求**删除了测试代码**（`test/` 目录、rostest 用例、单元测试数据与
   测试脚本）。由于部分构建文件声明了这些测试目标，删除测试源文件后，
   对应的**测试目标声明被一并停用**（12–13 个 `CMakeLists.txt`，仅涉及
   `catkin_add_nosetests` / `add_rostest` / `catkin_add_gtest` 与 `install(DIRECTORY)`
   中的 `test` 项），否则 `catkin_make` 会因找不到测试文件而直接报错。
   该改动**不涉及任何运行时目标**。
3. 模式 B 的子模块（`fast_lio` / `livox_ros_driver`）在来源基线中记录为
   git 链接且未检出；为满足"解压即可构建、不要求评审执行 `git submodule update`"，
   包内已**实体化其源码**（版本与基线记录一致）。
   该子模块在**裸状态下存在预存在的构建依赖缺陷**（缺少消息生成依赖声明），
   基线自带修复补丁（`scripts/patches/`）。详见 `README_运行与复现.md`
   第 4.3 节与 `manifest/PACKAGE_AUDIT.md` 中的 `PREEXISTING_BASELINE_ISSUE`。

---

## 6. 复现入口速查

```bash
# 0) 解压后进入任意一个基线目录
cd baseline_single_floor          # 或 baseline_full_competition

# 1) 构建（容器内）
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash

# 2) 模式 A：环境 + 任务
SEED=77 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 GUI=true ENABLE_REALSENSE=1 \
  UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 \
  ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 ./auto.sh
roslaunch tools/baselines/floor0_video_return_v1/floor0_full_replay.launch max_time:=1800.0

# 3) 模式 B：环境 + 任务
roslaunch competition_bringup competition_pipeline_full.launch \
    max_time:=1800.0 start_floor:=0
```

完整步骤、环境矩阵、结果判据、停止与重跑方式见 `README_运行与复现.md`。
