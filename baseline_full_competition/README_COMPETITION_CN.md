# DG-202602 四足机器人危险源自主搜索系统

> 本文档面向第一次拿到本压缩包、需要理解并复现比赛流程的评审专家。
> 阅读顺序建议：第 1~3 节了解系统全貌，第 4~8 节按需复现。

---

## 1. 项目简介

本系统面向 DG-202602 比赛场景，控制 Unitree A1 四足机器人在未知室内楼栋中自主完成：

- 定位与建图：基于 FAST-LIO 的激光惯性定位 + 逐楼层独立 2D 占据栅格建图；
- 自主探索：基于 Frontier 的未知区域探索，并在关键位置原地旋转扫描；
- 危险源识别与定位：基于 RealSense RGB-D 的颜色/几何检测与三维反投影；
- 危险源空间去重与全局注册：跨帧、跨楼层统一确认，避免重复上报；
- 多楼层任务调度：按公开场景信息生成楼层访问顺序并驱动状态机；
- 电梯自主通行：通过建筑官方服务开厅门、乘梯、跨楼层切换；
- 自主返航与结果输出：回到出发楼层与起点附近，输出 `detected_danger.json`。

系统采用模块化 ROS 架构：定位、建图、规划、感知、控制、任务编排各自独立成包，
通过冻结的 ROS 话题/服务契约通信，任一模块失效都不会让其他模块静默输出错误运动指令。

---

## 2. 代码结构

```text
src/
├── competition_localization/   FAST-LIO 适配、world->base TF、跨楼层复位
├── competition_mapping/        逐楼层独立占据栅格建图与楼层过渡
├── competition_planning/       Frontier 探索、路径规划、电梯门口定位
├── competition_perception/     RGB-D 危险源检测与候选发布
├── competition_control/        运动安全网关、路径跟随、电梯/门槛执行器
├── competition_loop/           顶层任务 FSM、多楼层子 FSM、结果输出
├── competition_bringup/        统一 launch 装配与参数透传
└── competition_interfaces/     competition 内部 msg 契约
```

| 目录 | 职责（一句话） |
|---|---|
| `competition_localization` | 把 FAST-LIO 输出适配成比赛定位接口，并维护 world -> base 一致性 |
| `competition_mapping` | 维护每层独立栅格，负责楼层切换与地图生命周期 |
| `competition_planning` | 决定"下一步去哪、怎么走"，并合法发现电梯厅门口 |
| `competition_perception` | 从 RGB-D 产生危险源观测候选 |
| `competition_control` | 唯一速度输出通道、路径跟随与电梯执行器运动仲裁 |
| `competition_loop` | 比赛状态机、多楼层流程编排、危险源去重与结果落盘 |
| `competition_bringup` | 把上述模块按 MODE A / MODE B 组装起来 |
| `competition_interfaces` | 定义模块间消息（含 `ElevatorMission` / `ElevatorStatus`） |

---

## 3. 核心功能

### 3.1 自主定位与建图

定位节点以 FAST-LIO 里程计为输入，补偿固定的 `base -> livox_imu_link` 外参，
输出 `world -> base` 位姿，并作为该 TF 链的唯一动态发布者。
世界坐标系对齐基准取公开场景文件中的机器人起点，且只在首个有效位姿处锁存一次，
避免启动稳定期间机器人移动引入对齐误差。系统不使用 Gazebo 真值做在线修正。

建图节点融合定位与 Livox 点云生成 2D 占据栅格。
每层楼持有独立栅格，任一时刻只有一层处于激活状态；切换楼层只切换激活指针，
不会把不同楼层的点云混进同一张地图。

### 3.2 Frontier 自主探索

规划节点从占据栅格提取"已知自由空间与未知空间的交界"作为 Frontier，
对相邻 Frontier 聚类后按可达性与代价选点，生成到该点的全局路径。
当某一位置的视觉覆盖不足时，规划节点会发布仅含旋转的路径做原地扫描。
探索完成的判定要求"连续多次无可用 Frontier 且扫描已结束"，避免瞬时假完成。

### 3.3 危险源识别与三维定位

感知节点同步订阅 RGB 图、深度图与各自的 `CameraInfo`，
在 HSV 空间提取红/绿掩膜，做形态学去噪后提取轮廓，
依据面积、圆度与长宽比分类为红色球（危险源）、红色方块与绿色球（干扰源）。
随后在轮廓质心附近取中值深度，用深度相机内参反投影为三维点。

### 3.4 危险源空间去重与全局注册

感知只负责"产生观测候选"，是否可信、是否重复由 `competition_loop` 侧判定：
候选先做坐标系与时间戳校验，再按观测时间戳变换到 `world` 坐标系，
最后与当前楼层已确认的位置做距离去重。

危险源注册表是全局的，切换到新楼层时只改变去重作用域，**不清空**已确认的历史结果；
否则已完成楼层的战果会在最终结果中丢失。

### 3.5 路径规划与运动控制

规划节点在障碍膨胀后的栅格上做路径搜索，并输出带朝向的路径到 `/competition/path`。
路径跟随器在锁外完成控制量计算、提交前校验状态版本未更新，避免用过期快照下发指令。

所有速度指令经由 `cmd_vel_guard` 网关校验后才发布到 `/cmd_vel`。
系统遵循单一 motion owner 原则：任何时刻只有一个控制器真正控制机器人运动。

### 3.6 多楼层任务管理

顶层状态机管理 `WAITING / RUNNING / RETURNING / FINISHED / ERROR`，
其中多楼层流程位于 `RUNNING` 内部：
逐层探索 → 电梯定位 → 电梯请求 → 乘梯 → 楼层切换 → 下一层 → 全部完成 → 返航。

楼层顺序由公开场景信息中的电梯服务楼层集合升序确定，不含随机性，同一输入必得同一计划。

### 3.7 电梯自主通行

公开信息只给出电梯与厅门的 **id**，不给出位置。因此电梯位置通过合法可观测链路获取：
先用官方 `/set_door_state` 打开公开厅门，再让机器人走一条有界重观测路线，
从开门前后的栅格差异中定位门口袋，并据此生成进梯/出梯目标。

"就绪"类阶段必须由正式证据驱动：轿厢到位以 `/call_elevator` 返回的楼层为准，
门开启以 `/set_door_state` 返回的状态为准，不做时间推断、不读仿真真值。
电梯执行期间路径跟随器被置为 hold，由执行器接管受约束的局部运动。

### 3.8 自主返航与结果输出

全部楼层探索完成后，若当前不在起始楼层，系统会先乘电梯返回起始楼层，
再进入返航流程规划回到出发位置附近的路径。
到达判定要求定位有效、里程计新鲜、路径跟随成功，且在起点容差内连续采样若干次。

最终结果写入比赛要求的 `detected_danger.json`，格式严格遵循官方 schema，
采用临时文件 + 原子替换方式写入。

---

## 4. 运行环境

以下信息依据本仓库实际内容整理（`docker/Dockerfile`、`compose.yaml`、
`docs/quick-start.md`、各 `package.xml`）。

**必须依赖**

| 项目 | 说明 |
|---|---|
| 操作系统 | Ubuntu 20.04（`docker/Dockerfile` 基于 `nvidia/cuda:12.8.0-devel-ubuntu20.04`；`docs/quick-start.md` 写明 Ubuntu 20.04 或兼容环境） |
| ROS | ROS Noetic（`ros-noetic-desktop-full`，含 Gazebo Classic） |
| 仿真 | Gazebo Classic |
| 构建系统 | catkin（`catkin_make`；镜像内亦提供 `catkin_tools`） |
| 主要语言 | Python 3 与 C++ |

主要 ROS 依赖（取自各 `package.xml`）：

```text
rospy, roscpp, nav_msgs, sensor_msgs, geometry_msgs, std_msgs, std_srvs,
tf2_ros, tf2_geometry_msgs, cv_bridge, message_filters,
competition_interfaces, fast_lio,
python3-numpy, python3-scipy
```

**可选依赖**

- `python3-opencv`：感知节点（颜色掩膜、形态学、轮廓提取）需要。
- `python3-numpy`：感知与 `building_obstacles` 评估脚本需要。
- `python3-yaml`：场景生成与评估脚本需要。

**容器与 GPU**

- Docker 与 Docker Compose：仓库提供 `compose.yaml` 与 `docker/Dockerfile`，
  并以 Docker 作为推荐的隔离复现方式。
- CUDA 12.8 与 LibTorch（cu128）：镜像内安装，用于 Unitree A1 的 `junior_ctrl` 控制器。
- `compose.yaml` 使用 `network_mode: host` 与 `gpus: all`。
  仿真整套链路需要可用的 NVIDIA GPU 驱动与 `nvidia-container-toolkit`。

**开发机配置（非运行必需）**

开发过程中使用过较高规格的 GPU 与 X11 转发用于可视化。
代码本身没有对特定 GPU 型号或 RTX 5090 的硬性要求；
无 GUI 场景可参考 `docs/quick-start.md` 中的 `GUI=false` 用法。

---

## 5. 构建方法

### 前置：初始化子模块

仓库含两个 git submodule，构建脚本会先行检查：

```bash
git submodule update --init --recursive
# 或克隆时一步到位：
# git clone --recurse-submodules <repo-url>
```

缺失时 `scripts/build-workspace.sh` 会直接报错退出。

### 方式一：Docker（推荐）

```bash
cd <workspace>
bash scripts/setup-user-container.sh      # 或 cp .env.example .env 后按需修改
docker compose build simenv
docker compose run --rm simenv bash
```

容器内工作目录为 `/workspace/SimEnv`，`entrypoint` 会自动 `source`
`/opt/ros/noetic/setup.bash` 与工作区 `devel/setup.bash`（若已构建）。
首次构建需要下载 ROS 依赖与 LibTorch，耗时较长。

### 方式二：本机 ROS Noetic

在已安装 ROS Noetic 与 `ros-noetic-desktop-full` 的机器上：

```bash
cd <workspace>
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

`catkin_make` 前需确保 `src/CMakeLists.txt` 是指向 ROS 顶层 `toplevel.cmake` 的符号链接
（本仓库中已如此配置）。`scripts/build-workspace.sh` 封装了同样的流程并把日志写入
`artifacts/logs/`。

> 说明：本仓库未提供完整的裸机环境依赖安装清单，`docker/Dockerfile` 中的
> `rosdep install` 步骤是当前最权威的依赖解析方式。若裸机安装遇到依赖缺失，
> 建议优先使用 Docker 方式复现。

---

## 6. 两种启动模式

比赛系统提供两种启动模式，二者的差异全部由 launch arg 控制，MODE A 的行为未被改写。

### MODE A：单楼层兼容模式

```bash
roslaunch competition_bringup competition_system.launch
```

默认参数：

```text
mission_mode = single_floor
elevator_executor_enabled = false
```

MODE A 保留了已经验证过的单楼层闭环能力（自主探索 → 危险源识别 → 自主返航 → 结果输出），
不启动电梯执行器，也不进入跨楼层流程。
单楼层模式下，状态机会跳过不需要的跨楼层阶段。

### MODE B：真实比赛完整流程

```bash
roslaunch competition_bringup competition_pipeline_full.launch \
    max_time:=1800.0 \
    start_floor:=0
```

该 launch 是一个 wrapper，会强制：

```text
mission_mode = multi_floor
elevator_executor_enabled = true
```

MODE B 覆盖真实比赛完整流程，包括多楼层探索、电梯通行、跨楼层返航。

> **重要说明（请勿混淆）**
>
> MODE B 当前状态为：**代码完成 / 构建通过 / 单元与静态测试通过 /
> 端到端 Gazebo 仿真尚未验证**。
>
> 也就是说，"代码已经完成"不等于"完整多楼层比赛流程已经通过仿真验证"。
> 多楼层端到端仿真运行、电梯行为参数标定、跨楼层返航精度等均属于待验证项。

### 任务流程（纯文本）

```text
INITIALIZE / WAITING
        ↓
      RUNNING
        ↓
FLOOR_EXPLORATION
        ↓
 ELEVATOR_LOCATE
        ↓
ELEVATOR_REQUEST
        ↓
ELEVATOR_TRANSIT
        ↓
  FLOOR_SWITCH
        ↓
     下一层
        ↓
ALL_FLOORS_COMPLETE
        ↓
RETURN_TO_START_FLOOR
        ↓
   RETURN_HOME
        ↓
  ARRIVED_START
        ↓
    FINISHED
```

说明：单楼层模式会跳过不需要的跨楼层阶段（`ELEVATOR_*` / `FLOOR_SWITCH` /
`RETURN_TO_START_FLOOR`），直接由探索完成进入返航。

---

## 7. 核心 ROS 接口

下表只列出跨模块的关键接口。每个接口的名称、发布方与订阅方均在代码中双向闭环。

| 接口 | 发布模块 | 订阅模块 | 用途 |
|---|---|---|---|
| `/competition/elevator/mission` | `competition_loop` | `door_elevator_executor` | 下发一次电梯运输任务（含 `mission_id`、源/目标楼层） |
| `/competition/elevator/status` | `door_elevator_executor` | `competition_loop`, `planning` | 回报电梯阶段（READY_TO_ENTER / RIDING / READY_TO_EXIT 等）与有效性 |
| `/competition/floor_state` | `competition_loop` | `mapping` | 楼层指令：`-1` 表示电梯过渡中，非负值表示激活该楼层 |
| `/competition/mapping/current_floor` | `mapping` | `competition_loop` | 回报当前已激活的楼层，作为楼层切换完成的证据 |
| `/competition/planning/exploration_complete` | `planning` | `competition_loop` | 本层探索完成信号（带去抖与锁存语义） |
| `/competition/planning/elevator_locate` | `competition_loop` | `planning` | 请求规划层开始电梯厅门口定位行为 |
| `/competition/planning/elevator_ready` | `planning` | `competition_loop` | 报告电梯进/出梯目标已定位完成 |
| `/competition/planning/floor_reset` | `competition_loop` | `planning` | 通知规划层复位楼层相关状态并作废旧世代数据 |
| `/competition/localization/floor_reset` | `competition_loop` | `localization` | 通知定位层复位校正/对齐状态，等待新楼层输入重新收敛 |
| `/competition/path` | `planning` | `path_follower`, `door_elevator_executor` | 全局路径/局部 transit 目标的唯一来源，保证单一运动源 |

运动所有权仲裁：`door_elevator_executor` 通过 `/path_follower/execution_mode`
参数把路径跟随器切到 `normal` / `hold` / `precision`，
因此 `/cmd_vel` 在任何时刻都只有一个实际控制者。

---

## 8. 结果输出

最终危险源结果写入比赛要求的 `detected_danger.json`：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

- 只写入经过校验并确认的危险源位置，格式严格遵循官方输出要求；
- 写入前内部会进行空间去重，重复观测不会产生重复条目；
- 多楼层危险源注册表在切换楼层时不会被清空，最终结果包含所有已确认楼层的危险源；
- 楼层信息仅用于内部去重与注册，**不写入输出文件**，因为官方 schema 中没有该字段；
- 结果路径由 launch 参数决定（默认 `results/detected_danger.json`，
  相对 `workspace_root` 解析），请以实际运行配置为准。

结果文件采用临时文件 + 原子替换的方式写入，读取方不会看到写了一半的内容。

---

## 9. 当前验证状态

| 项目 | 状态 |
|---|---|
| 代码完整性 | PASS |
| build | PASS |
| unit/static tests | PASS |
| 单楼层自主探索闭环 | 已验证 |
| 危险源识别与定位 | 已验证 |
| 单楼层自主返航 | 已验证 |
| 多楼层比赛代码 | 已完成（implemented） |
| 多楼层端到端 Gazebo | 待运行验证（not yet validated end-to-end） |

请严格区分 **implemented**（代码已实现）与 **validated**（已通过运行验证）：
上表中"已完成"表示对应代码路径完整、可编译、通过单元与静态测试；
"已验证"表示在仿真中实际跑通过对应闭环。

---

## 10. 复现注意事项

- 建议使用 Docker 隔离环境复现；仓库的 `scripts/setup-user-container.sh` 与
  `compose.yaml` 已按多人共享同一份代码的方式配置。
- 使用 `network_mode: host` 时，`ROS_MASTER_URI` / `GAZEBO_MASTER_URI`
  需避免与宿主机上其他 ROS/Gazebo 实例端口冲突。
- 不要同时启动两个 motion owner：MODE A 下不要手工启动电梯执行器；
  MODE B 下不要绕过 `execution_mode` 仲裁手工发布速度指令。
- 启动前确认已 `source devel/setup.bash`（Docker entrypoint 会自动执行）。
- 首次构建需要下载 ROS 依赖与 LibTorch，耗时可能较长。
- 仿真推进通常慢于真实时间，MODE B 的 1800 s 仿真预算在实机上可能需要数小时。
- 首次运行前请确认 `generated_building/team_scene_info.json` 已由场景生成流程产生。
- 正式控制链路不使用 Gazebo 真值：
  不订阅 `/Odometry_gazebo`、`/ground_truth/*`，不调用 `/gazebo/get_model_state`
  等真值接口，也不读取 `layout_metadata.json`、`building_config.json`、
  `scene_manifest.json`、`danger_truth.json`、`door_config.yaml`、
  `elevator_config.yaml` 等非公开文件。

---

## 11. 代码阅读建议

若只有 5~10 分钟浏览代码，建议按以下顺序：

1. `src/competition_loop/src/competition_state_machine.py` — 顶层阶段与运动使能规则；
2. `src/competition_loop/src/floor_mission_fsm.py` — 多楼层状态推进与切层条件；
3. `src/competition_loop/src/mission_plan.py` — 楼层计划来自哪些公开字段；
4. `src/competition_planning/scripts/planning_node` — 探索/返航/电梯三种规划模式；
5. `src/competition_control/scripts/path_follower_node` — 运动仲裁与单一 motion owner；
6. `src/competition_control/scripts/elevator_service_sequence.py` — 电梯官方服务证据链。

每个文件的文件头均包含该模块的中文职责说明与数据流描述。
