# 附录B 系统运行及复现说明素材

> 本文件为技术报告"系统运行与复现"章节的事实素材，按"全新机器/解压包"起点
> 逐条给出可执行步骤。命令均取自本提交包内源码与脚本，未凭记忆补写。
> 两种模式（A/B）分别给全；两者共用同一仿真环境与容器。

---

## B.1 环境准备（从全新机器或解压包开始）

### B.1.0 前提

- 机器需能运行 Docker（推荐），或已装 ROS Noetic 的 Ubuntu 20.04；
- 解压提交包后得到 `DG-202602-core-code-submission-v1/`；
- 选择要复现的基线目录：`baseline_single_floor/`（模式 A）或
  `baseline_full_competition/`（模式 B）。下文记作 `<WS>`。

```bash
unzip 哈尔滨工业大学（深圳）-黄誉铭-危境智巡--面向高危未知场景的四足机器人自主探测导航系统-核心代码.zip
cd DG-202602-core-code-submission-v1/baseline_single_floor    # 或 baseline_full_competition
```

**无需**执行 `git submodule update`：子模块源码（`fast_lio`、`livox_ros_driver`、
嵌套 `ikd-Tree`）已随包实体化，版本与基线记录一致。

### B.1.1 工作区路径约定

系统把工作区根固定为 `/workspace/SimEnv`（`workspace_root` 参数与容器挂载点）。
推荐按 `compose.yaml` 把 `<WS>` 挂载到该路径；否则需要覆盖 `workspace_root`。

---

## B.2 依赖安装

### B.2.1 方式一：Docker（推荐，二选一）

**(a) 由仓库构建镜像**（`docker/Dockerfile`，基于 `nvidia/cuda:12.8.0-devel-ubuntu20.04`）：

```bash
cd <WS>
cp .env.example .env                # 可选：设置镜像名、显示、UID/GID
docker compose build simenv         # 内含 ros-noetic-desktop-full + rosdep + LibTorch
docker compose run --rm simenv bash # 进入容器，工作目录 /workspace/SimEnv
```

**(b) 使用已有等价镜像**：

```bash
cd <WS>
docker run -it --rm --network host --gpus all --shm-size 2g \
  -v "$PWD":/workspace/SimEnv \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  simenv:noetic-cu128 bash
```

> `--gpus all` 与 X11 挂载对**运行**是可选：无 GPU 时控制器自动回退 CPU；
> 无显示环境请用 `GUI=false`。

### B.2.2 方式二：裸机 APT 依赖（要点）

```bash
# 1) ROS Noetic
sudo apt update
sudo apt install -y ros-noetic-desktop-full python3-catkin-tools python3-rosdep \
                    python3-numpy python3-opencv python3-yaml python3-pip python3-vcstool

# 2) 工程与系统库
sudo apt install -y build-essential cmake pkg-config git \
                    libeigen3-dev libarmadillo-dev liblcm-dev libyaml-cpp-dev \
                    libapr1-dev qtbase5-dev ninja-build mesa-utils x11-apps

# 3) 由 package.xml 解析剩余 ROS 依赖
sudo rosdep init && rosdep update --include-eol-distros --rosdistro=noetic
rosdep install --from-paths src --ignore-src -y --rosdistro=noetic

# 4) LibTorch（C++）：下载 libtorch-cxx11-abi-shared-with-deps-2.7.1+cu128.zip
#    解压到 /opt/libtorch，或构建时用 -DLIBTORCH_ROOT=<路径> 指定
```

（`docker/Dockerfile` 中的 `rosdep install` 步骤是当前最权威的依赖解析方式；
裸机若遇依赖缺失，建议优先使用 Docker。）

---

## B.3 编译

两模式命令相同，**在 `<WS>` 根目录**执行（容器内或裸机均已 source ROS）：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

说明：

- `src/CMakeLists.txt` 是指向 ROS 顶层 `toplevel.cmake` 的标准 catkin 符号链接
  （包内已保留；在未安装 ROS 的宿主机上该链接不可解析属正常，进入 ROS 环境即正常）。
- LibTorch 不在 `/opt/libtorch` 时：`catkin_make -j -DLIBTORCH_ROOT=/your/libtorch`。
- **模式 B 需先应用基线自带子模块补丁**（预存在的基线缺陷，见 B.10 与
  `manifest/PACKAGE_AUDIT.md`）：

```bash
patch -p1 -d src/fast_lio         < scripts/patches/fast_lio_message_dependency.patch
patch -p1 -d src/livox_ros_driver < scripts/patches/livox_ros_driver_skip_sdk_clone.patch
```

- 不应用上述补丁时，模式 B 会在 `fast_lio` 处报 `Pose6D.h: No such file or directory`，
  或在 `livox_ros_driver` 处尝试联网 clone Livox-SDK（离线环境会卡住）。

---

## B.4 环境变量与 source

```bash
source /opt/ros/noetic/setup.bash
cd <WS>
source ./devel/setup.bash

# 与宿主机其它 ROS/Gazebo 会话隔离的端口（两个终端必须一致）
export ROS_MASTER_URI=http://127.0.0.1:11511
export GAZEBO_MASTER_URI=http://127.0.0.1:11545

# 需要可视化时（容器内 GUI）
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=/tmp/.Xauthority
```

容器 entrypoint 会自动 source ROS 与 `devel/setup.bash`；`auto.sh` 自身还会
导出 `ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`PYTHONPATH`（工作区 `src` 下的
`building_generator_*`）以及 `GAZEBO_MODEL_PATH`、`GAZEBO_PLUGIN_PATH`。

---

## B.5 场景加载 / 生成

场景由 `auto.sh` 在启动时调用
`src/building_obstacles/scripts/generate_competition_scene.py` 生成，
输出到 `<WS>/generated_building/`（世界文件、公开信息、门/电梯服务配置）
与 `<WS>/results/`（真值，仅评估用）。

模式 A（验证运行所用冻结条件）：

```text
SEED=77   FLOOR_COUNT=1   ROOMS_PER_FLOOR=4
BUILDING_WIDTH=20.0  BUILDING_LENGTH=36.0
DANGER_COUNT=3:6     DISTRACTOR_COUNT=4:8
ROBOT_X=0.0  ROBOT_Y=-3.2  ROBOT_Z=0.6  ROBOT_YAW=1.5708
ENABLE_GROUND_TRUTH=0  ENABLE_REFEREE_ODOM=0  WRITE_GENERATED_TRUTH_COPY=0
```

模式 B（多楼层）：

```text
SEED=20260507  FLOOR_COUNT=3  ROOMS_PER_FLOOR=4
（其余同上）
```

只生成场景、不启动 Gazebo（可选）：

```bash
source ./devel/setup.bash
rosrun building_obstacles generate_competition_scene.py \
  --seed 77 --floor-count 1 --rooms-per-floor 4 \
  --width 20 --length 36 --danger-count 4 --distractor-count 6 \
  --output-dir ./generated_building --results-dir ./results
```

关键产物：

| 文件 | 内容 |
|---|---|
| `generated_building/competition_scene.world` | Gazebo 世界（由 `auto.sh` 通过 `BUILDING_WORLD_FILE` 加载） |
| `generated_building/team_scene_info.json` | 公开场景信息（起点、公开门/电梯 id、允许接口、结果路径）——算法唯一允许读取的公开文件 |
| `generated_building/door_config.yaml` / `elevator_config.yaml` | 由环境服务读取（**禁止**作为算法输入） |
| `results/danger_truth.json` | 危险源真值（**禁止**作为算法输入） |

机器人（A1）出生点由 `ROBOT_X/Y/Z/YAW` 决定，经
`unitree_guide multi_floor_gazeboSim.launch` 参数传入并 spawn；
`auto.sh` 会等待 spawn 成功（`/gazebo/get_model_state` 或日志关键字，超时
`ROBOT_SPAWN_TIMEOUT=120`）后再启动门/电梯服务与控制器。

---

## B.6 程序启动

### B.6.1 模式 A —— 单层完整闭环

**终端 1：环境**

```bash
cd <WS>
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
SEED=77 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 \
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 \
GUI=true ENABLE_REALSENSE=1 ENABLE_SENSORS=1 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 \
./auto.sh
```

（等价脚本副本：`tools/baselines/floor0_video_return_v1/start_sim.sh`）

**终端 2：任务栈**（验证运行使用的冻结 wrapper，与验证运行时逐字节一致）

```bash
cd <WS>
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
roslaunch tools/baselines/floor0_video_return_v1/floor0_full_replay.launch max_time:=1800.0
```

生产默认入口：

```bash
roslaunch competition_bringup competition_system.launch
```

**参数确认（验证运行时执行过）**

```bash
rosparam get /competition_loop/max_time          # 期望 1800.0（wrapper 覆盖后）
rosparam get /use_sim_time                       # 期望 true
rosparam get /planning/room_mission_enabled      # 期望 true
rosparam get /planning/visual_scan_route_spacing # 期望 4.0
```

### B.6.2 模式 B —— 真实比赛完整流程

**终端 1：环境（多楼层）**

```bash
cd <WS>
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
SEED=20260507 FLOOR_COUNT=3 ROOMS_PER_FLOOR=4 \
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 \
GUI=true ENABLE_REALSENSE=1 ENABLE_SENSORS=1 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 \
./auto.sh
```

**终端 2：完整比赛流程**

```bash
roslaunch competition_bringup competition_pipeline_full.launch \
    max_time:=1800.0 \
    start_floor:=0
```

该 wrapper 强制 `mission_mode=multi_floor`、`elevator_executor_enabled=true`，
并透传电梯流程超时参数（`locate_timeout=180.0`、`elevator_request_timeout=30.0`、
`floor_switch_ready_timeout=60.0`、`max_elevator_retries=1`、`door_service_timeout=45.0`）。

**可选观测**

```bash
rostopic echo /competition/state
rostopic echo /competition/floor_state                 # -1 = 电梯过渡中
rostopic echo /competition/mapping/current_floor
rostopic echo /competition/elevator/status
rostopic echo /competition/elevator/mission
rostopic echo /competition/planning/exploration_complete
```

---

## B.7 任务执行流程

### B.7.1 模式 A

```text
WAITING
  └─ 接口合规校验（禁止话题/文件）+ 参数与时钟检查 + start_delay
RUNNING
  └─ 自主探索：Frontier 选点 → 路径 → 跟随
     · 每到达探索目标：有界主动视觉扫描（原地多朝向旋转）
     · 物理学楔住：接触学习（探索阶段）+ 有界恢复
  └─ 危险源识别：RGB-D 检测 → 三维反投影 → 世界变换 → 距离去重
  └─ CURRENT_FLOOR_COMPLETE（探索完成去抖 + 最后扫描）
RETURNING
  └─ 返航路径生成与执行（起点容差 0.25 m，连续 3 帧采样）
ARRIVED_START
FINISHED
（异常路径：ERROR；安全兜底：/competition/emergency_stop = 1）
```

### B.7.2 模式 B

```text
WAITING
  └─ 同上合规校验；读取公开场景 → 生成楼层计划（电梯服务楼层升序，确定性）
RUNNING
  └─ FLOOR_EXPLORATION          # 当前楼层探测（含视觉覆盖）
  └─ ELEVATOR_LOCATE            # 用官方门服务开厅门 + 有界重观测定位门口袋
  └─ ELEVATOR_REQUEST           # /call_elevator（含目标楼层）
  └─ ELEVATOR_TRANSIT           # 执行器接管受约束局部运动（进入轿厢）
  └─ FLOOR_SWITCH               # sentinel → 地图过渡 → 定位/规划复位 → 激活新楼层
  └─ …（对每一层重复上述流程；危险源 registry 跨层不清空）
  └─ ALL_FLOORS_COMPLETE
  └─ RETURN_TO_START_FLOOR      # 若不在起始楼层，先乘电梯返回
  └─ RETURN_HOME                # 主 FSM 进入 RETURNING
  └─ ARRIVED_START
FINISHED
（异常路径：ERROR）
```

---

## B.8 结果输出

### B.8.1 结果文件

```text
<WS>/results/detected_danger.json
```

格式（官方 schema，原子写入，不会出现半写文件）：

```json
{
  "exploration_time": 783.5,
  "detected_danger_sources": [
    {"position": [x, y, z]}
  ]
}
```

写入前处理：候选校验（frame=world、位置有限）→ 距离去重
（`hazard_candidate_dedup_distance`，默认 0.30 m）→ 每源聚合
（`hazard_cluster_radius`，默认 1.0 m，z 取均值、confidence 取最大、count 记观测数）
→ 只写 `hazard_allowed_labels`（默认仅 `red_sphere`）内的源。

### B.8.2 状态与日志

```text
任务相位：   /competition/state（WAITING/RUNNING/RETURNING/FINISHED/ERROR）
安全信号：   /competition/enabled、/competition/emergency_stop
Gazebo：     <WS>/logs/competition_gazebo.log
控制器：     <WS>/logs/junior_ctrl.log（含 RL_HANDOFF_READY 就绪标志）
门/电梯：    <WS>/logs/building_control.log
ROS 节点：   ROS_HOME 下按运行建立的目录（本仓库默认 <WS>/artifacts/ros/log/<uuid>/）
```

### B.8.3 成功判据（模式 A 已实测口径）

```text
1) /competition/state 依次出现 RUNNING → CURRENT_FLOOR_COMPLETE → RETURNING
   → ARRIVED_START → FINISHED；
2) 未触发 max_time（MAX_TIME_REACHED = false）；
3) results/detected_danger.json 的条目与 results/danger_truth.json 的真值源
   在 1.0 m 容差内一一对应（验证运行：6/6，误报 0）；
4) 全程无人工干预（MANUAL_INTERVENTION_COUNT = 0）；
5) 无真值反馈（GROUND_TRUTH_FEEDBACK_COUNT = 0）。
```

模式 B 的成功判据与上述结构相同，但需额外满足跨楼层流程（`FLOOR_SWITCH` /
`RETURN_TO_START_FLOOR`）完成；**该口径下的端到端运行尚未执行**（见 B.10）。

---

## B.9 正常停止与重新运行

### B.9.1 停止

```bash
# 1) 在任务栈终端 Ctrl-C（结束 roslaunch）
# 2) 在环境终端 Ctrl-C（结束 auto.sh，Gazebo 随之退出）
```

终端丢失/进程残留时按 PID 清理（**不要**使用过宽的 `pkill -f` 模式）：

```bash
ps -ef | grep -E 'roslaunch|gzserver|gzclient|junior_ctrl|roscore' | grep -v grep
kill <PID>        # 必要时 kill -9 <PID>
```

### B.9.2 重新运行

```bash
rm -rf <WS>/generated_building <WS>/results <WS>/logs <WS>/artifacts/ros
# 重新执行 B.6 的“环境 → 任务栈”两步
```

### B.9.3 端口冲突

- `compose.yaml` 使用 `network_mode: host`，同一时刻只应有一套 ROS/Gazebo 在跑；
- 宿主机已有会话占用 `11311/11345` 时，改用隔离端口
  （本包验证运行使用 `11511/11545`），且**两个终端设置必须一致**；
- 查看当前占用：`ss -ltnp | grep -E '11311|11511|11345|11545'`。

---

## B.10 已验证边界（必须如实标注）

| 模式 | 状态 |
|---|---|
| **A：单层完整闭环** | **runtime validated**：3 次端到端 Gazebo 完整闭环（`20260914-101516` / `121632` / `153840`），6/6 危险源、0 误报、0 人工干预、0 真值反馈，`CURRENT_FLOOR_COMPLETE → RETURNING → ARRIVED_START → FINISHED`。注：实测耗时 783–854 s，是在 1800 s 验证看门狗下测得，**均大于**正式 600 s 预算，本包不主张 600 s 内的性能结论。 |
| **B：真实比赛完整流程** | **code complete / build + unit + static validated / full end-to-end runtime NOT yet validated**：代码完整（多楼层、电梯、跨层返航、全局 registry、结果序列化均已实现），构建通过，单元测试 380/380 通过，接口双向闭环静态审计通过；但完整多楼层端到端 Gazebo 运行、电梯行为参数标定、跨楼层返航精度均未验证。 |

预存在的基线问题（**未在提交包中修复，仅如实报告**）：

```text
PREEXISTING_BASELINE_ISSUE_1 =
  fast_lio @ e6aeba1（模式 B）缺少消息生成依赖声明（add_dependencies），
  -j 并行构建可能早于消息生成而失败（fast_lio/Pose6D.h 缺失）。
  基线自带修复：scripts/patches/fast_lio_message_dependency.patch

PREEXISTING_BASELINE_ISSUE_2 =
  livox_ros_driver @ 3d240d5（模式 B）在缺少系统 Livox-SDK 时会于 CMake
  configure 阶段联网 clone github.com/Livox-SDK/Livox-SDK.git（离线环境会卡住）。
  基线自带修复：scripts/patches/livox_ros_driver_skip_sdk_clone.patch

注：模式 A 的 fast_lio @ bf5f051 已包含上述两类修复，无需打补丁。
```

其他已知限制：

- 仿真推进速度约为实时 0.15–0.18×，1800 s 仿真预算可能需要数小时真实时间；
- 模式 A 的 RETURNING 阶段自愈能力有限（返程楔住属已知偶发失败模式，4 次验证
  运行中有 1 次因此作废，已在来源材料中如实记录）；
- 模式 B 的电梯/门槛参数（`locate_min_pocket_cells`、观测路点数量、approach
  standoff、cabin depth）尚未标定。
