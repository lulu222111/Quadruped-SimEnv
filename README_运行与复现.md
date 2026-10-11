# 运行与复现说明（从解压开始）

本文件说明如何**仅使用本提交包解压后的内容**，把两个基线各自构建并启动。
不依赖任何原开发机绝对路径、现有 git 分支、历史容器或已有 `build`/`devel`。

> 两个基线互相独立：请进入你要复现的目录（`baseline_single_floor/` 或
> `baseline_full_competition/`）后执行下述命令。下文用 `<WS>` 表示该目录。

---

## 1. 运行环境矩阵（依据源码审计，非经验填写）

依据：`docker/Dockerfile`、`compose.yaml`、`docs/quick-start.md`、各 `package.xml`、
各 `CMakeLists.txt`、`auto.sh`。

### 1.1 REQUIRED（必需）

| 项 | 值 | 依据 |
|---|---|---|
| 操作系统 | Ubuntu 20.04（或兼容的 glibc 环境） | `docker/Dockerfile` 基于 `nvidia/cuda:12.8.0-devel-ubuntu20.04`；`docs/quick-start.md` |
| ROS | ROS Noetic（`ros-noetic-desktop-full`） | `docker/Dockerfile`；`auto.sh` source `/opt/ros/noetic/setup.bash` |
| 仿真 | Gazebo Classic **11.15.1**（`ros-noetic-desktop-full` 自带） | 镜像内 `gazebo --version` = 11.15.1 |
| 构建系统 | catkin（`catkin_make`；镜像内也提供 `catkin_tools`） | `src/CMakeLists.txt`（catkin toplevel 符号链接）、`scripts/build-workspace.sh` |
| 语言运行时 | Python 3.8；C++（g++ 9 / C++14 及以上） | 镜像内 `python3 --version` = 3.8.10；`g++ --version` = 9.4.0 |
| LibTorch（C++） | **LibTorch 2.7.1+cu128**，安装路径 `/opt/libtorch` | `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt`：`find_package(Torch REQUIRED CONFIG)`，`LIBTORCH_ROOT` 默认 `/opt/libtorch` |
| CUDA 工具链 | CUDA 12.8（与 LibTorch cu128 配套） | `docker/Dockerfile` |
| 系统库 | `libeigen3-dev`、`libarmadillo-dev`、`liblcm-dev`、`libyaml-cpp-dev`、`qtbase5-dev`、`libapr1-dev`（APT `libapr1`） | 各 `CMakeLists.txt`（Eigen/PCL/APR/LCM/Qt）、`docker/Dockerfile` |
| 关键 ROS 依赖 | `pcl_ros`、`cv_bridge`、`image_transport`、`message_filters`、`tf2_ros`、`tf2_geometry_msgs`、`gazebo_ros`、`gazebo_plugins`、`controller_manager`、`joint_state_controller`、`robot_state_publisher`、`rviz`、`rostest`、`message_generation` | 各 `package.xml`（见 `docs/附录A_素材.md` A.8） |
| Python 包 | `numpy`、`opencv-python`（`python3-opencv`）、`PyYAML` | `competition_perception`（HSV/形态学/轮廓）、场景生成与评估脚本 |
| 磁盘 | 建议 ≥ 15 GB（源码 + 完整 catkin 构建 + Gazebo 资源） | 实测构建后 `build/`+`devel/` 约 2–3 GB，镜像本身约 10 GB |

### 1.2 OPTIONAL（可选）

| 项 | 说明 |
|---|---|
| NVIDIA GPU + 驱动 + `nvidia-container-toolkit` | **不是运行必需**。RL 控制器在源码中先探测 `torch::cuda::is_available()`，不可用时自动回退 `torch::kCPU`（`State_RL_test.cpp` 第 780–787 行）。GPU 仅影响渲染与仿真推进速度。 |
| Gazebo GUI（`GUI=true`） | 可视化需要可用的 X11/OpenGL；无显示环境请用 `GUI=false`。 |
| LibTorch CUDA 架构 | `CMakeLists.txt` 里 `TORCH_CUDA_ARCH_LIST` 默认 `"12.0"`。若使用与本开发机不同算力的 GPU 且希望启用 GPU 推理，请用 `-DTORCH_CUDA_ARCH_LIST=<你的算力>` 覆盖；不启用 GPU 时无需处理。 |
| Livox-SDK（`/usr/local`） | 仅真实 Livox 硬件驱动需要。仿真链路使用 Gazebo 激光插件，不需要它；缺失时 `livox_ros_driver` 的处理方式见 4.3 节。 |

### 1.3 DEVELOPMENT_MACHINE_ONLY（开发机专用，非运行必需）

- 开发过程中使用过 RTX 5090 与 X11 转发进行录制与可视化；**代码未对具体 GPU 型号做任何硬性要求**。
- `scripts/` 下的 `verify-*`、`monitor_drift.py`、`eval_localization.py` 等为开发期辅助脚本，不在正式链路中。
- 历史实验记录（`artifacts/`、`reports/` 中的部分文档）保留在包内作为来源可追溯材料，**不是运行输入**。

### 1.4 工作区路径约定（重要）

生产装配把工作区根目录固定为 **`/workspace/SimEnv`**
（`competition_bringup/launch/competition_system.launch` 的 `workspace_root` 参数、
`config/default.yaml`、`competition_loop_node` 的默认值）。
`compose.yaml` 正是把仓库挂载到该路径。

因此有两种用法：

1. **推荐**：按 `compose.yaml` 把解压后的基线目录挂载到容器内 `/workspace/SimEnv`
   （见 2.2 节），一切默认值直接可用；
2. 裸机直接跑：请把仓库放到 `/workspace/SimEnv`，或在启动时覆盖
   `~workspace_root`（例如
   `roslaunch ... --ros-args` 不适用，请改为在 launch 中覆盖该 `<param>`，
   或临时把仓库放到该路径）。

> 本提交包内**没有**任何对 `/home/ubuntu/SimEnv-*` 这类开发机路径的依赖，
> 你可以把它解压到任意目录。

---

## 2. 环境准备（从解压到可构建）

### 2.1 获取代码

```bash
unzip 哈尔滨工业大学（深圳）-黄誉铭-危境智巡--面向高危未知场景的四足机器人自主探测导航系统-核心代码.zip
cd DG-202602-core-code-submission-v1
cd baseline_single_floor          # 模式 A；模式 B 请改为 baseline_full_competition
```

包内**已包含** `src/fast_lio`、`src/livox_ros_driver`（及其嵌套 `ikd-Tree`）的
完整源码，**不需要** `git submodule update`，也不需要联网拉取子模块。

### 2.2 方式一：Docker（推荐，与验证运行环境一致）

镜像定义在 `docker/Dockerfile`（`FROM nvidia/cuda:12.8.0-devel-ubuntu20.04`，
内含 ROS Noetic + Gazebo Classic + LibTorch cu128）。

```bash
cd <WS>
cp .env.example .env                 # 按需修改镜像名/显示/用户
docker compose build simenv          # 首次构建需下载 ROS 依赖与 LibTorch，耗时较长
docker compose run --rm simenv bash  # 进入容器，工作目录即 /workspace/SimEnv
```

若宿主机已有等价的 `simenv:noetic-cu128` 镜像，也可直接：

```bash
docker run -it --rm --network host --gpus all --shm-size 2g \
  -v "$PWD":/workspace/SimEnv \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  simenv:noetic-cu128 bash
```

`entrypoint` 会自动 `source /opt/ros/noetic/setup.bash` 与工作区
`devel/setup.bash`（若已构建）。

### 2.3 方式二：本机 ROS Noetic

在已安装 ROS Noetic（含 `desktop-full`）与 LibTorch 的机器上：

```bash
cd <WS>
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

若 LibTorch 不在 `/opt/libtorch`，请覆盖：
`catkin_make -j -DLIBTORCH_ROOT=/your/libtorch`。

---

## 3. 启动方式 A：单层完整闭环（已验证）

模式 A 的端到端验证方式为两段式：**环境（仿真 + 场景 + 控制器）** 与
**任务栈（定位 + 建图 + 规划 + 感知 + 控制 + 任务状态机）**。

### 3.1 操作系统 / Docker / ROS / 依赖

见第 1 节与第 2 节（模式 A 与模式 B 使用同一套环境）。

### 3.2 构建

```bash
cd baseline_single_floor
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

### 3.3 场景生成参数（与验证运行一致）

模式 A 的验证运行使用**冻结场景条件**：

```text
SEED             = 77
FLOOR_COUNT      = 1
ROOMS_PER_FLOOR  = 4
ENABLE_GROUND_TRUTH        = 0    # 不发布真值话题
ENABLE_REFEREE_ODOM        = 0    # 不发布 /Odometry_gazebo
WRITE_GENERATED_TRUTH_COPY = 0    # 不在生成目录写真值副本
GUI                        = true # 需要可视化时；无显示环境改 false
ENABLE_REALSENSE           = 1    # RGB-D 必须在线
UNITREE_AUTO_CMD_VEL       = 1    # 无人值守：控制器自动站立并进入 /cmd_vel 模式
CONTROLLER_FOREGROUND      = 0    # 控制器后台运行
```

场景由 `auto.sh` 调用
`src/building_obstacles/scripts/generate_competition_scene.py` 生成，
输出到 `<WS>/generated_building/`（`competition_scene.world`、`team_scene_info.json`
等公开信息）与 `<WS>/results/`（真值文件仅供离线评估）。

### 3.4 启动终端 1 —— 仿真环境（Gazebo + 机器人 + 控制器）

```bash
# 在容器内（工作目录 /workspace/SimEnv）
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash

export ROS_MASTER_URI=http://127.0.0.1:11511     # 与宿主机其它 ROS 会话隔离
export GAZEBO_MASTER_URI=http://127.0.0.1:11545

SEED=77 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4 \
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 \
GUI=true ENABLE_REALSENSE=1 ENABLE_SENSORS=1 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 \
./auto.sh
```

`auto.sh` 会依次：清理旧进程 → 生成随机场景 → 启动 Gazebo 与 A1 模型/传感器 →
启动门/电梯控制服务 → 启动 `junior_ctrl` 控制器。
等价的最小脚本见 `tools/baselines/floor0_video_return_v1/start_sim.sh`
（即验证运行实际使用的启动脚本副本）。

### 3.5 启动终端 2 —— 任务栈（本次验证运行使用的冻结配置）

```bash
# 另开一个容器终端，同样 source 环境并设置同样的两个 *_MASTER_URI
cd /workspace/SimEnv

roslaunch tools/baselines/floor0_video_return_v1/floor0_full_replay.launch \
    max_time:=1800.0
```

该 launch 是**验证用 wrapper**（`tools/baselines/floor0_video_return_v1/`，
与验证运行时逐字节一致），它只**选取**已存在于生产 launch 中的参数：

```text
include localization_corrected.launch   correction_enabled=true
                                        correction_mode=planar_relative
include competition_system.launch       localization_enabled=false
                                        floor_lifecycle_enabled=true
                                        planning_room_mission_enabled=true
                                        planning_room_scan_zones = 4 个房间框
                                        planning_visual_scan_route_spacing=4.0
之后覆盖 /competition_loop/max_time     = 1800.0（验证看门狗）
```

> 正式比赛预算是 600 s；1800 s 只是验证期间的看门狗，避免被 600 s 硬截断。
> 想按正式预算跑，去掉 `max_time:=1800.0` 即可（默认 7200，生产节点内为 600）。

**确认参数确实生效**（验证运行时也做了这一步）：

```bash
rosparam get /competition_loop/max_time      # 期望 1800.0
rosparam get /use_sim_time                   # 期望 true
rosparam get /planning/room_mission_enabled  # 期望 true
```

### 3.6 观察任务状态（预期流程）

```bash
rostopic echo /competition/state        # WAITING → RUNNING → RETURNING → FINISHED
rostopic hz /competition/map            # 建图持续输出
rostopic echo /competition/planning/status
rostopic echo /competition/perception/status
rostopic echo /competition/hazard_candidates
```

预期状态序列（与验证运行一致）：

```text
WAITING
→ RUNNING
→ （自主探索：Frontier 目标 + 原地旋转视觉扫描）
→ CURRENT_FLOOR_COMPLETE
→ RETURNING
→ ARRIVED_START（起点容差内，连续多帧采样）
→ FINISHED
```

### 3.7 结果文件与日志位置

| 内容 | 路径 |
|---|---|
| 最终危险源结果 | `<WS>/results/detected_danger.json` |
| 场景真值（仅供离线评估，**非**算法输入） | `<WS>/results/danger_truth.json` |
| 生成场景与公开信息 | `<WS>/generated_building/`（`competition_scene.world`、`team_scene_info.json` …） |
| Gazebo/环境启动日志 | `<WS>/logs/competition_gazebo.log` |
| 门/电梯服务日志 | `<WS>/logs/building_control.log` |
| 控制器日志 | `<WS>/logs/junior_ctrl.log` |
| ROS 节点日志 | 由 `ROS_HOME` 决定；本仓库默认 `<WS>/artifacts/ros/log/<uuid>/`（逐节点 stdout/log） |
| 任务状态 | ROS 话题 `/competition/state`（可用 `rostopic echo` 或 rosbag 记录） |

离线评估命令（可选）：

```bash
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

### 3.8 如何判断任务成功

```text
1) /competition/state 出现过 CURRENT_FLOOR_COMPLETE 与 RETURNING；
2) 最终状态为 FINISHED（不是 ERROR、不是 max_time reached）；
3) results/detected_danger.json 存在，且 detected_danger_sources 条目与
   results/danger_truth.json 的源在 1.0 m 容差内一一对应（6/6）；
4) 全程无人工干预、未订阅任何真值话题（可用 3.9 节审计命令复核）。
```

### 3.9 合规自检（可复现验证运行中的 GT 审计）

```bash
for t in /Odometry_gazebo /ground_truth/odom /ground_truth/state; do
  echo "--- $t"; rostopic info "$t" 2>/dev/null | sed -n '/Subscribers:/,$p'
done
rostopic list | grep -E 'ground_truth|Odometry_gazebo' || echo "(none)"
rosnode list
```

预期：`/Odometry_gazebo` 与 `/ground_truth/odom|state` **不存在或无人订阅**；
`/gazebo/model_states`、`/gazebo/link_states` 即使被 Gazebo 发布，也**没有生产
节点订阅**。验证运行的实际审计输出见
`tools/baselines/floor0_video_return_v1/VALIDATED_RUN_GT_COMPLIANCE.txt`。

---

## 4. 启动方式 B：真实比赛完整流程

### 4.1 环境准备 / 构建 / source

与第 2 节完全一致，只需把目录换成 `baseline_full_competition/`：

```bash
cd baseline_full_competition
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

### 4.2 场景 / 环境启动

模式 B 需要**多楼层**场景（默认 `FLOOR_COUNT=3`）：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11511
export GAZEBO_MASTER_URI=http://127.0.0.1:11545

SEED=20260507 FLOOR_COUNT=3 ROOMS_PER_FLOOR=4 \
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0 \
GUI=true ENABLE_REALSENSE=1 ENABLE_SENSORS=1 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 \
./auto.sh
```

（多楼层场景生成同样由 `generate_competition_scene.py` 完成，
`--floor-count` 即来自 `FLOOR_COUNT`。）

### 4.3 程序启动

```bash
roslaunch competition_bringup competition_pipeline_full.launch \
    max_time:=1800.0 \
    start_floor:=0
```

该 wrapper 强制：

```text
mission_mode                = multi_floor
elevator_executor_enabled   = true
```

并透传 `locate_timeout=180.0`、`elevator_request_timeout=30.0`、
`floor_switch_ready_timeout=60.0`、`max_elevator_retries=1`、
`door_service_timeout=45.0`。

**关于子模块补丁的说明（重要，属预存在的基线问题）**

模式 B 的子模块源码版本（`fast_lio` = `e6aeba1`、`livox_ros_driver` = `3d240d5`）
在**裸状态下无法直接编译**，原因是：

1. `fast_lio/CMakeLists.txt` 缺少消息生成依赖声明，`-j` 并行构建时
   `fastlio_mapping` 可能早于消息生成而报
   `fatal error: fast_lio/Pose6D.h: No such file or directory`；
2. `livox_ros_driver/CMakeLists.txt` 在找不到系统 Livox-SDK 时会**联网 clone**
   `https://github.com/Livox-SDK/Livox-SDK.git`，离线环境下会卡住。

来源基线自带修复补丁（本提交包内已包含，未做任何改写）：

```bash
patch -p1 -d src/fast_lio            < scripts/patches/fast_lio_message_dependency.patch
patch -p1 -d src/livox_ros_driver    < scripts/patches/livox_ros_driver_skip_sdk_clone.patch
```

按提交要求，本包**没有**替基线修复这个问题（未修改原基线代码），只如实报告。
处理后 `catkin_make -j` 可完整构建通过（已在本提交包的解压副本上验证）。
完整记录见 `manifest/PACKAGE_AUDIT.md` 的 `PREEXISTING_BASELINE_ISSUE` 一节。

### 4.4 任务执行流程（代码层相位）

```text
WAITING
→ RUNNING
→ FLOOR_EXPLORATION
→ ELEVATOR_LOCATE        （用官方 /set_door_state 开厅门后做有界重观测，定位门口袋）
→ ELEVATOR_REQUEST       （调用官方 /call_elevator）
→ ELEVATOR_TRANSIT
→ FLOOR_SWITCH           （sentinel → 过渡 → planning/localization 复位 → 激活新楼层）
→ …（下一层重复）…
→ ALL_FLOORS_COMPLETE
→ RETURN_TO_START_FLOOR  （若不在起始楼层，先乘电梯返回）
→ RETURN_HOME            （主 FSM 进入 RETURNING）
→ ARRIVED_START
→ FINISHED
```

### 4.5 关键 ROS 状态与停止方式

```bash
rostopic echo /competition/state                      # 顶层相位
rostopic echo /competition/floor_state                # -1 = 电梯过渡中
rostopic echo /competition/mapping/current_floor
rostopic echo /competition/elevator/status
rostopic echo /competition/elevator/mission
rostopic echo /competition/planning/exploration_complete
```

以及结果与日志位置（与模式 A 相同）：

```text
结果：  <WS>/results/detected_danger.json
日志：  <WS>/logs/{competition_gazebo,building_control,junior_ctrl}.log
        ROS_HOME 下的逐节点日志目录
```

停止方式：先 `Ctrl-C` 结束任务栈终端，再 `Ctrl-C` 结束 `auto.sh` 终端；
如需强制清理，按第 6 节处理（注意不要用过宽的 `pkill -f` 模式）。

### 4.6 已验证边界（必须如实理解）

```text
MODE B = code complete
         build PASS
         unit/static PASS（380/380 单元测试，0 failure / 0 error；接口双向闭环审计 PASS）
         **完整多楼层端到端 Gazebo 运行尚未验证**
```

也就是说："代码已经完成"不等于"完整多楼层比赛流程已经通过仿真验证"。
以下属于待验证项：多楼层端到端运行、电梯行为参数标定
（`locate_min_pocket_cells`、观测路点数量、approach standoff、cabin depth）、
跨楼层返航精度、600 s 预算下的性能。

---

## 5. 日志与结果说明（两种模式通用）

| 你想知道什么 | 看哪里 |
|---|---|
| 程序是否跑起来了 | `<WS>/logs/competition_gazebo.log`（Gazebo/launch）、`<WS>/logs/junior_ctrl.log`（控制器） |
| 任务推进到哪一步 | `/competition/state` 话题；ROS 节点日志目录中 `competition_loop` 的 stdout |
| 建图/规划是否正常 | `mapping` / `planning` 节点日志，以及 `/competition/map`、`/competition/path` 话题频率 |
| 危险源是否被识别 | `/competition/hazard_candidates` 话题、`perception` 节点日志 |
| 最终结果 | `<WS>/results/detected_danger.json` |
| 异常/失败原因 | 各节点日志 + `competition_loop` 的 `ERROR` 相位描述；`/competition/emergency_stop` |

日志约定：ROS 节点使用 `rospy` 标准日志（`ROS_HOME` 下按运行时间建立目录，
每个节点一个 `.log` 与一个 stdout 文件）；仿真与控制器使用 `auto.sh` 重定向到
`<WS>/logs/*.log`。**判断运行成功请只看本次运行的日志与结果文件**，不需要查阅
任何历史开发记录。

---

## 6. 正常停止与重新运行

### 6.1 正常停止

```bash
# 1) 停止任务栈终端（Ctrl-C 结束 roslaunch）
# 2) 停止环境终端（Ctrl-C 结束 auto.sh；Gazebo 会随之退出）
```

若终端已丢失或进程残留，请**按 PID 精确清理**，避免使用过宽的
`pkill -f` 模式（过宽的模式可能匹配到你自己的命令或无关进程）：

```bash
ps -ef | grep -E 'roslaunch|gzserver|gzclient|junior_ctrl|roscore' | grep -v grep
kill <PID>          # 必要时再 kill -9 <PID>
```

### 6.2 重新运行

```bash
# 清理上一轮生成物，避免统计混入旧结果（可选但推荐）
rm -rf <WS>/generated_building <WS>/results <WS>/logs
rm -rf <WS>/artifacts/ros        # ROS 日志目录（如使用默认 ROS_HOME）

# 然后重复第 3 节 / 第 4 节的“环境 → 任务栈”两步
```

### 6.3 避免端口冲突

- `compose.yaml` 使用 `network_mode: host`，因此**不要**同时启动两套 ROS/Gazebo。
- 宿主机若有其它 ROS 会话占用 `11311/11345`，请像验证运行一样改用隔离端口：
  `ROS_MASTER_URI=http://127.0.0.1:11511`、`GAZEBO_MASTER_URI=http://127.0.0.1:11545`，
  **两个终端必须设置同样的值**。
- Gazebo GUI 若意外掉窗口（GPU 复位/休眠），`gzserver` 通常仍正常：
  只需重挂观察者 `gzclient`，不必重启整套仿真。

---

## 7. 已知限制（如实列出）

| 项 | 状态 |
|---|---|
| 模式 A 单层完整闭环 | 已验证（3 次 PASS，见 `README_提交说明.md` 第 1 节） |
| 模式 B 多楼层端到端 | 未验证（代码完成 + 构建/单元/静态通过） |
| 模式 B 子模块裸构建 | 预存在缺陷，需按 4.3 节应用基线自带补丁 |
| 模式 B 电梯行为参数 | 未标定（代码完整，阈值待仿真调） |
| 模式 B 跨楼层返航精度 | 未验证 |
| 600 s 正式预算性能 | **未验证**：模式 A 的 3 次 PASS 实测耗时 783–854 s，是在 1800 s 验证看门狗下测得的，均**大于**正式 600 s 预算；600 s 口径下的成绩需按实际配置重新测量，本包不主张任何 600 s 内的性能结论 |
| 仿真推进速度 | 实测约 0.15–0.18× 实时，1800 s 仿真预算可能需要数小时真实时间 |
