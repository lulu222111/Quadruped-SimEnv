# floor0_video_return_v1 —— 已验证单层完整闭环的复现入口

本目录是**验证/复现入口**，属于模式 A（`baseline_single_floor/`）的一部分，
**不是生产默认配置**。它冻结了产生"3 次完整闭环 PASS"的那一套单层配置。

## 目录内容

| 文件 | 来源 | 说明 |
|---|---|---|
| `floor0_full_replay.launch` | 验证 run 目录（`20260914-153840`） | 本次验证使用的 wrapper，**字节一致**（SHA256 `08c095ed...`）。只"选取"生产 launch 中已存在的参数，不改任何算法。 |
| `start_sim.sh` | 同上 | 环境启动脚本：冻结场景条件（`SEED=77 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4`、关闭真值通道、开启 RGB-D、`GUI` 可选），最后 `exec bash ./auto.sh`。 |
| `run_mission.sh` | 同上 | 验证运行编排脚本（readiness 等待 → RGB-D 相机 gate → rosbag/watcher → 启动任务栈 → 实时参数与真值合规审计 → 等 FSM 结束）。 |
| `watcher.py` | 同上 | 10 s 采样观察器（可选工具，用于记录 sim_time / FSM / pose / 计划 / 跟随状态）。 |
| `VALIDATED_RUN_RESULT.md` | 同上（原 `RUN_RESULT.md`） | 该次 PASS 的原始结果记录（耗时 783.50 s、6/6 危险源、0 误报）。 |
| `VALIDATED_RUN_PARAMS.txt` | 同上（原 `params.txt`） | 运行中实时回读的 ROS 参数（`max_time=1800.0` 等）。 |
| `VALIDATED_RUN_GT_COMPLIANCE.txt` | 同上（原 `gt_compliance.txt`） | 真值合规审计输出（无真值发布者/无生产订阅者）。 |

未随包内容（按要求排除的大体积运行产物）：`run.bag`（约 250 MB）、
`competition_stack.log` 等。

## 冻结配置

```text
场景        SEED=77  FLOOR_COUNT=1  ROOMS_PER_FLOOR=4
真值/裁判   ENABLE_GROUND_TRUTH=0  ENABLE_REFEREE_ODOM=0  WRITE_GENERATED_TRUTH_COPY=0
控制器      UNITREE_AUTO_CMD_VEL=1  CONTROLLER_FOREGROUND=0
显示        GUI=true（可改 false）
定位        localization_corrected.launch  correction_enabled=true  planar_relative
任务栈      competition_system.launch      localization_enabled=false
                                          floor_lifecycle_enabled=true
                                          planning_room_mission_enabled=true
                                          planning_room_scan_zones = 4 个房间框
                                          planning_visual_scan_route_spacing=4.0
时限        /competition_loop/max_time = 1800.0（验证看门狗；正式预算 600 s 不变）
```

## 使用方法（两条命令）

```bash
# 终端 1：环境（场景 + Gazebo + 控制器）
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash   # 在工作区根目录
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
bash tools/baselines/floor0_video_return_v1/start_sim.sh

# 终端 2：任务栈
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 GAZEBO_MASTER_URI=http://127.0.0.1:11545
roslaunch tools/baselines/floor0_video_return_v1/floor0_full_replay.launch max_time:=1800.0
```

启动后可用 `rosparam get /competition_loop/max_time` 确认覆盖是否生效
（期望 `1800.0`）。

## 说明

- `run_mission.sh` 内部按验证时的运行目录硬编码了 `RUN_DIR`
  （`/workspace/SimEnv/artifacts/floor0-video-return/20260914-153840`），
  并会调用同目录的 `start_sim.sh`、`watcher.py` 与该 run 目录下的 launch。
  如需脚本化复现，请自行把 `RUN_DIR` 指向本目录；
  直接复现建议使用上面的"两条命令"方式。
- 本目录只用于**复现与演示**；正式比赛/生产启动请使用
  `roslaunch competition_bringup competition_system.launch`。
- 正式预算为 600 s。验证运行实测耗时 783–854 s，因此**不能**据此主张 600 s 内
  的性能结论。
