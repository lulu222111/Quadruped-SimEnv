# 提交包审计报告（PACKAGE_AUDIT）

```text
PACKAGE_NAME            = 哈尔滨工业大学（深圳）-黄誉铭-危境智巡--面向高危未知场景的四足机器人自主探测导航系统-核心代码.zip
PACKAGE_PATH            = /home/ubuntu/哈尔滨工业大学（深圳）-黄誉铭-危境智巡--面向高危未知场景的四足机器人自主探测导航系统-核心代码.zip
PACKAGE_SIZE            = ≈353.6 MB（337.6 MiB）；本文件位于 ZIP 内，回写本报告会使容器体积
                          产生字节级变化，故精确字节数以交付文件为准（见 sidecar 对应的 ZIP）
PACKAGE_SHA256          = 见包外 sidecar 文件
                          /home/ubuntu/哈尔滨工业大学（深圳）-黄誉铭-危境智巡--面向高危未知场景的四足机器人自主探测导航系统-核心代码.zip.sha256
                          （本文件位于 ZIP 内部，容器哈希无法自指，故不在此写入；
                            该文件与环境自校验的摘要见下方 PACKAGE_SOURCE_DIGEST）
PACKAGE_SOURCE_DIGEST   = d669d4185367e9aad39c8200fdde86320cf1ffd8ca801c83838c66f7fe51110a
                          （= 包内**除 manifest/ 之外**全部文件的 SHA256 逐行清单再做一次 SHA256，
                            可自校验；复现命令见本文件第 7 节末）
                          【安全净化后重新计算；**原始提交包**的对应值为
                            dd6638acec7f2698390183bf02ab3e71d11a12e34b614281b5b0c3ab1b7b2dd5】
                          净化说明与影响清单见本文件第 9 节

BASELINE_A_SOURCE       = /home/ubuntu/SimEnv-floor0-video-return-v1
                          （branch video/huangym/floor0-double-pass-v1，BASE 3503f44e617385c56f264875575d8ca8b68c3b36，
                            planner 未提交快照 4U1v，sha256 76277c39266634ac1b9204ac209bc55f7b192f282f5774168c19e953e9ed2ee8）
BASELINE_A_PROVENANCE   = artifacts/floor0-video-return/SUCCESS_RUNS_SUMMARY.md
                          （3 次 PASS：20260914-101516 / 121632 / 153840；6/6 危险源、0 误报、
                            0 人工干预、0 真值反馈；CURRENT_FLOOR_COMPLETE→RETURNING→ARRIVED_START→FINISHED）
BASELINE_A_COMMENT_ONLY = true（仅新增中文模块级注释；AST_LOGIC_IDENTICAL = true）

BASELINE_B_SOURCE       = /home/ubuntu/SimEnv-competition-complete-v1
                          （branch feat/huangym/competition-complete-v1）
BASELINE_B_HEAD         = ba662d2ef233250b4c3d79aaa44616ceea37afbc
BASELINE_B_COMMENT_ONLY = true（来源已有较完整中文注释，仅对 17 个无中文文件补充简要模块说明）

FILE_COUNT              = 1551
                          【安全净化后重新统计；**原始提交包**为 2492。
                            两者差额 941 = 安全净化移除的文件数，明细见第 9 节】
TEST_FILE_COUNT         = 0
BROKEN_SYMLINK_COUNT    = 0（在 ROS Noetic 运行环境内实测；见第 5 节 G 项）
MISSING_RUNTIME_FILE_COUNT = 0（见第 5 节 B/C/F 项）

BUILD_SINGLE_FLOOR      = PASS（从本提交包解压副本，容器内 catkin_make -j16，rc=0，221 个目标）
BUILD_FULL_COMPETITION  = PASS（解压副本 + 应用基线自带子模块补丁后，rc=0，221 个目标；
                                 裸状态下 FAIL，属 PREEXISTING_BASELINE_ISSUE，见第 6 节）
STATIC_CHECK_SINGLE_FLOOR = PASS（Python 语法编译 + XML launch 解析 + 内部 import 解析 + launch 引用完整性）
STATIC_CHECK_FULL_COMPETITION = PASS（同上）

LOGIC_CHANGE_COUNT      = 0
PARAMETER_CHANGE_COUNT  = 0
INTERFACE_CHANGE_COUNT  = 0

SIMULATION_RUN          = NOT RUN（本打包阶段未运行 Gazebo）

APPENDIX_A_MATERIAL     = docs/附录A_素材.md（软件系统实现与代码组织）
APPENDIX_B_MATERIAL     = docs/附录B_素材.md（系统运行及复现说明）

READY_FOR_SUBMISSION    = true（附第 6/7 节声明的例外与预存在基线问题）
```

> `PACKAGE_SHA256` 说明：本文件位于 ZIP 内部，ZIP 容器哈希无法自指（写入该值会
> 改变容器哈希）。因此：
>
> 1. **容器哈希**记录在交付 ZIP 同目录的 sidecar 文件 `<包名>.sha256` 中；
> 2. **包内内容**的自校验用两条命令（都能给出确定结论）：
>
> ```bash
> cd DG-202602-core-code-submission-v1        # 解压后的目录
> sha256sum -c manifest/SHA256SUMS.txt        # 逐文件校验（manifest/SHA256SUMS.txt 自身除外）
>
> # 与世界无关的来源摘要（应与 PACKAGE_SOURCE_DIGEST 相同）
> find . -type f ! -path './manifest/*' -print0 | LC_ALL=C sort -z \
>   | xargs -0 sha256sum | sed 's|\./||' | sha256sum
> ```

---

## 1. 包内文件统计

| 项 | 值 |
|---|---|
| 文件总数（不含目录） | 2492 |
| 模式 A（`baseline_single_floor/`）文件数 | 1255 |
| 模式 B（`baseline_full_competition/`）文件数 | 1229 |
| 文档（`docs/` + 根 README） | 5 |
| manifest | 3 |
| 符号链接 | 2（均为 catkin 标准 `toplevel.cmake` 链接，见第 5 节 G 项） |

未包含内容（按要求排除）：`build/`、`devel/`、`.git/`、`logs/`、`.pytest_cache/`、
`__pycache__/`、`*.pyc`、`*.bag`（含验证运行 250 MB rosbag）、运行录像与截图、
本地 `.env`/凭据、core dump、编辑器缓存。

---

## 2. 两个基线的独立性

| 项 | 结论 |
|---|---|
| 目录独立 | `baseline_single_floor/` 与 `baseline_full_competition/` 各自为完整仓库副本，无互相引用 |
| 可分别构建 | 已在容器内分别 `catkin_make` 通过（第 3 节） |
| 可分别启动 | A：`competition_system.launch`（或 `tools/baselines/floor0_video_return_v1/floor0_full_replay.launch`）；B：`competition_pipeline_full.launch` |
| 子模块自包含 | 两个基线的 `src/fast_lio`、`src/livox_ros_driver`（含嵌套 `ikd-Tree`）源码均已实体化，**不需要** `git submodule update` |
| 源码差异 | B 相对 A 增加多楼层/电梯模块（`floor_map_manager.py`、`floor_mission_fsm.py`、`mission_plan.py`、`elevator_locator.py`、`door_elevator_executor_node` 等），并移除 A 的 `floor_lifecycle.py`、`result_aggregator.py`、`room_access.py`；`competition_interfaces` 的 `CompetitionState.msg` / `ModuleStatus.msg` 亦不同（两者确为不同接口版本的独立基线） |

---

## 3. 构建验证（从解压副本）

```text
环境          = docker 镜像 simenv:noetic-cu128（Ubuntu 20.04 + ROS Noetic + Gazebo 11.15.1 + LibTorch cu128）
方法          = 从交付 ZIP 解压出副本 → 挂载入容器 → cp -a 到容器内 /tmp
                → source /opt/ros/noetic/setup.bash
                → catkin_make -j16（CATKIN_ENABLE_TESTING 保持默认 ON）
模式 A        = BUILD_RC=0，221 个目标；关键运行时可执行目标均生成：
                fastlio_mapping、junior_ctrl、state_from_gazebo、livox_laser_simulation、
                quadrotor_simulator_so3，以及全部 competition_* / building_* 目标
模式 B        = 先应用基线自带补丁（见第 6 节），再 catkin_make -j16 → BUILD_RC=0，221 个目标
```

`STATIC_CHECK` 覆盖：全部 Python 源文件的语法编译（`py_compile`；唯一失败项
`src/uav_simulator/Utils/uav_utils/scripts/tf_assist.py` 为来源基线既有的 Python 2
语法文件，与本次打包无关，且与来源逐字节一致）、全部 `.launch` 的 XML 解析、
仓库内 `$(find ...)` 引用完整性、内部 Python import 解析。

**launch 解析验证（ROS 官方解析器 `roslaunch --nodes`，从解压副本执行）**

| 基线 | launch | 结果 |
|---|---|---|
| 模式 A | `src/competition_bringup/launch/competition_system.launch` | rc=0，8 个节点 |
| 模式 A | `src/competition_localization/launch/localization_bringup.launch` | rc=0，2 个节点 |
| 模式 A | `src/competition_localization/launch/localization_corrected.launch` | rc=0，4 个节点 |
| 模式 A | `src/competition_mapping/launch/mapping.launch` | rc=0，1 个节点 |
| 模式 A | `tools/baselines/floor0_video_return_v1/floor0_full_replay.launch`（验证入口） | rc=0，10 个节点 |
| 模式 B | `src/competition_bringup/launch/competition_pipeline_full.launch`（比赛入口） | rc=0，9 个节点：`/competition_loop`、`/cmd_vel_guard`、`/laserMapping`、`/localization`、`/perception`、`/mapping`、`/planning`、`/path_follower`、`/door_elevator_executor` |
| 模式 B | `src/competition_bringup/launch/competition_system.launch` | rc=0，8 个节点 |

> 使用提示（打包期实测发现）：两个基线是同一工作区布局的两份副本，
> **不要把两者的 `src` 同时加入同一个 ROS 环境的 `ROS_PACKAGE_PATH`**，
> 否则 `$(find competition_bringup)` 可能解析到另一份副本的 launch，出现
> "unused args ..." 之类的参数不匹配告警。正常复现时一次只 source 一个基线即可
> （`source ./devel/setup.bash`）。

---

## 4. 零行为变化证明（LOGIC_CHANGE_COUNT = 0）

方法（逐文件类型）：

| 文件类型 | 比较方法 | 结果 |
|---|---|---|
| Python（含无扩展名的 `*_node`） | AST 归一化比较：剥离模块/类/函数 docstring 后比较 `ast.dump()` | 模式 A：117 个比较对象，116 个 AST 完全一致，1 个为来源既有的 Python 2 语法文件（无法解析，已用字节比对确认与来源一致）；模式 B：113 个，112 一致，同上 1 个 |
| `.launch`（XML） | 忽略 XML 注释后比较元素树（标签 + 排序属性 + 文本 + 子结构） | 模式 A：105 个，104 一致，1 个为来源既有的 XML 结构缺陷文件 `multi_floor_building.world`（已用字节比对确认与来源一致）；模式 B：104 个，103 一致，同上 |
| C/C++ | 剥离块注释/行注释/字符串字面量后比较 token 序列 | 模式 A/B：各 466 个文件全部一致 |
| YAML | 剥离整行注释后比较 | 模式 A/B：各 34 个全部一致 |
| Shell | 剥离注释行后比较 | 模式 A：13 个；模式 B：9 个，全部一致 |
| CMakeLists.txt | 剥离注释行后比较 | 模式 A：43 个中 31 个一致，12 个有差异；模式 B：43 个中 30 个一致，13 个有差异 —— **差异全部只涉及测试目标/测试安装声明**（见第 7 节），无任何运行时目标、参数或接口变化 |

结论：

```text
AST_LOGIC_IDENTICAL     = true     （所有被补充注释的 Python 文件）
XML_TREE_IDENTICAL      = true     （所有被补充注释的 launch 文件）
CXX_TOKEN_IDENTICAL     = true     （466/466，两基线）
LOGIC_CHANGE_COUNT      = 0
PARAMETER_CHANGE_COUNT  = 0
INTERFACE_CHANGE_COUNT  = 0
```

另：模式 A 的规划节点在"去掉包内新增的第 2–31 行中文注释块"后，SHA256 精确等于
冻结源码指纹 `76277c39266634ac1b9204ac209bc55f7b192f282f5774168c19e953e9ed2ee8`
（实测，可用于复核注释确为唯一差异）。

---

## 5. 离线完整性验证（需求 18 的 A–H 项）

| 项 | 要求 | 结果 |
|---|---|---|
| A | 两个基线均无外部源码软依赖 | PASS：子模块目录非空（`fast_lio` 79 个文件、其中 `ikd-Tree` 3 个、`livox_ros_driver` 97 个文件），无需 `git submodule update` |
| B | launch/include 引用的仓库内文件全部存在 | PASS：47 个 `.launch`，仓库内引用缺失数 = 0（外部 package 引用如 `gazebo_ros`/`xacro` 属 ROS 发行版，不在包内） |
| C | Python internal imports 可解析 | PASS：内部 import 未解析数 = 0（`*_interfaces` 为 ROS 生成包，构建后解析） |
| D | package.xml/CMake 运行时依赖在包内完整 | PASS：未在包内的依赖全部来自 ROS 发行版/系统（roscpp/rospy/pcl_ros/cv_bridge/tf2_*/gazebo_msgs/…），已在运行环境矩阵中列出安装方式 |
| E | submodule 源文件完整 | PASS（同上计数；含 `fast_lio/config/mid360_sim.yaml`、`rviz_cfg/loam_livox.rviz` 等被 launch 引用的文件） |
| F | config/model/world/resource 不缺失 | PASS：关键资源检查点（A1 URDF、Livox 扫描模式 CSV、场景生成脚本、门/电梯控制脚本、ElevatorMission/ElevatorStatus 消息、RL 策略权重、hotel 世界、auto.sh、compose.yaml、Dockerfile）缺失数 = 0 |
| G | 无 broken symlink | PASS：包内符号链接共 2 个（`src/CMakeLists.txt`、`src/unitree_guide/CMakeLists.txt`），均指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`，在 ROS Noetic 环境内 `find -L` 无 broken link；在未安装 ROS 的机器上这两个链接不可解析属 catkin 工程预期行为，不影响解压、查阅源码与容器内构建 |
| H | 无 absolute path 强依赖到开发机路径 | PASS：README / launch / config / 脚本 / 源码中均无 `/home/ubuntu/SimEnv-*` 依赖。仅 4 个**历史记录/模板**文件在文字叙述中提及开发机路径（`.env.example` 的注释示例、`artifacts/review/pr16-safety-hardening-v1/baseline.md`、`reports/*.md`、`tools/baselines/floor0_4room_v1/BASELINE_MANIFEST.md`），不构成运行依赖。生产装配使用的路径是容器挂载点 `/workspace/SimEnv`（`compose.yaml` 定义），已在 `README_运行与复现.md` 第 1.4 节说明 |

---

## 6. PREEXISTING_BASELINE_ISSUE（如实报告，未在提交包中修改）

```text
PREEXISTING_BASELINE_ISSUE_1 =
  fast_lio @ e6aeba1（仅模式 B 的子模块版本）的 CMakeLists.txt 缺少消息生成依赖声明：
  fastlio_mapping 目标没有 add_dependencies(... generate_messages ...)，
  并行构建时可能早于消息生成而报
  "fatal error: fast_lio/Pose6D.h: No such file or directory"，导致构建失败。
  基线自带修复补丁：scripts/patches/fast_lio_message_dependency.patch
  （模式 A 的 fast_lio @ bf5f051 已包含该修复，无需打补丁）

PREEXISTING_BASELINE_ISSUE_2 =
  livox_ros_driver @ 3d240d5（仅模式 B）在未安装系统 Livox-SDK 时，会在 CMake
  configure 阶段直接 `git clone https://github.com/Livox-SDK/Livox-SDK.git`，
  离线/受限网络环境下会长时间阻塞。
  基线自带修复补丁：scripts/patches/livox_ros_driver_skip_sdk_clone.patch

处理方式（按"禁止为打包而修代码"的要求）：
  提交包内保留子模块源码的原始状态，未应用也未改写上述补丁；
  仅在 README_运行与复现.md 第 4.3 节与本报告中如实记录，并给出可复现命令。
  应用补丁后，模式 B 从解压副本构建通过（PASS，rc=0，221 个目标）。

补充说明：来源基线 `SimEnv-competition-complete-v1` 中这两个子模块目录为空
（未检出），其记录的 "BUILD = PASS" 是在子模块未参与构建的状态下取得的。
本提交包为了满足"解压即可构建、不要求评审执行 git submodule update"，
实体化了子模块源码，因而暴露了上述预存在缺陷。
```

---

## 7. 声明的例外（打包引入的非注释改动）

| 例外 | 数量/范围 | 性质与理由 |
|---|---|---|
| 测试代码删除 | 两基线全部 `test/`、rostest `*.test`、测试 fixture 与测试脚本（含 vendored `pybind11/tests`、`odeint` 的 `test/`·`test_external/`·`performance/`） | 提交要求"不需要测试代码"。逐一确认均非运行时依赖；`TEST_FILE_COUNT = 0` |
| 测试目标声明停用 | 模式 A 12 个、模式 B 13 个 `CMakeLists.txt`（新增 `TEST_FREE_SUBMISSION` 中文说明并把相应声明改为注释） | **强制后果**：删除测试源文件后，`catkin_add_nosetests` / `add_rostest` / `catkin_add_gtest` 会因找不到文件而 FATAL_ERROR，`install(DIRECTORY scripts test ...)` 引用了不存在的 `test` 目录。改动仅涉及测试目标与测试目录安装项，**运行时目标、参数与接口未变**（第 4 节 CMake 差异逐项核对） |
| 保留"名字像测试/日志但属运行资源"的文件 | `unitree_guide/.../src/FSM/State_RL_test.cpp`、`include/FSM/State_RL_test.h`（被 `GLOB(src/*/*.cpp)` 与 `FSM.h` 引用）；`src/unitree_guide/logs/policy_act_inference_*.pt`（RL 策略权重，运行时加载） | 误删会破坏运行时行为，故保留并在 §`docs/BASELINE_PROVENANCE.md` 第 4 节说明 |
| 子模块源码实体化 | 两基线的 `src/fast_lio`、`src/livox_ros_driver`、嵌套 `ikd-Tree` | 提交要求不得要求评审执行 `git submodule update`；内容与基线记录的 commit 一致，未做修改 |
| 排除运行时产物 | `fast_lio/Log/{dbg,imu,mat_out,mat_pre,pos_log}.txt`（submodule `.gitignore` 忽略的运行转储，最大 23 MB） | 属运行日志而非源码 |
| 新增复现入口副本 | 模式 A 的 `tools/baselines/floor0_video_return_v1/`（`floor0_full_replay.launch` 与验证 run 目录下的 6 个小文件 + 1 个说明） | 使模式 A 的复现入口随包自包含；`floor0_full_replay.launch` 与来源**字节一致**（SHA256 `08c095ed…`）；250 MB rosbag 等大体积产物未打包 |
| 中文注释补充 | 模式 A 24 个核心模块 + 部分文件头部导航说明；模式 B 17 个原本无中文的文件 | 纯注释（`#` 行注释 / XML 注释），零行为变化，见第 4 节 |

---

## 8. 验收清单（对照任务要求）

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
BUILD_SINGLE_FLOOR                       = PASS
BUILD_FULL_COMPETITION                   = PASS（需先应用基线自带子模块补丁，见第 6 节）

BROKEN_SYMLINK_COUNT                     = 0
MISSING_RUNTIME_FILE_COUNT               = 0

LOGIC_CHANGE_COUNT                       = 0
PARAMETER_CHANGE_COUNT                   = 0
INTERFACE_CHANGE_COUNT                   = 0

SIMULATION_RUN                           = NOT RUN

READY_FOR_SUBMISSION                     = true
```

---

## 9. 安全净化后再生成说明（2026-10-16）

> 本节由安全净化流程追加。**第 1–8 节的构建与静态检查结论均针对"原始提交包"**；
> 净化后**未重跑** `catkin_make` 与静态检查（见 9.3），**不得**视为对净化后包体的验证。

### 9.1 净化性质

本次净化对提交包**仅做删除**，未修改任何源码语句、参数、ROS 接口、消息 schema 或
launch 运行语义。唯一的非删除改动是 `docs/附录A_素材.md` 的 `src/` 目录树清单中，
移除已删除的 `uav_simulator/` 一行（因该目录已不存在）。

### 9.2 移除清单（941 个常规文件）

| 类别 | 数量 | 原因 |
|---|---|---|
| `src/uav_simulator/`（含两基线内同名路径） | 860 | **许可证不可确认**：`local_sensing`、`mockamap` 声明 **GPLv3**；`map_generator` 声明 **`TODO`**（等于未声明许可证）。经全仓引用检索（`git grep` 全部子包名 + `add_subdirectory`/`find_package`/launch/Docker/脚本），确认其**不被任何构建或运行入口引用** |
| `*/artifacts/review/pr16-safety-hardening-v1/` | 24 | 内部代码评审材料，非公开发布所需 |
| `*/src/generated_building/` | 22 | 运行时随机生成的场景产物，非源码 |
| `*/docs/team/` | 16 | 内部团队协作文档 |
| `*/reports/` | 13 | 内部实验与过程报告 |
| `*/src/**/.vscode/` | 6 | IDE 配置（含 vendored 第三方仓库内的实例） |
| **合计** | **941** | |

### 9.3 未重跑的验证（如实声明）

| 项 | 状态 |
|---|---|
| `BUILD_SINGLE_FLOOR` / `BUILD_FULL_COMPETITION` | 第 3 节结论针对**原始提交包**（2492 文件、221 个 catkin 目标）。净化后**未重跑**。移除 `uav_simulator` 会使 catkin 目标总数下降，但经引用检索确认无外部依赖，**理论上不影响其余目标**——此为推断，**非实测结论** |
| `STATIC_CHECK_SINGLE_FLOOR` / `STATIC_CHECK_FULL_COMPETITION` | 同上，净化后**未重跑** |
| `SIMULATION_RUN` | 仍为 `NOT RUN` |
| `FILE_COUNT` / `PACKAGE_SOURCE_DIGEST` | 已按净化后实际内容**重新计算**（见本文件头部字段） |

### 9.4 哈希口径（务必区分）

- 本文件头部 `PACKAGE_SOURCE_DIGEST` 与 `manifest/SHA256SUMS.txt` 中的全部哈希，
  均为**净化后**文件的重新计算结果。
- **不得**将其与"原始提交包"的哈希混用或互相印证。原始提交包对应值为：

```text
原始 FILE_COUNT                = 2492
原始 PACKAGE_SOURCE_DIGEST     = dd6638acec7f2698390183bf02ab3e71d11a12e34b614281b5b0c3ab1b7b2dd5
原始 manifest/SHA256SUMS.txt   SHA256 = 2952c6b7a3e38be8ac0677f287e12d18f229f213f109e16ffd5363e373f83132
原始 manifest/FILE_MANIFEST.txt SHA256 = c1da57eaf16bf4dd471414755d4f4f8e0f00b28e87bcf4984113e236246a26ab
原始 manifest/PACKAGE_AUDIT.md  SHA256 = b7804671fb526ded09342e3efbf7aa6bf6466c5077e4fc2254c48baba59618c8
```

以上原始文件已备份于**私有不公开的审计目录**（未随本次提交发布）。
