# 基线来源与可追溯性（BASELINE_PROVENANCE）

本文件记录提交包内两个基线的**准确来源、冻结指纹、验证证据与打包动作**，
用于评审与后续技术报告的可追溯性核对。

---

## 1. 基线 A：单层完整闭环（`baseline_single_floor/`）

### 1.1 来源

```text
SOURCE_WORKTREE   = /home/ubuntu/SimEnv-floor0-video-return-v1
BRANCH            = video/huangym/floor0-double-pass-v1
BASE_COMMIT       = 3503f44e617385c56f264875575d8ca8b68c3b36
                    （fix(building-generator): fully clear main entrance when open）
工作区状态         = base commit + 1 个未提交的 planner 修改
                    （src/competition_planning/scripts/planning_node）
                    + 9 个 __pycache__/*.pyc（打包时已排除）
本任务对原 worktree = 只读，未做任何修改
```

### 1.2 冻结源码指纹（来源侧）

```text
PLANNER_SOURCE          = src/competition_planning/scripts/planning_node
                          编辑器本地历史快照 `4U1v`（2026-09-14 03:18:49 保存）
SOURCE_SHA256           = 76277c39266634ac1b9204ac209bc55f7b192f282f5774168c19e953e9ed2ee8
SOURCE_LINES            = 4058
SUBMODULE  fast_lio     = bf5f05130f693fd9e32a1913d9abbb65f84a07f2
SUBMODULE  livox        = 24cf049379cd32f96b673d62766df4cd5bcd0cf8 （v2.6.0-16）
SUBMODULE  ikd-Tree     = e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
                        （src/fast_lio/include/ikd-Tree）
权威 provenance 文档    = artifacts/floor0-video-return/SUCCESS_RUNS_SUMMARY.md
                          （原 worktree 内；未随包打包，结论已转写至本文件）
```

### 1.3 提交包内对应文件指纹（含本次补充的中文注释）

```text
PACKAGE_SHA256          = 1a5fef9b6e14ff8da2d7b361cb2f4ec826041a3b6a2b8313144b1b1b54fe6079
PACKAGE_LINES           = 4088（= 4058 行源码 + 30 行中文模块说明注释）
差异性质                 = 仅在文件头部新增 `#` 注释块，不含任何语句变化
零行为变化证明            = Python AST 归一化比较：AST_LOGIC_IDENTICAL = true
                          （见 manifest/PACKAGE_AUDIT.md 第 4 节）
```

> 说明：由于新增了中文注释，包内文件的**字节哈希**与来源侧冻结哈希不同。
> 若需核对"这就是双 PASS 那版算法"，请以 **AST 等价性 + 本文件记录**为准；
> 若需字节级一致文件，可从本包中删除 `planning_node` 头部
> `# ==== 模块说明（中文） ...` 注释块（第 2–31 行）后校验，结果应为
> `76277c39...`。

### 1.4 已验证成功运行（3 次完整闭环）

证据来源：原 worktree 的
`artifacts/floor0-video-return/SUCCESS_RUNS_SUMMARY.md` 与各 run 目录。

| RUN ID | 结果 | 闭环耗时 | `ARRIVED_START` 距离 | 接触标记 |
|---|---|---|---|---|
| `20260914-101516` | PASS | 828.80 s | 0.215 m | 3（自行恢复） |
| `20260914-121632` | PASS | 854.10 s | 0.093 m | 6（自行恢复） |
| `20260914-153840` | PASS | **783.50 s** | **0.090 m** | 2 |
| `20260914-111857` | 作废（返程楔住，非 harness 故障） | — | 未到达 | — |

共同关键指标：

```text
HAZARDS                     = 6/6（唯一真值源匹配，1.0 m 口径）
FALSE_ALARMS                = 0
MAX_TIME_REACHED            = false（max_time = 1800 s 验证看门狗）
SKIPS = 0   UNREACHABLE = 0   PERMANENT_HOLD = no
GROUND_TRUTH_FEEDBACK_COUNT = 0
MANUAL_INTERVENTION_COUNT   = 0
状态序列                     = CURRENT_FLOOR_COMPLETE → RETURNING → ARRIVED_START → FINISHED
```

运行条件（冻结）：容器 `simenv-floor0-video-return-v1`（`--network host`、
`--gpus all`、`DISPLAY=:1`、挂载 worktree），ROS 11511 / Gazebo 11545，
`SEED=77 FLOOR_COUNT=1 ROOMS_PER_FLOOR=4`，
`ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 WRITE_GENERATED_TRUTH_COPY=0`，
`GUI=true`，`max_time=1800 s`（验证看门狗；正式预算 600 s 保持不变）。

### 1.5 随包的复现入口（验证运行实际使用的文件副本）

位置：`baseline_single_floor/tools/baselines/floor0_video_return_v1/`

| 文件 | 来源（原 worktree 内路径） | 说明 |
|---|---|---|
| `floor0_full_replay.launch` | `artifacts/floor0-video-return/20260914-153840/floor0_full_replay.launch` | **字节一致**，SHA256 `08c095eda1b891a670fa215d79e623f2dc1961b28657f2a2c8b8c1571a3032bb` |
| `start_sim.sh` | 同上 run 目录 | 环境启动脚本副本（冻结环境变量 + `auto.sh`） |
| `run_mission.sh` | 同上 run 目录 | 验证运行编排脚本副本（readiness / RGB-D gate / rosbag / watcher） |
| `watcher.py` | 同上 run 目录 | 10 s 采样观察器（可选观测工具） |
| `VALIDATED_RUN_RESULT.md` | 同上 run 目录 `RUN_RESULT.md` | 该次 PASS 的结果记录（原始文本） |
| `VALIDATED_RUN_PARAMS.txt` | 同上 run 目录 `params.txt` | 运行中实时回读的 ROS 参数 |
| `VALIDATED_RUN_GT_COMPLIANCE.txt` | 同上 run 目录 `gt_compliance.txt` | 真值合规审计输出 |

未随包内容（按要求排除）：该 run 目录内的 `run.bag`（约 250 MB）、
`competition_stack.log`（约 870 KB）等大体积运行产物。

> `run_mission.sh` 内部硬编码了验证时的 `RUN_DIR` 路径
> （`/workspace/SimEnv/artifacts/floor0-video-return/20260914-153840`）。
> 复现时请按 `README_运行与复现.md` 第 3 节手工执行两条启动命令，或自行把
> `RUN_DIR` 指向该脚本所在目录后使用。

---

## 2. 基线 B：真实比赛完整流程（`baseline_full_competition/`）

### 2.1 来源

```text
SOURCE_WORKTREE   = /home/ubuntu/SimEnv-competition-complete-v1
BRANCH            = feat/huangym/competition-complete-v1
HEAD              = ba662d2ef233250b4c3d79aaa44616ceea37afbc
工作区状态         = 干净（git status --short 无输出）
本任务对原 worktree = 只读，未做任何修改
```

### 2.2 状态（来源报告，如实转写）

来源：`reports/competition_code_completeness_v1.md`、
`reports/competition_submission_readability_v1.md`、`README_COMPETITION_CN.md`

```text
COMPETITION_CODE_COMPLETE            = true
COMPETITION_SUBMISSION_READABILITY   = PASS
BUILD                                = PASS（来源工作区容器内 catkin_make）
UNIT_TESTS                           = 380/380 PASS（0 failure / 0 error）
STATIC / INTERFACE AUDIT             = PASS（9 条多楼层接口发布/订阅双向闭环）
GT_COMPLIANCE                        = PASS（生产文件无真值/隐藏文件引用）
PLACEHOLDER_COUNT                    = 0 关键占位
END_TO_END_MULTI_FLOOR_GAZEBO        = NOT YET VALIDATED（来源报告 SIMULATION_RUN = NOT RUN BY REQUEST）
```

来源报告给出的单元测试分布：`competition_bringup 7`、`competition_control 40`、
`competition_localization 59`、`competition_loop 158`、`competition_mapping 49`、
`competition_planning 67`（合计 380）。

### 2.3 子模块来源（重要）

来源工作区中 `src/fast_lio` 与 `src/livox_ros_driver` 为**未检出的 git 链接**
（目录为空），因此：

```text
BASELINE_B_RECORDED_SUBMODULES = src/fast_lio         e6aeba10869c2f9f51edb2db2124472bbc569f3c
                                 src/livox_ros_driver 3d240d5666129e1a3052e78ee8487a04b08fdda3
                                 （nested）src/fast_lio/include/ikd-Tree
                                                       e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
MATERIALIZED_FROM              = /home/ubuntu/SimEnv（同一仓库，检出到上述 commit，
                                 子模块工作区干净无未提交改动）
PACKAGE 内子模块目录            = 已实体化（评审无需执行 git submodule update）
```

包内子模块源码仅为满足"解压即可构建"的要求而实体化；**未对其内容做任何修改**
（未应用任何补丁）。其裸状态构建缺陷见第 4 节 `PREEXISTING_BASELINE_ISSUE`。

### 2.4 提交包内对应文件指纹

```text
src/competition_planning/scripts/planning_node  SHA256 = 5635639ad45ff59221321e7df7b10e0fef3924039ccb82b35e80fe514aba3a46
                                                （与来源 worktree 字节一致，未加注释）
README_COMPETITION_CN.md                        SHA256 = 4479d95b5d6743ccf641f6e3f05949227bbfa465b067acc117ba3a9a06de22b7
```

---

## 3. 提交包中"中文注释"的性质声明

```text
CHINESE_COMMENT_CHANGE_TYPE = comment-only presentation change
```

- 基线 A：为 24 个核心模块（定位 / 建图 / 探索 / 视觉覆盖 / 规划 / 跟随 /
  安全网关 / 任务状态机 / 危险源识别 / 去重 / 结果写出 / 启动装配等）补充
  中文模块级说明块，并在部分文件的头部说明块内列出"核心类/函数/复杂算法块"
  的导航（含行号与设计原因）。
- 基线 B：来源已具备较完整中文注释，仅对**完全没有中文注释**的 17 个文件
  补充简要中文模块说明，未对既有中文注释做改写或堆砌。
- 所有注释均以 `#` 行注释或 XML 注释形式加入，**不修改任何表达式、条件、
  常量、调用、参数与接口**；零行为变化证明见
  `manifest/PACKAGE_AUDIT.md` 第 4 节。

---

## 4. 打包动作与例外（全部已声明）

| 动作 | 范围 | 原因与影响 |
|---|---|---|
| 删除测试代码 | 两个基线的 `test/` 目录、`*.test`（rostest）、测试专用 fixture/数据、`test_*.py`、`*_test.cpp`、`test_dynamics/`、`pybind11/tests/`、`odeint` 的 `test/`·`test_external/`·`performance/` | 提交要求"不需要测试代码"；已逐一确认非运行时依赖 |
| 保留"名字像测试但属运行资源"的文件 | `src/unitree_guide/.../src/FSM/State_RL_test.cpp` 与 `include/FSM/State_RL_test.h` | 被 `junior_ctrl` 的 `GLOB(src/*/*.cpp)` 收集且被 `FSM.h` 引用，属运行所需 FSM 状态 |
| 保留"名字像日志但属运行资源"的目录 | `src/unitree_guide/logs/policy_act_inference_*.pt` | RL 策略权重，运行时被加载（`State_RL_test.cpp` 第 777 行） |
| 停用测试目标声明 | 模式 A 12 个、模式 B 13 个 `CMakeLists.txt` | 测试源文件删除后，`catkin_add_nosetests` / `add_rostest` / `catkin_add_gtest` / `install(DIRECTORY ... test ...)` 会导致 `catkin_make` 直接 FATAL_ERROR；只停用测试目标，运行时可执行目标未变 |
| 实体化子模块源码 | 两个基线的 `src/fast_lio`、`src/livox_ros_driver`（含嵌套 `ikd-Tree`） | 提交要求不得要求评审执行 `git submodule update` |
| 排除运行产物 | `fast_lio/Log/{dbg,imu,mat_out,mat_pre,pos_log}.txt`（被 submodule 的 `.gitignore` 忽略的运行时转储，最大 23 MB） | 属运行日志，非源码 |
| 排除开发垃圾 | `.git/`、`build/`、`devel/`、`logs/`、`.pytest_cache/`、`__pycache__/`、`*.pyc`、`*.bag`、运行录像/截图、本地 `.env` | 提交要求 |
| 逆移植 provenance 素材 | 上述 1.5 节的 7 个小文件（来自验证 run 目录） | 使模式 A 的复现入口随包自包含；大体积运行产物未打包 |

---

## 5. 可复核命令（全部自包含，不需要任何开发机路径）

```bash
cd DG-202602-core-code-submission-v1

# 1) 校验包内文件完整性与哈希
sha256sum -c manifest/SHA256SUMS.txt | tail -5

# 2) 复现"包内规划节点 = 冻结 4U1v 源码 + 30 行中文注释"（实测成立）
python3 - <<'PY'
import hashlib
p = "baseline_single_floor/src/competition_planning/scripts/planning_node"
lines = open(p, encoding="utf-8").read().split("\n")
core = "\n".join([lines[0]] + lines[31:])          # 去掉第 2–31 行中文注释块
print("stripped_sha256 =", hashlib.sha256(core.encode()).hexdigest())
print("expected        = 76277c39266634ac1b9204ac209bc55f7b192f282f5774168c19e953e9ed2ee8")
PY

# 3) 校验中文模块说明存在于各核心模块（抽样）
grep -l "模块说明（中文）" baseline_single_floor/src/competition_*/scripts/* \
      baseline_single_floor/src/competition_loop/src/*.py 2>/dev/null | wc -l
```

"注释零行为变化"的完整方法与统计结果（Python AST 归一化、XML 元素树、
C/C++ token、CMake/YAML/Shell 去注释比较）见 `manifest/PACKAGE_AUDIT.md` 第 4 节。
