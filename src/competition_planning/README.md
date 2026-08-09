yanyx@ubuntu-System-Product-Name:~/SimEnv$ cd "$HOME/SimEnv"

sudo docker compose down

nl -ba \
  src/competition_planning/README.md
[+] down 1/1
 ✔ Container simenv-yanyx Removed                                                                                0.1s
     1  # competition_planning
     2
     3  **负责人**：闫雨萱
     4  **开发分支**：`feat/yanyx/planning`
     5
     6  ## 模块目标
     7
     8  根据占据栅格地图和机器人定位结果：
     9
    10  1. 提取未知区域与空闲区域交界处的 Frontier；
    11  2. 将相邻 Frontier 格子聚类并过滤小区域；
    12  3. 综合区域大小和距离选择探索目标；
    13  4. 对障碍物进行安全膨胀；
    14  5. 使用单源四邻域 BFS 一次构建可达性搜索树，复用至所有 Frontier 候选点，通过父节点表重建最短路径；
    15  6. 向控制模块发布导航目标和路径；
    16  7. 在输入失效、坐标系不一致或规划失败时撤销旧路径。
    17
    18  ## 输入
    19
    20  | 输入话题 | 类型 | 提供模块 | 说明 |
    21  |---|---|---|---|
    22  | `/competition/map` | `nav_msgs/OccupancyGrid` | mapping | 占据栅格地图 |
    23  | `/competition/localization/odom` | `nav_msgs/Odometry` | localization | 机器人当前位置 |
    24
    25  ### 坐标系约束
    26
    27  `/competition/map` 与 `/competition/localization/odom` 的
    28  `header.frame_id` 必须均为非空值且完全相同。
    29
    30  当前版本不执行 TF 坐标转换。若两个坐标系不一致，规划模块将：
    31
    32  - 发布 `ModuleStatus(state=FAILED, valid=false)`；
    33  - 发布空的 `/competition/path`，撤销此前锁存的路径；
    34  - 不发布新的导航目标；
    35  - 不继续进行路径规划。
    36
    37  ## 输出
    38
    39  | 输出话题 | 类型 | 使用模块 | 说明 |
    40  |---|---|---|---|
    41  | `/competition/navigation_goal` | `geometry_msgs/PoseStamped` | control | 当前 Frontier 导航目标 |
    42  | `/competition/path` | `nav_msgs/Path` | control | 四邻域最短路径 |
    43  | `/competition/planning/status` | `competition_interfaces/ModuleStatus` | competition_loop | 规划模块状态 |
    44
    45  ### 输出安全约定
    46
    47  - `/competition/navigation_goal` 不锁存，避免新启动的控制节点收到旧目标；
    48  - `/competition/path` 使用锁存，但规划失效时会发布空路径撤销旧路径；
    49  - 控制模块收到空路径或 `planning/status.valid=false` 时应停止路径跟踪；
    50  - 有效路径中的每个位姿均根据路径切线设置行进方向四元数；
    51  - 最后一个路径位姿沿用最后一段路径的方向。
    52
    53  ## ROS 参数
    54
    55  | 参数 | 类型 | 默认值 | 说明 |
    56  |---|---|---:|---|
    57  | `~frontier_min_size` | int | `5` | Frontier 聚类最少格子数 |
    58  | `~replan_interval` | float | `2.0` | 重规划周期，单位为秒 |
    59  | `~map_timeout` | float | `5.0` | 地图消息最长允许未更新时间，单位为秒 |
    60  | `~localization_timeout` | float | `5.0` | 定位消息最长允许未更新时间，单位为秒 |
    61  | `~inflation_radius` | float | `0.25` | 障碍物安全膨胀半径，单位为米 |
    62  | `~size_weight` | float | `1.0` | Frontier 区域大小评分权重 |
    63  | `~distance_weight` | float | `1.0` | 机器人到目标距离评分权重 |
    64  | `~obstacle_threshold` | int | `50` | 判定占用格子的阈值，合法范围为 `[1, 100]` |
    65
    66  ### 参数安全约束
    67
    68  节点启动时会检查：
    69
    70  - `frontier_min_size >= 1`；
    71  - `replan_interval > 0`；
    72  - `map_timeout > 0`；
    73  - `localization_timeout > 0`；
    74  - `inflation_radius >= 0`；
    75  - `size_weight >= 0`；
    76  - `distance_weight >= 0`；
    77  - `1 <= obstacle_threshold <= 100`；
    78  - 所有浮点参数必须是有限值，拒绝 `NaN` 和 `Inf`。
    79
    80  ## 当前已实现
    81
    82  - [x] 从 `OccupancyGrid` 提取 Frontier 格子
    83  - [x] 采用八邻域进行 Frontier 聚类
    84  - [x] 使用 `frontier_min_size` 过滤零散噪声
    85  - [x] 选择距离聚类中心最近的真实 Frontier 格子
    86  - [x] 综合区域大小和距离选择 Frontier 簇
    87  - [x] 同簇代表点不可达时继续尝试同簇其他候选点
    88  - [x] 整个 Frontier 簇不可达后尝试下一片 Frontier
    89  - [x] 地图格子坐标与世界坐标双向转换
    90  - [x] 支持带旋转的地图原点
    91  - [x] 单源四邻域BFS最短路径规划
    92  - [x] 单源 BFS 可达性搜索树在所有 Frontier 候选间复用
    93  - [x] 障碍物安全膨胀
    94  - [x] 膨胀覆盖机器人起点时，仅恢复原本空闲的起点格
    95  - [x] 禁止路径穿过未知区域和障碍物
    96  - [x] 接入地图和定位 ROS 话题
    97  - [x] 强制检查地图与定位坐标系一致
    98  - [x] 地图或定位输入超时后撤销旧路径并发布 `TIMEOUT`
    99  - [x] 发布导航目标、路径和模块状态
   100  - [x] 规划失效时发布空路径撤销旧锁存路径
   101  - [x] 导航目标不锁存
   102  - [x] 根据路径切线生成路径位姿方向四元数
   103  - [x] 区分“探索完成”和“暂时不可达”
   104  - [x] 启动时完整校验参数并拒绝 `NaN`、`Inf` 和非法范围
   105  - [x] 62 项无 ROS 依赖的自动测试通过
   106  - [x] ROS 自动集成测试通过
   107
   108  ## 规划状态
   109
   110  | 场景 | `ModuleStatus.state` | `valid` | 路径输出 |
   111  |---|---:|---:|---|
   112  | 等待地图或定位 | `STARTING` | `false` | 空路径 |
   113  | 正常生成路径 | `RUNNING` | `true` | 有效路径 |
   114  | 探索完成、没有 Frontier | `SUCCEEDED` | `false` | 空路径 |
   115  | 存在 Frontier 但均不可达 | `BLOCKED` | `false` | 空路径 |
   116  | 地图或定位输入过期 | `TIMEOUT` | `false` | 空路径 |
   117  | 参数、数据或坐标系错误 | `FAILED` | `false` | 空路径 |
   118
   119  ## 已完成验证
   120
   121  ### 无 ROS 单元测试
   122
   123  已验证：
   124
   125  - Frontier 提取、聚类和过滤；
   126  - Frontier 代表点及簇评分；
   127  - 同簇多候选点回退；
   128  - 地图格子坐标与世界坐标转换；
   129  - 带旋转地图原点；
   130  - 四邻域BFS最短路径；
   131  - 障碍物膨胀；
   132  - 机器人起点被膨胀覆盖时的安全恢复；
   133  - 探索完成与不可达状态区分；
   134  - 20×36 m 和 400×400 地图规划性能；
   135  - 路径方向计算；
   136  - 单源 BFS 可达性搜索在所有 Frontier 候选间复用；
   137  - 同簇代表点不可达时回退至同簇其他候选（含真实网格验证）；
   138  - 重建路径完全由合法空闲格组成；
   139  - 起点等于目标、起点越界、起点被障碍占用的边界情况。
   140
   141  当前预期结果：
   142
   143  ```text
   144  Ran 62 tests
   145  OK
   146  ```
   147
   148  ### ROS 自动集成测试
   149
   150  已验证：
   151
   152  - 节点能根据 mock 地图和定位生成有效路径；
   153  - 有效路径中的四元数与路径前进方向一致；
   154  - 坐标系不一致时发布 `FAILED` 和空路径；
   155  - Frontier 不可达时发布 `BLOCKED` 和空路径；
   156  - 输入超时后发布 `TIMEOUT` 和空路径；
   157  - 成功规划后再次失效时，旧锁存路径被清空；
   158  - 新订阅者不会收到过期的锁存导航目标。
   159
   160  当前预期结果：
   161
   162  ```text
   163  RESULT: SUCCESS
   164  TESTS: 1
   165  ERRORS: 0
   166  FAILURES: 0
   167  ```
   168
   169  ### Docker 与 ROS 环境
   170
   171  已在个人 Docker 环境中完成：
   172
   173  - ROS Noetic 环境检查；
   174  - `catkin_make` 编译；
   175  - `rospack find competition_planning` 检查；
   176  - 62 项单元测试；
   177  - ROS 自动集成测试；
   178  - headless Gazebo 启动检查；
   179  - mock 地图与定位话题验证；
   180  - 导航目标、路径和模块状态输出检查。
   181
   182  ## 自动测试
   183
   184  在仓库根目录执行：
   185
   186  ```bash
   187  python3 -m unittest discover \
   188    -s src/competition_planning/test \
   189    -p 'test_*.py' \
   190    -v
   191  ```
   192
   193  ## ROS 编译与运行
   194
   195  在团队统一的 ROS Noetic Docker 环境中执行：
   196
   197  ```bash
   198  source /opt/ros/noetic/setup.bash
   199  cd /workspace/SimEnv
   200  catkin_make -j
   201  source devel/setup.bash
   202  rosrun competition_planning planning_node
   203  ```
   204
   205  ## ROS 自动集成测试
   206
   207  ```bash
   208  source /opt/ros/noetic/setup.bash
   209  cd /workspace/SimEnv
   210  catkin_make -j
   211  source devel/setup.bash
   212  rostest competition_planning planning_node_ros.test --text
   213  ```
   214
   215  ## 尚待团队联调
   216
   217  - [ ] 使用 mapping 模块真实发布的 `/competition/map` 联调
   218  - [ ] 使用 localization 模块真实发布的 `/competition/localization/odom` 联调
   219  - [ ] 与 control 模块确认空路径和 `valid=false` 时立即停止跟踪
   220  - [ ] 验证 control 模块是否使用路径姿态
   221  - [ ] 在真实 Gazebo 地图中验证动态重规划
   222  - [ ] 验证20×36 m 场景中的可达房间覆盖率
   223  - [ ] 完善目标到达和重复目标抑制
   224  - [ ] 实现多楼层、电梯和返回起点策略
   225
   226  ## 禁止事项
   227
   228  规划模块不得：
   229
   230  - 读取 `layout_metadata.json` 获取房间或目标位置；
   231  - 读取 `danger_truth.json` 获取危险源位置；
   232  - 订阅 `/ground_truth/*`；
   233  - 使用裁判真值作为规划输入；
   234  - 在无效状态下继续保留旧路径。
   235
   236  ## 团队协作接口
   237
   238  - 马跃（mapping）：提供 `/competition/map`
   239  - 纪雯瀚（localization）：提供 `/competition/localization/odom`
   240  - 杨渊浩（control）：使用 `/competition/navigation_goal` 和 `/competition/path`
   241  - competition_loop：使用 `/competition/planning/status` |
| 正常生成可执行路径 | `RUNNING` | `true` | 非空 Path |
| 探索完成、没有 Frontier | `SUCCEEDED` | `false` | 空 Path |
| 上游 invalid 或 Frontier 不可达 | `BLOCKED` | `false` | 空 Path |
| status、Map 或 Odometry 超时 | `TIMEOUT` | `false` | 空 Path |
| frame、参数、数据或时间戳错误 | `FAILED` | `false` | 空 Path |

## 已完成验证

### 无 ROS 单元测试

已覆盖 Frontier、坐标转换、单源 BFS、候选回退、障碍膨胀、状态区分、路径方向以及 20×36 m 和 400×400 地图性能。

```text
Ran 62 tests
OK
```

运行命令：

```bash
python3 -m unittest discover \
  -s src/competition_planning/test \
  -p 'test_*.py' \
  -v
```

### ROS 契约集成测试

已验证：

- 节点订阅四个正式输入；
- 缺少上游 status 时禁止规划；
- 合法 `world/base` 输入生成非空 Path；
- Mapping invalid 时立即发布空 Path；
- status 恢复不能自动复活旧 generation 数据；
- 收到新 Map 和 Odometry 后恢复规划；
- 非 `world/base` 输入被拒绝；
- 上游 status 超时后发布 `TIMEOUT` 和空 Path。

```text
RESULT: SUCCESS
TESTS: 1
ERRORS: 0
FAILURES: 0
```

运行命令：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
catkin_make -j
source devel/setup.bash
rostest competition_planning planning_node_ros.test --text
```

### Docker 与 ROS 环境

已在个人 Docker 环境中完成：

- ROS Noetic `catkin_make` 编译；
- 62 项无 ROS 单元测试；
- ROS 契约集成测试；
- headless Gazebo、Mock Map/Odometry 与输出话题基础验证。

## 尚待团队联调

- [ ] 与真实 Mapping 的 Map、status 发布频率及延迟联调
- [ ] 与真实 Localization 的 Odometry、status、`world -> base` 联调
- [ ] 根据真实频率调整四项 timeout 默认值
- [ ] 与 Control 验证 Path 替换、空 Path 和 `valid=false` 停车行为
- [ ] 验证动态重规划、定位跳变和短时丢失
- [ ] 验证 ROS `/clock` 真实回退场景
- [ ] 验证 20×36 m 场景可达房间覆盖率
- [ ] 完善目标到达与重复目标抑制
- [ ] 实现多楼层、电梯和返回起点策略

## 禁止事项

规划模块不得：

- 读取 `layout_metadata.json` 获取房间或目标位置；
- 读取 `danger_truth.json` 获取危险源位置；
- 订阅 `/ground_truth/*`；
- 使用裁判真值作为规划输入；
- 在上游或自身无效时继续保留旧 Path。

测试 Mock 只能放在 `competition_testing`、Planning test 或测试 launch 中，不得进入正式算法节点或正式比赛 launch。

## 团队协作接口

- Mapping：提供 `/competition/map` 和 `/competition/mapping/status`
- Localization：提供 `/competition/localization/odom` 和 `/competition/localization/status`
- Control：使用 `/competition/navigation_goal`、`/competition/path` 和 `/competition/planning/status`
- competition_loop：使用 `/competition/planning/status`
