# competition_perception

**负责人**: 鲍语菲（环境感知与危险源识别）

## 模块目标

使用 RealSense RGB-D 数据检测室内环境中的红色球体（危险源），
并与红色方块和绿色球体（干扰源）区分。输出三维候选位置到深度相机坐标系
（由 `header.frame_id` 指明），由 TF/定位链转换为 world 坐标后供
result_writer 使用。

## 输入

| 输入 | 类型 | 说明 |
|------|------|------|
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB 相机标定 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 深度图像（16UC1 毫米 / 32FC1 米） |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机标定 |

四个话题经 `ApproximateTimeSynchronizer`（slop=0.20 s）同步后成对处理。
本节点不订阅 `/real_sense/depth/points`，也不使用任何真值话题。

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/hazard_candidates` | `HazardCandidateArray` | competition_loop / result_writer | 危险源候选，帧为深度相机 optical frame |
| `/competition/perception/status` | `ModuleStatus` | competition_loop | 感知模块状态（latched） |

候选 `header.frame_id` 为深度 CameraInfo 的 `frame_id`（仿真中为
`real_sense_optical_frame`）。转换到 `world` 由下游 TF 链完成；
result_writer 只接受 `frame_id="world"` 且 `position_valid=true` 的候选。

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~rgb_topic` | string | `/real_sense/rgb/image_raw` | RGB 图像话题 |
| `~rgb_info_topic` | string | `/real_sense/rgb/camera_info` | RGB CameraInfo 话题 |
| `~depth_topic` | string | `/real_sense/depth/image_raw` | 深度图像话题 |
| `~depth_info_topic` | string | `/real_sense/depth/camera_info` | 深度 CameraInfo 话题 |
| `~min_contour_area` | float | 150.0 | 最小轮廓面积（像素） |
| `~min_depth` | float | 0.4 | 有效深度下限（米） |
| `~max_depth` | float | 8.0 | 有效深度上限（米） |
| `~input_timeout_sec` | float | 1.0 | 输入超时阈值（秒） |
| `~watchdog_period_sec` | float | 0.1 | watchdog 检查周期（秒） |
| `~future_stamp_tolerance_sec` | float | 0.5 | 未来时间戳容忍（秒） |
| `~max_input_age_sec` | float | 1.0 | 观测最大允许年龄（秒） |

## 状态语义（契约 v0.2.0）

| state | valid | 含义 |
|-------|-------|------|
| STARTING | false | 尚未接受任何观测 |
| RUNNING | true | 已接受本 generation 内的 fresh RGB-D，输出可用（可能没有候选） |
| TIMEOUT | false | 输入超时或 ROS `/clock` 回退；此前缓存/锁存输出立即失效 |

`ModuleStatus.valid` 是权威可用性门控，`state` 仅用于解释原因。

- 没有检测到候选时，模块仍保持 `RUNNING / valid=true`（模块工作正常，
  只是当前画面没有危险源）。
- 单个坏帧（时间戳异常、转换失败、CameraInfo 无效）只被跳过并记录日志，
  不会使模块失效——失效统一由 watchdog 超时或时钟回退触发，
  避免 transient 错误导致下游反复失效/恢复。
- **输入超时与 ROS `/clock` 回退都是 generation 边界切换**：进入
  timeout 的瞬间（第一次 watchdog tick）记录 `generation_start_stamp`，
  时间戳早于该边界、但年龄仍小于 `~max_input_age_sec` 的 delayed 帧
  属于旧 generation，一律拒绝；重复 tick 不会重复创建 generation。
- 超时后恢复必须满足：观测属于当前 generation（时间戳 ≥ 边界）、
  严格大于已接受的最新观测、且年龄不超过 `~max_input_age_sec`。
- ROS `/clock` 回退：立即 `valid=false`、开启新 generation、清空旧
  freshness/monotonic 状态，并且**重建 synchronized subscriptions**
  （unregister 旧 Subscriber、丢弃旧 synchronizer 队列，保证旧时间线
  数据不会在新 generation 中被消费）；旧 generation 观测（回退前
  时间线）还会被 future 检查拒绝。
- image callback 在处理入口记录 generation snapshot；处理期间若发生
  timeout/rollback 迁移，该 in-flight 帧在 commit 阶段被丢弃。

## 检测流程

1. HSV 颜色空间提取红色（双区间）与绿色掩码；
2. 形态学开/闭运算去噪；
3. 提取轮廓，过滤过小面积；
4. 轮廓中心附近窗口取深度中值；
5. 用深度 CameraInfo 内参反投影到深度相机坐标系
   （ROS optical 约定：X 右、Y 下、Z 前）；
6. 分类：

| 标签 | 判据 |
|------|------|
| `green_sphere` | 绿色区域（干扰源） |
| `red_sphere` | 圆形度 ≥ 0.80 且最小外接圆填充率 ≥ 0.78 且宽高比 ∈ [0.75, 1.33]；有深度时还需物理直径 ∈ [0.20, 0.42] m |
| `red_box` | 其余红色区域（干扰源） |

7. 置信度为面积/形状/深度有效性的启发式加权，范围 [0, 1]。

候选的 `position_valid=false` 只表示该候选三维位置不可信（例如深度
缺失），不影响模块整体 `valid`。

## 关键假设与限制

- **RGB/深度对齐**：当前实现按分辨率比例把 RGB 像素映射到深度图。
  竞赛仿真中 RealSense 由单个 `gazebo_ros_openni_kinect` 传感器建模
  （`unitree_ros/robots/a1_description/xacro/gazebo.xacro`：一个 camera
  block、640x480、60° HFOV，RGB 与深度共享同一投影与 optical frame），
  因此两幅图按构造即已配准，比例映射严格成立（等分辨率时比例为 1）。
  **该假设不适用于真实未配准的 D435 硬件**。
- **单目 2D 形状歧义**：正对视角的红色方块与红色球体轮廓接近，依靠
  圆形度 + 填充率双重阈值区分；严重遮挡或极端视角下仍可能误分，
  属于已知限制，需要真实仿真定量验证。
- 未做多帧跟踪去重；同一目标可能被多次上报为候选。
- `position` 暂时处于传感器帧，由 `header.frame_id` 指明，本模块不
  负责 world 转换。

## 测试

纯 Python 单元测试（不依赖 ROS master / Gazebo）：

```bash
# 在包含 python3-opencv / numpy 的环境中：
python3 -m unittest discover -s src/competition_perception/test -v
```

或经 catkin：

```bash
catkin run_tests competition_perception
catkin_test_results
```

覆盖：fresh 观测接受、空场景、无深度时 `position_valid=false`、超时
失效与 generation 边界、超时后 delayed/stale/边界前帧不恢复、边界后
fresh 帧恢复、时钟回退新 generation、重复/乱序时间戳拒绝、红球/红盒/
绿球合成图分类、参数非法 fail-fast、node wrapper 级并发/订阅重建行为
（fake ROS layer，无需 Gazebo / roscore）。

## 禁止事项

- 不得根据危险源模型名称（`danger_red_sphere_*`）识别源
- 不得调用 Gazebo get_model_state 获取源真值位置
- 不得读取 `danger_truth.json` 获取源列表
- 不得订阅 `/ground_truth/*`
- 不得订阅 `/Odometry_gazebo`

## 最小运行命令

```bash
source devel/setup.bash
rosrun competition_perception perception_node
```

## 验收标准

- [x] 启动后先发布 `valid=false` 的状态
- [x] 不订阅禁止话题
- [x] 不读取禁止文件
- [x] 能区分红色球体、红色方块、绿色球体（合成图单元测试覆盖；
      真实仿真效果待联调验证）
- [x] 输入超时 → `TIMEOUT / valid=false`；fresh 当前 generation 观测可恢复
- [x] ROS `/clock` 回退 → 失效 + 新 generation
- [ ] 三维位置误差（相机帧）< 0.2 m @ 3 m 距离（待真实仿真验证）
- [ ] recall > 0.8（不遗漏危险源）（待真实仿真验证）
- [ ] 虚警率可控（误报干扰源的比例低）（待真实仿真验证）

## 需要向上下游成员交付的内容

- 黄誉铭（loop）：`/competition/hazard_candidates`（帧为深度相机
  optical frame，最终需由 TF 链转换为 world 帧）
- 马跃（mapping）：可选提供检测到的物体位置用于语义建图
- 纪雯瀚（localization）：依赖 TF 进行坐标转换
