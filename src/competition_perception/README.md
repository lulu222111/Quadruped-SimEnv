# competition_perception

**负责人**: 鲍语菲（环境感知与危险源识别）

## 模块目标

使用 RealSense RGB-D 数据检测室内环境中的红色球体（危险源），
并与红色方块和绿色球体（干扰源）区分。输出三维候选位置到传感器坐标系，
由 TF/定位链转换为 world 坐标后供 result_writer 使用。

## 输入

| 输入 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | 10 Hz | RGB 图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | RGB 相机标定 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 10 Hz | 深度图像 |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | 深度相机标定 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 10 Hz | 深度点云 |

## 输出

| 输出 | 类型 | 消费者 | 说明 |
|------|------|--------|------|
| `/competition/hazard_candidates` | `HazardCandidateArray` | competition_loop | 危险源候选（传感器帧） |
| `/competition/perception/status` | `ModuleStatus` | competition_loop | 感知模块状态 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~red_hue_min` | int | 0 | 红色色相下限（HSV） |
| `~red_hue_max` | int | 10 | 红色色相上限（HSV） |
| `~min_confidence` | float | 0.5 | 最小置信度阈值 |
| `~publish_debug_image` | bool | false | 是否发布调试图像 |

## 当前状态

**v0.1.0 — 骨架**

- [x] 占位节点只发布 `ModuleStatus(state=STARTING, valid=false)`
- [x] 不发布伪造危险源候选
- [ ] RGB 颜色分割（红色提取）
- [ ] 深度点云聚类
- [ ] 球体/方块形状分类（半径 ~0.15m sphere vs ~0.30m box）
- [ ] 传感器坐标系到 world 坐标系转换
- [ ] 多帧跟踪去重

**默认安全行为**：
- 不发布候选检测（没有 HazardCandidateArray 消息）

## 待实现 TODO

1. **颜色分割**：HSV 阈值或深度学习方法提取红色区域
2. **形状判断**：对聚类后的点云拟合几何体（球体半径 vs 方块尺寸）
3. **深度估计**：从深度图/点云获取候选的三维位置
4. **坐标系**：输出 `frame_id="real_sense"`，由定位链转换为 `"world"`
5. **多帧关联**：避免同一物体因多帧检测被重复计入
6. **置信度估计**：基于检测一致性、点云密度、距离等因素

## 禁止事项

- 不得根据危险源模型名称（`danger_red_sphere_*`）识别源
- 不得调用 Gazebo get_model_state 获取源真值位置
- 不得读取 `danger_truth.json` 获取源列表
- 不得订阅 `/ground_truth/*`
- 不得在骨架模式下输出 `position_valid=true` 的候选

## 最小运行命令

```bash
source devel/setup.bash
rosrun competition_perception perception_node
```

## 验收标准

- [x] 启动后只发布 `valid=false` 的状态
- [x] 不订阅禁止话题
- [x] 不读取禁止文件
- [ ] 能正确区分红色球体、红色方块、绿色球体
- [ ] 三维位置误差（相机帧）< 0.2 m @ 3 m 距离
- [ ] recall > 0.8（不遗漏危险源）
- [ ] 虚警率可控（误报干扰源的比例低）

## 需要向上下游成员交付的内容

- 黄誉铭（loop）：`/competition/hazard_candidates`（最终需转换为 world 帧）
- 马跃（mapping）：可选提供检测到的物体位置用于语义建图
- 纪雯瀚（localization）：依赖 TF 进行坐标转换
