#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
perception_node — hazard source detection using RealSense RGB-D.

允许订阅的 RealSense 话题：
  /real_sense/rgb/image_raw
      sensor_msgs/Image
  /real_sense/rgb/camera_info
      sensor_msgs/CameraInfo
  /real_sense/depth/image_raw
      sensor_msgs/Image
  /real_sense/depth/camera_info
      sensor_msgs/CameraInfo


处理流程：

  1. 同步接收 RGB 图像、RGB CameraInfo、深度图像和深度 CameraInfo；
  2. 将 RGB 图像转换到 HSV 颜色空间；
  3. 提取红色和绿色区域；
  4. 对颜色掩码进行形态学去噪；
  5. 提取候选轮廓并计算面积、圆度和宽高比；
  6. 在轮廓中心附近读取深度中值；
  7. 使用深度 CameraInfo 将像素反投影到深度相机坐标系；
  8. 根据颜色和形状生成 HazardCandidate；
  9. 发布 /competition/hazard_candidates；
 10. 发布 /competition/perception/status。

"""

from __future__ import annotations

import math

import cv2
import message_filters
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

from competition_interfaces.msg import (
    HazardCandidate,
    HazardCandidateArray,
    ModuleStatus,
)

class PerceptionNode(object):
    """基于 RealSense RGB-D 的危险源感知节点。"""

    def __init__(self):
        rospy.init_node(
            "perception_node",
            log_level=rospy.INFO,
        )

        self.bridge = CvBridge()

    # RealSense 话题
        self.rgb_topic = rospy.get_param(
            "~rgb_topic",
            "/real_sense/rgb/image_raw",
        )

        self.rgb_info_topic = rospy.get_param(
            "~rgb_info_topic",
            "/real_sense/rgb/camera_info",
        )

        self.depth_topic = rospy.get_param(
            "~depth_topic",
            "/real_sense/depth/image_raw",
        )

        self.depth_info_topic = rospy.get_param(
            "~depth_info_topic",
            "/real_sense/depth/camera_info",
        )

    # 输出话题
        self.candidate_pub = rospy.Publisher(
            "/competition/hazard_candidates",
            HazardCandidateArray,
            queue_size=10,
        )

        self.status_pub = rospy.Publisher(
            "/competition/perception/status",
            ModuleStatus,
            queue_size=10,
            latch=True,
        )


    # 算法参数
        # 轮廓面积阈值，面积过小的区域不作为候选
        self.min_contour_area = rospy.get_param(
            "~min_contour_area",
            150.0,
        )

        # RealSense 深度有效测距范围，单位：米
        self.min_depth = rospy.get_param(
            "~min_depth",
            0.4,
        )

        self.max_depth = rospy.get_param(
            "~max_depth",
            8.0,
        )

        # HSV 红色阈值
        self.red_lower_1 = np.array(
            [0, 80, 60],
            dtype=np.uint8,
        )
        self.red_upper_1 = np.array(
            [10, 255, 255],
            dtype=np.uint8,
        )

        self.red_lower_2 = np.array(
            [170, 80, 60],
            dtype=np.uint8,
        )
        self.red_upper_2 = np.array(
            [179, 255, 255],
            dtype=np.uint8,
        )

        # HSV 绿色阈值
        self.green_lower = np.array(
            [35, 60, 50],
            dtype=np.uint8,
        )
        self.green_upper = np.array(
            [90, 255, 255],
            dtype=np.uint8,
        )

        # 同步订阅 RGB、RGB CameraInfo、深度和深度 CameraInfo
        rgb_sub = message_filters.Subscriber(
            self.rgb_topic,
            Image,
        )

        rgb_info_sub = message_filters.Subscriber(
            self.rgb_info_topic,
            CameraInfo,
        )

        depth_sub = message_filters.Subscriber(
            self.depth_topic,
            Image,
        )

        depth_info_sub = message_filters.Subscriber(
            self.depth_info_topic,
            CameraInfo,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                rgb_sub,
                rgb_info_sub,
                depth_sub,
                depth_info_sub,
            ],
            queue_size=20,
            slop=0.20,
            allow_headerless=False,
        )

        self.sync.registerCallback(self.image_callback)

        self.publish_status(
            ModuleStatus.STARTING,
            False,
            0.0,
            (
                "waiting for synchronized RGB image, RGB CameraInfo, "
                "depth image and depth CameraInfo"
            ),
        )

        rospy.loginfo("perception_node started")
        rospy.loginfo("RGB image topic: %s", self.rgb_topic)
        rospy.loginfo("RGB CameraInfo topic: %s", self.rgb_info_topic)
        rospy.loginfo("Depth image topic: %s", self.depth_topic)
        rospy.loginfo(
            "Depth CameraInfo topic: %s",
            self.depth_info_topic,
        )
        rospy.loginfo(
            "Output candidate topic: /competition/hazard_candidates"
        )
        rospy.loginfo(
            "Output status topic: /competition/perception/status"
        )

    # 状态发布

    def publish_status(
        self,
        state,
        valid,
        confidence,
        detail,
    ):
        """发布感知模块状态。"""

        msg = ModuleStatus()

        msg.header.stamp = rospy.Time.now()
        msg.state = state
        msg.valid = bool(valid)

        # 防止 confidence 超出 [0, 1]
        msg.confidence = max(
            0.0,
            min(1.0, float(confidence)),
        )

        msg.detail = str(detail)

        self.status_pub.publish(msg)


    def valid_camera_info(self, camera_info, name):
        """
        检查 CameraInfo 是否包含可用的内参和坐标系。
        必要条件：

          - K 至少包含 9 个元素；
          - fx、fy 有限且大于 0；
          - frame_id 非空。
        """

        if camera_info is None:
            return False

        if len(camera_info.K) < 9:
            rospy.logwarn_throttle(
                5.0,
                "%s CameraInfo.K has fewer than 9 elements",
                name,
            )
            return False

        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])

        if not math.isfinite(fx) or not math.isfinite(fy):
            rospy.logwarn_throttle(
                5.0,
                "%s CameraInfo focal length is not finite",
                name,
            )
            return False

        if fx <= 0.0 or fy <= 0.0:
            rospy.logwarn_throttle(
                5.0,
                (
                    "%s CameraInfo has invalid focal length: "
                    "fx=%f fy=%f"
                ),
                name,
                fx,
                fy,
            )
            return False

        if not camera_info.header.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "%s CameraInfo has empty frame_id",
                name,
            )
            return False

        return True

    # RGB-D 同步回调
    def image_callback(
        self,
        rgb_msg,
        rgb_info_msg,
        depth_msg,
        depth_info_msg,
    ):
        """
        同步处理：
          - RGB 图像；
          - RGB CameraInfo；
          - 深度图像；
          - 深度 CameraInfo。
        """

        if not self.valid_camera_info(
            rgb_info_msg,
            "RGB",
        ):
            self.publish_status(
                ModuleStatus.STARTING,
                False,
                0.0,
                "invalid RGB CameraInfo",
            )
            return

        if not self.valid_camera_info(
            depth_info_msg,
            "depth",
        ):
            self.publish_status(
                ModuleStatus.STARTING,
                False,
                0.0,
                "invalid depth CameraInfo",
            )
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="bgr8",
            )

            depth_image = self.depth_to_meters(depth_msg)

        except CvBridgeError as exc:
            rospy.logwarn_throttle(
                2.0,
                "cv_bridge conversion failed: %s",
                str(exc),
            )

            self.publish_status(
                ModuleStatus.ERROR,
                False,
                0.0,
                "cv_bridge conversion failed",
            )
            return

        if bgr_image is None or depth_image is None:
            self.publish_status(
                ModuleStatus.DEGRADED,
                False,
                0.0,
                "RGB or depth image conversion returned empty data",
            )
            return

        if bgr_image.size == 0 or depth_image.size == 0:
            self.publish_status(
                ModuleStatus.DEGRADED,
                False,
                0.0,
                "RGB or depth image is empty",
            )
            return

        # 检查 RGB CameraInfo 与 RGB 图像尺寸是否一致
        if (
            rgb_info_msg.width > 0
            and rgb_info_msg.height > 0
            and (
                rgb_info_msg.width != bgr_image.shape[1]
                or rgb_info_msg.height != bgr_image.shape[0]
            )
        ):
            rospy.logwarn_throttle(
                5.0,
                (
                    "RGB CameraInfo size %dx%d differs from "
                    "image size %dx%d"
                ),
                rgb_info_msg.width,
                rgb_info_msg.height,
                bgr_image.shape[1],
                bgr_image.shape[0],
            )

        # 检查深度 CameraInfo 与深度图像尺寸是否一致
        if (
            depth_info_msg.width > 0
            and depth_info_msg.height > 0
            and (
                depth_info_msg.width != depth_image.shape[1]
                or depth_info_msg.height != depth_image.shape[0]
            )
        ):
            rospy.logwarn_throttle(
                5.0,
                (
                    "Depth CameraInfo size %dx%d differs from "
                    "image size %dx%d"
                ),
                depth_info_msg.width,
                depth_info_msg.height,
                depth_image.shape[1],
                depth_image.shape[0],
            )

        # 执行颜色分割、轮廓分析、深度读取和三维反投影
        candidates = self.detect_objects(
            bgr_image=bgr_image,
            depth_image=depth_image,
            rgb_camera_info=rgb_info_msg,
            depth_camera_info=depth_info_msg,
        )

        array_msg = HazardCandidateArray()

        # 使用 RGB 图像时间戳作为本帧候选数组时间戳
        array_msg.header.stamp = rgb_msg.header.stamp

        # 当前三维点使用深度 CameraInfo 进行反投影
        # 因此候选数组使用深度相机的 frame_id
        frame_id = depth_info_msg.header.frame_id

        if not frame_id:
            frame_id = depth_msg.header.frame_id

        if not frame_id:
            frame_id = rgb_info_msg.header.frame_id

        if not frame_id:
            frame_id = rgb_msg.header.frame_id

        array_msg.header.frame_id = frame_id
        array_msg.candidates = candidates

        self.candidate_pub.publish(array_msg)

        # 只有 position_valid=true 的候选才计入有效检测
        valid_candidates = [
            candidate
            for candidate in candidates
            if candidate.position_valid
        ]

        if valid_candidates:
            frame_confidence = max(
                candidate.confidence
                for candidate in valid_candidates
            )

            self.publish_status(
                ModuleStatus.READY,
                True,
                frame_confidence,
                "RGB-D perception is ready and detecting candidates",
            )
        else:
            self.publish_status(
                ModuleStatus.READY,
                False,
                0.0,
                "RGB-D input is valid but no candidate has valid depth",
            )

    # 深度转换
    def depth_to_meters(self, depth_msg):
        """
        将 ROS 深度图像转换为单位为米的 float32 数组。
        16UC1 / mono16:
            单位为毫米，需要除以 1000。
        32FC1 / 32FC:
            单位为米。
        """
        encoding = depth_msg.encoding.lower()

        if encoding in ("16uc1", "mono16"):
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )

            depth = np.asarray(
                depth,
                dtype=np.float32,
            ) / 1000.0

        elif encoding in ("32fc1", "32fc"):
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )

            depth = np.asarray(
                depth,
                dtype=np.float32,
            )

        else:
            rospy.logwarn_throttle(
                5.0,
                "unsupported depth encoding: %s",
                depth_msg.encoding,
            )
            return None

        # 清理 NaN 和无穷大
        depth[~np.isfinite(depth)] = 0.0

        return depth

    # 颜色检测
    def detect_objects(
        self,
        bgr_image,
        depth_image,
        rgb_camera_info,
        depth_camera_info,
    ):
        """从 RGB 图像中提取红色和绿色目标。"""

        del rgb_camera_info

        hsv_image = cv2.cvtColor(
            bgr_image,
            cv2.COLOR_BGR2HSV,
        )

        # 红色掩码。
        red_mask_1 = cv2.inRange(
            hsv_image,
            self.red_lower_1,
            self.red_upper_1,
        )

        red_mask_2 = cv2.inRange(
            hsv_image,
            self.red_lower_2,
            self.red_upper_2,
        )

        red_mask = cv2.bitwise_or(
            red_mask_1,
            red_mask_2,
        )

        # 绿色掩码。
        green_mask = cv2.inRange(
            hsv_image,
            self.green_lower,
            self.green_upper,
        )

        # 形态学去噪
        red_mask = self.clean_mask(red_mask)
        green_mask = self.clean_mask(green_mask)

        candidates = []

        candidates.extend(
            self.detect_from_mask(
                mask=red_mask,
                color_name="red",
                depth_image=depth_image,
                depth_camera_info=depth_camera_info,
                rgb_shape=bgr_image.shape,
            )
        )

        candidates.extend(
            self.detect_from_mask(
                mask=green_mask,
                color_name="green",
                depth_image=depth_image,
                depth_camera_info=depth_camera_info,
                rgb_shape=bgr_image.shape,
            )
        )

        return candidates

    def clean_mask(self, mask):
        """对颜色掩码进行形态学去噪。"""

        kernel = np.ones(
            (5, 5),
            np.uint8,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2,
        )

        return mask

    # 轮廓检测和候选生成

    def detect_from_mask(
        self,
        mask,
        color_name,
        depth_image,
        depth_camera_info,
        rgb_shape,
    ):
        """从单个颜色掩码中提取轮廓并生成候选消息。"""

        contours_result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        contours = contours_result[-2]
        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.min_contour_area:
                continue

            # 计算外接矩形。
            _, _, width, height = cv2.boundingRect(contour)

            if width <= 0 or height <= 0:
                continue

            # 计算轮廓周长和圆度。
            perimeter = cv2.arcLength(
                contour,
                True,
            )

            if perimeter <= 0.0:
                continue

            circularity = (
                4.0
                * math.pi
                * area
                / (perimeter * perimeter)
            )

            aspect_ratio = float(width) / float(height)

            # 计算轮廓质心。
            moments = cv2.moments(contour)

            if abs(moments["m00"]) < 1e-6:
                continue

            center_u = moments["m10"] / moments["m00"]
            center_v = moments["m01"] / moments["m00"]

            # 读取轮廓中心附近的有效深度中值。
            depth_value = self.robust_depth(
                depth_image=depth_image,
                center_u=center_u,
                center_v=center_v,
                width=width,
                height=height,
                rgb_shape=rgb_shape,
            )

            position_valid = (
                depth_value is not None
                and math.isfinite(depth_value)
                and self.min_depth <= depth_value <= self.max_depth
            )

            if position_valid:
                point = self.project_to_3d(
                    center_u=center_u,
                    center_v=center_v,
                    depth_value=depth_value,
                    depth_shape=depth_image.shape,
                    rgb_shape=rgb_shape,
                    depth_camera_info=depth_camera_info,
                )
            else:
                # 深度无效时不填充伪坐标。
                point = (0.0, 0.0, 0.0)

            label = self.classify_object(
                color_name=color_name,
                circularity=circularity,
                aspect_ratio=aspect_ratio,
                width=width,
                height=height,
                depth_value=depth_value,
                depth_camera_info=depth_camera_info,
            )

            confidence = self.calculate_confidence(
                area=area,
                circularity=circularity,
                aspect_ratio=aspect_ratio,
                position_valid=position_valid,
                label=label,
            )

            candidate = HazardCandidate()

            # 如果 HazardCandidate.msg 包含 Header，
            # 使用当前 RGB 帧时间戳和深度相机坐标系。
            candidate.header.stamp = rospy.Time.now()
            candidate.header.frame_id = (
                depth_camera_info.header.frame_id
            )

            candidate.position.x = point[0]
            candidate.position.y = point[1]
            candidate.position.z = point[2]

            candidate.label = label
            candidate.confidence = confidence
            candidate.position_valid = position_valid

            detections.append(candidate)

        return detections

    # 深度读取

    def robust_depth(
        self,
        depth_image,
        center_u,
        center_v,
        width,
        height,
        rgb_shape,
    ):
        """获取目标中心区域的有效深度中值。"""

        depth_height, depth_width = depth_image.shape[:2]
        rgb_height, rgb_width = rgb_shape[:2]

        if rgb_width <= 0 or rgb_height <= 0:
            return None

        # 当前实现假设 RGB 和深度图像已对齐，
        # 这里按照图像尺寸比例将 RGB 像素映射到深度图。
        scale_x = float(depth_width) / float(rgb_width)
        scale_y = float(depth_height) / float(rgb_height)

        u = int(round(center_u * scale_x))
        v = int(round(center_v * scale_y))

        # 防止中心点落到图像边界外。
        u = max(0, min(depth_width - 1, u))
        v = max(0, min(depth_height - 1, v))

        half_w = max(
            2,
            int(round(width * scale_x * 0.2)),
        )

        half_h = max(
            2,
            int(round(height * scale_y * 0.2)),
        )

        u0 = max(0, u - half_w)
        u1 = min(depth_width, u + half_w + 1)

        v0 = max(0, v - half_h)
        v1 = min(depth_height, v + half_h + 1)

        if u0 >= u1 or v0 >= v1:
            return None

        roi = depth_image[v0:v1, u0:u1]

        valid = roi[
            np.isfinite(roi)
            & (roi >= self.min_depth)
            & (roi <= self.max_depth)
        ]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    # 三维反投影
    def project_to_3d(
        self,
        center_u,
        center_v,
        depth_value,
        depth_shape,
        rgb_shape,
        depth_camera_info,
    ):
        """
        将 RGB 图像中的像素点反投影到深度相机坐标系。

        针孔相机模型：

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            Z = depth
        ROS optical frame 约定：

            X 向右；
            Y 向下；
            Z 向前。

        注意：
        当前实现假设 RGB 图像和深度图已经对齐，
        使用图像尺寸比例将 RGB 像素映射到深度图像像素。
        """

        depth_height, depth_width = depth_shape[:2]
        rgb_height, rgb_width = rgb_shape[:2]

        del depth_height
        del rgb_height

        if rgb_width <= 0 or rgb_height <= 0:
            return (0.0, 0.0, 0.0)

        # CameraInfo.K：

        # [fx,  0, cx,
        #   0, fy, cy,
        #   0,  0,  1]

        fx = float(depth_camera_info.K[0])
        fy = float(depth_camera_info.K[4])
        cx = float(depth_camera_info.K[2])
        cy = float(depth_camera_info.K[5])

        if (
            not math.isfinite(fx)
            or not math.isfinite(fy)
            or fx <= 0.0
            or fy <= 0.0
        ):
            return (0.0, 0.0, 0.0)

        depth_u = (
            center_u
            * float(depth_width)
            / float(rgb_width)
        )

        depth_v = (
            center_v
            * float(depth_shape[0])
            / float(rgb_shape[0])
        )

        x = (depth_u - cx) * depth_value / fx
        y = (depth_v - cy) * depth_value / fy
        z = depth_value

        if not all(
            math.isfinite(value)
            for value in (x, y, z)
        ):
            return (0.0, 0.0, 0.0)

        return (
            float(x),
            float(y),
            float(z),
        )

    # 目标分类
    def classify_object(
        self,
        color_name,
        circularity,
        aspect_ratio,
        width,
        height,
        depth_value,
        depth_camera_info,
    ):
        """生成危险源语义标签。"""

        # 绿色球体属于干扰目标。
        if color_name == "green":
            return "green_sphere"

        fx = float(depth_camera_info.K[0])
        fy = float(depth_camera_info.K[4])

        if (
            depth_value is not None
            and math.isfinite(depth_value)
            and depth_value > 0.0
            and fx > 0.0
            and fy > 0.0
        ):
            diameter_x = (
                float(width)
                * depth_value
                / fx
            )

            diameter_y = (
                float(height)
                * depth_value
                / fy
            )

            diameter = 0.5 * (
                diameter_x + diameter_y
            )
        else:
            diameter = 0.0

        # 第一版启发式分类规则：
        # 圆度较高、宽高接近、估计直径约为 0.20~0.42 m 时判为红球。
        sphere_shape = (
            circularity >= 0.68
            and 0.70 <= aspect_ratio <= 1.43
            and 0.20 <= diameter <= 0.42
        )

        if sphere_shape:
            return "red_sphere"

        # 其他红色区域判为红色方块干扰源。
        return "red_box"


    # 置信度计算
    def calculate_confidence(
        self,
        area,
        circularity,
        aspect_ratio,
        position_valid,
        label,
    ):
        """根据面积、形状和深度有效性计算 [0, 1] 置信度。"""

        area_score = min(
            1.0,
            float(area) / 3000.0,
        )

        circularity_score = max(
            0.0,
            min(1.0, circularity),
        )

        # 宽高比越接近 1，aspect_score 越高。
        aspect_score = max(
            0.0,
            1.0 - min(
                1.0,
                abs(1.0 - aspect_ratio),
            ),
        )

        depth_score = 1.0 if position_valid else 0.25

        if label in (
            "red_sphere",
            "green_sphere",
        ):
            # 球体更重视圆度和宽高比。
            shape_score = (
                0.45 * circularity_score
                + 0.30 * aspect_score
                + 0.25 * depth_score
            )
        else:
            # 方块等非圆形目标。
            shape_score = (
                0.35 * (1.0 - circularity_score)
                + 0.35 * aspect_score
                + 0.30 * depth_score
            )

        confidence = (
            0.20 * area_score
            + 0.65 * shape_score
            + 0.15 * depth_score
        )

        return max(
            0.0,
            min(1.0, float(confidence)),
        )

    # ROS 事件循环

    def spin(self):
        """进入 ROS 事件循环，等待订阅回调。"""

        rospy.spin()

if __name__ == "__main__":
    node = PerceptionNode()
    node.spin()