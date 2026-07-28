#!/usr/bin/env python3
"""perception_node — hazard source detection using RealSense RGB-D.

SKELETON ONLY — algorithm not implemented.

Allowed inputs:
  /real_sense/rgb/image_raw       (sensor_msgs/Image)
  /real_sense/depth/image_raw     (sensor_msgs/Image)
  /real_sense/depth/points        (sensor_msgs/PointCloud2)
  /real_sense/rgb/camera_info     (sensor_msgs/CameraInfo)
  /real_sense/depth/camera_info   (sensor_msgs/CameraInfo)

Task:
  1. Detect red-coloured objects from RGB (colour thresholding or learning)
  2. Use depth to estimate 3D position in the camera frame
  3. Classify shape: sphere (radius ~0.15m, danger) vs box (~0.30m, distractor)
  4. Publish HazardCandidateArray with frame_id = camera optical frame
  5. Conversion to world frame must be done by the localization/TF chain

Forbidden:
  Gazebo model state queries for source position
  Reading danger_truth.json to look up source locations
  Subscribing to /ground_truth/*

处理流程：
    1. 同步接收 RGB 图像和深度图像。
    2. 将 RGB 图像转换到 HSV 颜色空间。
    3. 提取红色和绿色区域。
    4. 对颜色掩码进行形态学去噪。
    5. 提取候选轮廓，并计算面积、圆度、宽高比。
    6. 在轮廓中心附近读取深度中值。
    7. 根据相机内参将像素坐标反投影到相机坐标系。
    8. 根据颜色和形状生成 HazardCandidate。
    9. 发布 /competition/hazard_candidates。
"""

from __future__ import annotations
import math
import rospy
import cv2
import message_filters
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from competition_interfaces.msg import (
    ModuleStatus,
    HazardCandidate,
    HazardCandidateArray,
)

class PerceptionNode(object):
    """RGB-D感知节点"""
    def __init__(self):
        rospy.init_node("perception_node", log_level=rospy.INFO)

        self.bridge = CvBridge()
        self.rgb_topic = rospy.get_param(
            "~rgb_topic",
            "/real_sense/rgb/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic",
            "/real_sense/depth/image_raw"
        )
        self.depth_info_topic = rospy.get_param(
            "~depth_info_topic",
            "/real_sense/depth/camera_info"
        )
     # 发布所有检测到的候选目标。
        self.candidate_pub = rospy.Publisher(
            "/competition/hazard_candidates",
            HazardCandidateArray,
            queue_size=10
        )

        # latch=True 使新订阅者能够收到最近一次状态消息。
        self.status_pub = rospy.Publisher(
            "/competition/perception/status",
            ModuleStatus,
            queue_size=10,
            latch=True
        )
        # 轮廓面积阈值，面积过小不候选
        self.min_contour_area = rospy.get_param(
            "~min_contour_area",
            150.0
        )
         # RealSense 点云的有效测距范围
        self.min_depth = rospy.get_param("~min_depth", 0.4)
        self.max_depth = rospy.get_param("~max_depth", 8.0)

        # HSV_red
        self.red_lower_1 = np.array([0, 80, 60], dtype=np.uint8)
        self.red_upper_1 = np.array([10, 255, 255], dtype=np.uint8)
        self.red_lower_2 = np.array([170, 80, 60], dtype=np.uint8)
        self.red_upper_2 = np.array([179, 255, 255], dtype=np.uint8)

        # HSV_green
        self.green_lower = np.array([35, 60, 50], dtype=np.uint8)
        self.green_upper = np.array([90, 255, 255], dtype=np.uint8)

        #camera_info
        self.camera_info = None
        self.info_sub = rospy.Subscriber(
            self.depth_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )
        # use message_filters subscribe RGB & depth
        rgb_sub = message_filters.Subscriber(
            self.rgb_topic,
            Image
        )
        depth_sub = message_filters.Subscriber(
            self.depth_topic,
            Image
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.15
        )
        self.sync.registerCallback(self.image_callback)

        self.publish_status(
            ModuleStatus.STARTING,
            False,
            0.0,
            "waiting for depth camera info and synchronized images"
        )

        #logger_info
        rospy.loginfo("perception_node started")
        rospy.loginfo("RGB topic: %s", self.rgb_topic)
        rospy.loginfo("Depth topic: %s", self.depth_topic)
        rospy.loginfo("Depth info topic: %s", self.depth_info_topic)
    
    def publish_status(self,state,valid,confidence,detail):
        """发布本节点运行状态"""
        msg = ModuleStatus()
        msg.header.stamp = rospy.Time.now()
        msg.state = state
        msg.valid = bool(valid)

        #防止异常值（超出消息定义的[0,1]范围）
        msg.confidence = max(0.0,min(1.0,float(confidence)))
        msg.detail = detail
        self.status_pub.publish(msg)

    def camera_info_callback(self,msg):
        self.camera_info = msg

    def image_callback(self,rgb_msg,depth_msg):
        """RGB和深度图像同步之后的回调函数"""
        #exception logger 
        if self.camera_info is None:
            self.publish_status(
                ModuleStatus.STARTING,
                False,
                0.0,
                "waiting for depth camera info"
            )
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="bgr8"
            )
            depth_image = self.depth_to_meters(depth_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(
                2.0,
                "cv_bridge conversion failed: %s",
                str(exc)
            )
            self.publish_status(
                ModuleStatus.FAILED,
                False,
                0.0,
                "cv_bridge conversion failed"
            )
            return

        if bgr_image is None or depth_image is None:
            return

        if bgr_image.size == 0 or depth_image.size == 0:
            return

        # 执行颜色分割、轮廓分析、深度读取和三维反投影。
        candidates = self.detect_objects(
            bgr_image,
            depth_image,
            self.camera_info
        )

        array_msg = HazardCandidateArray()
        array_msg.header.stamp = rgb_msg.header.stamp

        # 相机坐标系CameraInfo.header.frame_id
        frame_id = self.camera_info.header.frame_id
        if not frame_id:
            frame_id = depth_msg.header.frame_id
        if not frame_id:
            frame_id = rgb_msg.header.frame_id

        array_msg.header.frame_id = frame_id
        array_msg.candidates = candidates
        self.candidate_pub.publish(array_msg)

        # 只有位置有效的候选才计入有效检测数量。
        valid_candidates = [
            candidate for candidate in candidates
            if candidate.position_valid
        ]

        if valid_candidates:
            # 状态 confidence 取本帧最高候选置信度。
            frame_confidence = max(
                candidate.confidence
                for candidate in valid_candidates
            )
            self.publish_status(
                ModuleStatus.RUNNING,
                True,
                frame_confidence,
                "detecting RGB-D candidates"
            )
        else:
            self.publish_status(
                ModuleStatus.RUNNING,
                False,
                0.0,
                "no candidate with valid depth"
            )

    def depth_to_meters(self, depth_msg):
        """
        将 ROS 深度图像转换为以米为单位的 float32 数组
            16UC1 / mono16:单位毫米,需要除以 1000。
            32FC1 / 32FC:单位米
        """
        encoding = depth_msg.encoding.lower()

        if encoding in ("16uc1", "mono16"):
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough"
            )
            depth = np.asarray(depth, dtype=np.float32) / 1000.0

        elif encoding in ("32fc1", "32fc"):
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough"
            )
            depth = np.asarray(depth, dtype=np.float32)

        else:
            rospy.logwarn_throttle(
                5.0,
                "unsupported depth encoding: %s",
                depth_msg.encoding
            )
            return None

        # 无效值清洗：NaN 和无穷大
        depth[~np.isfinite(depth)] = 0.0
        return depth

    def detect_objects(self, bgr_image, depth_image, camera_info):
          """从 RGB 图像中提取红色和绿色目标。"""
          hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

          # 红色
          red_mask_1 = cv2.inRange(
              hsv_image,
              self.red_lower_1,
              self.red_upper_1
          )
          red_mask_2 = cv2.inRange(
              hsv_image,
              self.red_lower_2,
              self.red_upper_2
          )
          red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

          # 绿色
          green_mask = cv2.inRange(
              hsv_image,
              self.green_lower,
              self.green_upper
          )

          # 去噪
          red_mask = self.clean_mask(red_mask)
          green_mask = self.clean_mask(green_mask)

          candidates = []
          candidates.extend(
              self.detect_from_mask(
                  red_mask,
                  "red",
                  depth_image,
                  camera_info,
                  bgr_image.shape
              )
          )
          candidates.extend(
              self.detect_from_mask(
                  green_mask,
                  "green",
                  depth_image,
                  camera_info,
                  bgr_image.shape
              )
          )

          return candidates

    def clean_mask(self, mask):
            """掩码处理，颜色去噪"""
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
                iterations=1
            )
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=2
            )

            return mask
    
    def detect_from_mask(
        self,
        mask,
        color_name,
        depth_image,
        camera_info,
        rgb_shape
    ):
        """
        从单个颜色掩码中提取轮廓并生成候选消息
        """
        contours_result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        contours = contours_result[-2]
        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue

            "轮廓判断（圆形）"
            # 外接矩形计算目标的宽高
            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue
            # 圆度公式(4*pi*面积 / 周长^2)判断圆形
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue

            circularity = (
                4.0 * math.pi * area / (perimeter * perimeter)
            )
            aspect_ratio = float(width) / float(height)

            # 获取轮廓质心
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue

            center_u = moments["m10"] / moments["m00"]
            center_v = moments["m01"] / moments["m00"]

            depth_value = self.robust_depth(
                depth_image,
                center_u,
                center_v,
                width,
                height,
                rgb_shape
            )

            position_valid = (
                depth_value is not None
                and self.min_depth <= depth_value <= self.max_depth
            )

            if position_valid:
                point = self.project_to_3d(
                    center_u,
                    center_v,
                    depth_value,
                    depth_image.shape,
                    rgb_shape,
                    camera_info
                )
            else:
                # 深度无效时point参数置0
                point = (0.0, 0.0, 0.0)

            label = self.classify_object(
                color_name,
                circularity,
                aspect_ratio,
                width,
                height,
                depth_value,
                camera_info
            )

            confidence = self.calculate_confidence(
                area,
                circularity,
                aspect_ratio,
                position_valid,
                label
            )

            candidate = HazardCandidate()
            candidate.header.stamp = rospy.Time.now()
            candidate.header.frame_id = camera_info.header.frame_id
            candidate.position.x = point[0]
            candidate.position.y = point[1]
            candidate.position.z = point[2]
            candidate.label = label
            candidate.confidence = confidence
            candidate.position_valid = position_valid

            detections.append(candidate)

        return detections
    
    def robust_depth(
        self,
        depth_image,
        center_u,
        center_v,
        width,
        height,
        rgb_shape
    ):
        """获取目标中心区域的有效深度中值"""
        depth_height, depth_width = depth_image.shape[:2]
        rgb_height, rgb_width = rgb_shape[:2]

        scale_x = float(depth_width) / float(rgb_width)
        scale_y = float(depth_height) / float(rgb_height)

        u = int(round(center_u * scale_x))
        v = int(round(center_v * scale_y))

        half_w = max(2, int(round(width * scale_x * 0.2)))
        half_h = max(2, int(round(height * scale_y * 0.2)))

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
    
    def project_to_3d(
        self,
        center_u,
        center_v,
        depth_value,
        depth_shape,
        rgb_shape,
        camera_info
    ):
        """
        将 RGB 图像中的像素点反投影到深度相机坐标系
        针孔相机模型：
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            Z = depth
        对 ROS optical frame约定:
            X 向右
            Y 向下
            Z 向前
        """
        depth_height, depth_width = depth_shape[:2]
        rgb_height, rgb_width = rgb_shape[:2]

        # CameraInfo.K 为：
        # [fx,  0, cx,
        #   0, fy, cy,
        #   0,  0,  1]
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

        if fx <= 0.0 or fy <= 0.0:
            return (0.0, 0.0, 0.0)

        depth_u = center_u * float(depth_width) / float(rgb_width)
        depth_v = center_v * float(depth_height) / float(rgb_height)

        x = (depth_u - cx) * depth_value / fx
        y = (depth_v - cy) * depth_value / fy
        z = depth_value

        return (float(x), float(y), float(z))

    def classify_object(
        self,
        color_name,
        circularity,
        aspect_ratio,
        width,
        height,
        depth_value,
        camera_info
    ):
        """生成危险源语义标签"""
        if color_name == "green":
            return "green_sphere"

        fx = camera_info.K[0]
        fy = camera_info.K[4]

        if depth_value is not None and depth_value > 0.0:
            diameter_x = float(width) * depth_value / max(fx, 1.0)
            diameter_y = float(height) * depth_value / max(fy, 1.0)
            diameter = 0.5 * (diameter_x + diameter_y)
        else:
            diameter = 0.0

        #TODO
        # 第一版启发式分类规则:圆度较高、宽高接近、估计尺寸接近 0.30 m 时判为红球。
        sphere_shape = (
            circularity >= 0.68
            and 0.70 <= aspect_ratio <= 1.43
            and 0.20 <= diameter <= 0.42
        )

        if sphere_shape:
            return "red_sphere"

        # 其他红色区域判为红色方形
        return "red_box"

    def calculate_confidence(
        self,
        area,
        circularity,
        aspect_ratio,
        position_valid,
        label
    ):
        """根据面积、形状和深度有效性计算置信度∈[0, 1]"""
        area_score = min(1.0, float(area) / 3000.0)
        circularity_score = max(0.0, min(1.0, circularity))

        # 宽高比越接近1,aspect_score 越高
        aspect_score = max(
            0.0,
            1.0 - min(1.0, abs(1.0 - aspect_ratio))
        )
        depth_score = 1.0 if position_valid else 0.25

        if label in ("red_sphere", "green_sphere"):
            # 球体更看重圆度和宽高比。
            shape_score = (
                0.45 * circularity_score
                + 0.30 * aspect_score
                + 0.25 * depth_score
            )
        else:
            # 非圆形
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

        return max(0.0, min(1.0, float(confidence)))
    

    def spin(self):
        """进入 ROS 事件循环，等待订阅回调"""
        rospy.spin()

if __name__ == "__main__":
    node = PerceptionNode()
    node.spin()
