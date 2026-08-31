#!/usr/bin/env python3
"""filter_livox — 过滤 /livox/Pointcloud2 并输出到 /livox/Pointcloud2_filtered。

两类剔除:
  1) 超远异常点 (r > max_range): SimEnv 中 Livox 仿真插件对未命中/穿出楼栋的
     射线产生 100~700m 超远点（雷达 max range 仅 40m），污染 FAST-LIO 配准。
  2) 近场身体点 (r < min_range): Mid360 安装在机器人身体上，0.5~1.5m 球壳内
     的 ~45% 点是机器人自身的身体/腿。走路时腿摆动会被 FAST-LIO 误当成静态
     环境去匹配，导致运动严重欠跟踪（真实 0.5m/s 只估 10%）。剔除后机器人
     自身不再进入配准。

用法（容器内）:
    python3 -u /workspace/SimEnv/scripts/filter_livox.py --max-range 40 --min-range 1.0

FAST-LIO 使用过滤后话题:
    roslaunch competition_localization localization_tuned.launch
"""

import argparse

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2


class LivoxFilter:
    def __init__(self, max_range, min_range):
        self.max_range = max_range
        self.min_range = min_range
        self.sub = rospy.Subscriber("/livox/Pointcloud2", PointCloud2,
                                    self.cb, queue_size=5, tcp_nodelay=True)
        self.pub = rospy.Publisher("/livox/Pointcloud2_filtered", PointCloud2,
                                   queue_size=5)
        self.fields_off = {}
        self.point_step = None
        self.removed_total = 0
        self.kept_total = 0

    def _resolve_fields(self, msg):
        for f in msg.fields:
            self.fields_off[f.name] = f.offset
        self.point_step = msg.point_step

    def cb(self, msg):
        if not self.fields_off:
            self._resolve_fields(msg)
        ox, oy, oz = self.fields_off["x"], self.fields_off["y"], self.fields_off["z"]
        ps = self.point_step
        n = msg.width * msg.height
        if n == 0 or len(msg.data) < n * ps:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps).reshape(n, ps)
        x = arr[:, ox:ox + 4].copy().view(np.float32).ravel()
        y = arr[:, oy:oy + 4].copy().view(np.float32).ravel()
        z = arr[:, oz:oz + 4].copy().view(np.float32).ravel()
        r = np.sqrt(x * x + y * y + z * z)
        valid = np.isfinite(r) & (r <= self.max_range) & (r >= self.min_range)
        n_keep = int(valid.sum())
        n_removed = n - n_keep
        self.kept_total += n_keep
        self.removed_total += n_removed

        if n_keep == 0:
            return
        keep_idx = np.nonzero(valid)[0]
        new_data = arr[keep_idx].tobytes()

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n_keep
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = ps
        out.row_step = n_keep * ps
        out.is_dense = msg.is_dense
        out.data = new_data
        self.pub.publish(out)

        if n_removed:
            rospy.loginfo_throttle(5.0,
                f"[filter_livox] 剔除 {n_removed}/{n} 点 "
                f"(<{self.min_range}m 身体点 或 >{self.max_range}m 超远点), "
                f"保留 {n_keep}；累计剔除率 {100.0*self.removed_total/max(1,self.removed_total+self.kept_total):.1f}%")
        else:
            rospy.loginfo_throttle(10.0, f"[filter_livox] 无剔除，保留 {n_keep}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--min-range", type=float, default=1.0,
                    help="剔除距离小于该值的点（机器人自身身体/腿）")
    args = ap.parse_args()
    rospy.init_node("filter_livox", anonymous=True)
    f = LivoxFilter(args.max_range, args.min_range)
    rospy.loginfo(f"[filter_livox] 订阅 /livox/Pointcloud2，剔除 <{args.min_range}m 与 >{args.max_range}m 的点")
    rospy.spin()
