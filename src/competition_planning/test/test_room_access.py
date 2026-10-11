#!/usr/bin/env python3
"""test_room_access.py — 验证距离场可区分宽房间与窄伪空间(电梯井/楼梯间)。

核心主张（供 B2 frontier room-bias 使用）：
  - 房间内部(远离墙)的 cell 距墙距离大(可达 ~4m+)
  - 走廊中心 ~1m、电梯井 ~1.2m、楼梯间 ~1.6m → 都小
  因此"到墙距离"是把探索引向房间宽区(危险源区)、避开窄伪空间的可用信号。
"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from room_access import distance_to_wall  # noqa: E402


def build_scene():
    """res=1.0: 走廊(x14-15) + 大房A/B(门洞3格) + 窄伪死端(2宽, 像电梯井)。"""
    W, H = 34, 30
    g = np.full((H, W), 100, dtype=np.int8)
    g[2:28, 14:16] = 0  # 走廊
    g[6:20, 2:13] = 0   # 房A
    g[10:13, 13] = 0    # A 门洞
    g[6:20, 17:28] = 0  # 房B
    g[10:13, 16] = 0    # B 门洞
    g[7:8, 3:5] = 100   # A 角落家具
    g[24:27, 10:12] = 0     # 窄伪死端
    g[25, 12:14] = 0        # 伪房入口
    g[0:2, :] = 100
    g[28:30, :] = 100
    return g.reshape(-1).tolist(), W, H


class DistFieldTest(unittest.TestCase):
    def setUp(self):
        self.data, self.w, self.h = build_scene()

    def test_room_center_far_from_wall(self):
        d = distance_to_wall(self.data, self.w, self.h)
        # 房A 与 房B 中心(dist 取局部高值)
        self.assertGreater(float(d[13][7]), 3.0)
        self.assertGreater(float(d[13][22]), 3.0)

    def test_corridor_and_pseudo_narrow_low_dist(self):
        d = distance_to_wall(self.data, self.w, self.h)
        # 走廊中心 < 1.5
        self.assertLess(float(d[8][14]), 1.5)
        # 窄伪死端(2 宽)上下行中心 < 1.5 → 不被当成房间宽区
        self.assertLess(float(d[24][10]), 1.5)
        self.assertLess(float(d[24][11]), 1.5)
        self.assertLess(float(d[26][11]), 1.5)

    def test_room_far_dist_exceeds_pseudo(self):
        d = distance_to_wall(self.data, self.w, self.h)
        # 房间最宽区 dist 显著大于伪窄区 → 可作 bias 区分
        room = float(np.max(d[7:19, 2:13]))
        pseudo = float(np.max(d[24:27, 10:12]))
        self.assertGreater(room, pseudo + 1.0)


if __name__ == "__main__":
    unittest.main()
