#!/usr/bin/env python3
"""room_access.py — 在线"房间块"识别（纯逻辑，无 ROS，可单测）。

思路（v1）：
  房间 = 走廊/大厅之外、墙围出的**宽块**。用"距最近墙的距离场"区分宽/窄：
    - 走廊宽~2.2m  → 半宽~1.1m → 中心距墙 ≤ ~1.1m
    - 电梯井 2.4m  → ≤ ~1.2m
    - 楼梯间 3.2m  → ≤ ~1.6m
    - 真房间宽 ~8.4m → 中心距墙可达 ~4m，存在大片 dist≥min_room_dist 的区域
  于是"dist ≥ min_room_dist 的 free 大片"即为房间/大厅级宽块；窄的伪空间(电梯井/
  楼梯间)被阈值自动排除。含机器人起始点(home)的宽块视为大厅/通道(非房间)。
  unknown 不当作墙（那是待探测区），房间"是否扫完"用块内 unknown 计数判定。

单位：所有几何在 cell 系内计算；分辨率仅用于把米阈值换算成 cell 与面积(m²)。
"""
from collections import deque

import numpy as np


def distance_to_wall(
    data, width, height, occ_thresh=50, free_value=0
):
    """4 邻多源 BFS：每个 free cell 到最近 occupied 墙的距离(cell 数)。

    unknown(-1) 与 free 一样可通行(不作为墙)。返回与 data 同形状的 float 数组，
    墙/未知源处为 0。
    """
    dist = np.full((height, width), -1.0, dtype=np.float32)
    dq = deque()
    for y in range(height):
        base = y * width
        for x in range(width):
            v = data[base + x]
            if v >= occ_thresh:  # occupied → source(墙)
                dist[y, x] = 0.0
                dq.append((x, y))
    # 4 邻传播
    while dq:
        x, y = dq.popleft()
        d = dist[y, x] + 1.0
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if dist[ny, nx] >= 0.0:
                continue
            idx = ny * width + nx
            v = data[idx]
            if v == free_value or v == -1:  # free 或 unknown
                dist[ny, nx] = d
                dq.append((nx, ny))
    return dist


def _label_components(mask):
    """mask: HxW bool。8 邻连通标记。返回 (labels, n, area_per_label dict)。"""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    current = 0
    area = {}
    # 用迭代栈(BFS)给每个连通域标号，避免递归深度问题。
    for sy in range(h):
        for sx in range(w):
            if mask[sy, sx] and labels[sy, sx] == 0:
                current += 1
                stack = [(sx, sy)]
                labels[sy, sx] = current
                cnt = 0
                while stack:
                    x, y = stack.pop()
                    cnt += 1
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            if dx == 0 and dy == 0:
                                continue
                            nx, ny = x + dx, y + dy
                            if not (0 <= nx < w and 0 <= ny < h):
                                continue
                            if mask[ny, nx] and labels[ny, nx] == 0:
                                labels[ny, nx] = current
                                stack.append((nx, ny))
                area[current] = cnt
    return labels, current, area


def find_room_candidates(
    data,
    width,
    height,
    resolution=0.1,
    min_room_dist_m=1.9,
    min_room_area_m2=20.0,
    home_cell=None,
    occ_thresh=50,
):
    """返回候选房间块列表。

    每块 dict:
      label, area_m2, rep=(x,y)[dist 最大 cell], unknown_count, bbox
    排除：含 home_cell 的宽块(大厅/通道)与面积不足的窄块(电梯井/楼梯间/噪声)。
    """
    min_dist_cell = max(1, int(round(min_room_dist_m / resolution)))
    dist = distance_to_wall(data, width, height, occ_thresh=occ_thresh)
    free_or_unknown = np.zeros((height, width), dtype=bool)
    for y in range(height):
        base = y * width
        for x in range(width):
            v = data[base + x]
            if v == 0 or v == -1:
                free_or_unknown[y, x] = True
    wide = free_or_unknown & (dist >= min_dist_cell)
    labels, n, area = _label_components(wide)
    blocks = []
    for lab in range(1, n + 1):
        ys, xs = np.where(labels == lab)
        if len(xs) < 4:
            continue
        # representative = dist 最大的 cell
        best = None
        bestd = -1.0
        unknown_cnt = 0
        for yy, xx in zip(ys.tolist(), xs.tolist()):
            d = float(dist[yy, xx])
            if d > bestd:
                bestd = d
                best = (xx, yy)
            idx = yy * width + xx
            if data[idx] == -1:
                unknown_cnt += 1
        area_m2 = len(xs) * (resolution * resolution)
        bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
        blocks.append({
            "label": lab,
            "area_m2": area_m2,
            "rep": best,
            "dist_at_rep": bestd,
            "unknown_count": unknown_cnt,
            "bbox": bbox,
        })
    # 分类：含 home 的块 = 大厅/通道；其余面积达标 = 房间候选
    home = tuple(home_cell) if home_cell is not None else None
    rooms, through = [], []
    for b in blocks:
        x0, y0, x1, y1 = b["bbox"]
        in_home = home is not None and (x0 <= home[0] <= x1 and y0 <= home[1] <= y1)
        if in_home:
            through.append(b)
        elif b["area_m2"] >= min_room_area_m2:
            rooms.append(b)
        # 面积不足的宽块忽略(窄伪空间本就到不了 min_room_dist，能到这阈值的基本是真房)
    rooms.sort(key=lambda b: (-b["area_m2"], b["bbox"][1]))
    return rooms, through, dist


def room_cleared(block, data, width, height):
    """房间块是否已无 unknown(扫完)。block 来自 find_room_candidates。"""
    if block is None:
        return True
    return block["unknown_count"] <= 0
