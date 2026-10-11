# =============================================================================
# 模块说明（中文）：floor_lifecycle.py —— "当前楼层完成"门（纯逻辑）
#
# 职责：从公开场景信息（门/电梯 id 列表）推导 home_floor 与必需楼层集合，
#       并用"两条相互独立的证据通道"判断当前楼层是否真的完成：
#         通道 1：/competition/planning/exploration_complete 布尔（必须先 false 后 true）
#         通道 2：任务状态消息，按 task_id + 时间戳单调性记录"succeeded and confirmed"
#       两者都新鲜且为真时，才把该层标记为完成。
# 为什么必须双证 + 新鲜度：探索完成信号存在瞬时抖动与陈旧重发，单通道判定会
#       造成"假完成"并让任务提前返航；判据不新鲜时宁可冻结运动，也不放行。
# 为什么必须纯逻辑：本文件不做任何几何、ROS 或环境服务调用，才能对"完成判定"
#       做确定性验证（不依赖 Gazebo/SDK）。
# 结构导航：FloorLifecycle（行 7 起）→ __init__（行 8-38，推导 home_floor 与
#       required 集合并对非法公开场景抛错）→ activate（行 40-53，调用者必须先
#       证明"已到达/已建图"）→ observe_bool / observe_status（行 55-75，两通道）
#       → consume（行 76-85，双证新鲜且为真才消费）。
# =============================================================================
"""Current-floor completion gate; no geometry, ROS or environment service calls."""
import math
import threading
import uuid


class FloorLifecycle:
    def __init__(self, public_scene, max_age=1.0):
        self.lock = threading.RLock()
        doors = public_scene.get('door_ids', public_scene.get('doors', []))
        homes = {d['floor_index'] for d in doors if d.get('kind') == 'main_entrance'}
        if len(homes) != 1:
            raise ValueError('public main entrance must identify one home floor')
        self.home_floor = next(iter(homes))
        self.catalog = {}
        for elevator in public_scene.get('elevators', []):
            floors = elevator.get('served_floors', [])
            name = elevator.get('id')
            if not isinstance(name, str) or not name or name in self.catalog:
                raise ValueError('invalid or duplicate public elevator id')
            if not floors or any(type(f) is not int or f < 0 for f in floors):
                raise ValueError('invalid public served floors')
            self.catalog[name] = frozenset(floors)
        if type(self.home_floor) is not int or self.home_floor < 0:
            raise ValueError('invalid home floor')
        self.required = frozenset({self.home_floor}.union(*self.catalog.values()))
        self.current_floor = self.home_floor
        self.completed = set()
        self.visited = set()
        self.active = False
        self.task_id = ''
        self.started_at = 0.0
        self.max_age = float(max_age)
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('invalid evidence maximum age')
        self._bool_at = None
        self._false_seen = False
        self._proof = None

    def activate(self, floor, now):
        """Caller must establish floor arrival/map lifecycle before activation."""
        with self.lock:
            if floor not in self.required or not math.isfinite(now) or now <= 0:
                raise ValueError('invalid floor activation')
            self.current_floor = floor
            self.visited.add(floor)
            self.task_id = uuid.uuid4().hex
            self.started_at = now
            self.active = True
            self._bool_at = None
            self._false_seen = False
            self._proof = None
            return self.task_id

    def observe_bool(self, value, now):
        with self.lock:
            if not self.active or not math.isfinite(now) or now < self.started_at:
                return
            if not value:
                self._false_seen = True
                self._bool_at = None
            elif self._false_seen:
                self._bool_at = now

    def observe_status(self, task_id, stamp, succeeded, confirmed, now):
        with self.lock:
            # An old-task message cannot either complete OR erase a new task.
            if not self.active or task_id != self.task_id:
                return
            if not self._fresh(stamp, now) or stamp < self.started_at:
                return
            if self._proof is not None and stamp < self._proof[0]:
                return
            self._proof = (stamp, bool(succeeded and confirmed))

    def consume(self, now):
        with self.lock:
            if (not self.active or self._bool_at is None or self._proof is None
                    or not self._proof[1] or not self._fresh(self._proof[0], now)
                    or not self._fresh(self._bool_at, now)):
                return False
            self.completed.add(self.current_floor)
            self.active = False
            self._bool_at = None
            return True

    @property
    def remaining(self):
        with self.lock:
            return tuple(sorted(self.required - self.completed))

    def _fresh(self, stamp, now):
        return (math.isfinite(stamp) and math.isfinite(now)
                and stamp > 0 and 0 <= now - stamp <= self.max_age)
