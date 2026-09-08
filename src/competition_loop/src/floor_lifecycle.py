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
