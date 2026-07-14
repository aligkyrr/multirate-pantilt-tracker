import math
import random

import config


class Target:
    def __init__(self, target_id: int, x: float, y: float, ttype: str = "normal"):
        self.id = target_id
        self.type = ttype if ttype in config.TARGET_TYPES else config.DEFAULT_TARGET_TYPE

        self.x = x
        self.y = y

        angle0 = random.uniform(0, 2 * math.pi)
        vmin, vmax = config.TARGET_SPEED_RANGE[self.type]
        speed0 = random.uniform(vmin, vmax)
        self.vx = speed0 * math.cos(angle0)
        self.vy = speed0 * math.sin(angle0)

        self.ax = 0.0
        self.ay = 0.0

        self._retarget_timer = random.uniform(0.5, 1.5)
        self._drift_angle = angle0

    # ------------------------------------------------------------
    def step(self, dt: float):
        vmin, vmax = config.TARGET_SPEED_RANGE[self.type]
        noise_scale = config.TARGET_NOISE_SCALE[self.type]
        damping = config.TARGET_ACCEL_DAMPING

        self._retarget_timer -= dt
        if self._retarget_timer <= 0.0:
            self._retarget_timer = random.uniform(0.6, 1.8)
            self._drift_angle += random.uniform(-1.2, 1.2)

        target_ax = noise_scale * math.cos(self._drift_angle) + random.uniform(-0.4, 0.4)
        target_ay = noise_scale * math.sin(self._drift_angle) + random.uniform(-0.4, 0.4)

        self.ax += (target_ax - self.ax) * min(1.0, damping * dt)
        self.ay += (target_ay - self.ay) * min(1.0, damping * dt)

        self.vx += self.ax * dt
        self.vy += self.ay * dt

        speed = math.hypot(self.vx, self.vy)
        if speed > vmax:
            scale = vmax / speed
            self.vx *= scale
            self.vy *= scale
        elif speed < vmin * 0.4 and speed > 1e-6:
            scale = (vmin * 0.4) / speed
            self.vx *= scale
            self.vy *= scale

        self.x += self.vx * dt
        self.y += self.vy * dt

        self._bounce(config.MAP_BOUNDS)

    def _bounce(self, bounds):
        xmin, xmax, ymin, ymax = bounds
        damp = config.TARGET_BOUNCE_DAMPING

        if self.x < xmin:
            self.x = xmin
            self.vx = -self.vx * damp
            self._drift_angle = math.pi - self._drift_angle
        elif self.x > xmax:
            self.x = xmax
            self.vx = -self.vx * damp
            self._drift_angle = math.pi - self._drift_angle

        if self.y < ymin:
            self.y = ymin
            self.vy = -self.vy * damp
            self._drift_angle = -self._drift_angle
        elif self.y > ymax:
            self.y = ymax
            self.vy = -self.vy * damp
            self._drift_angle = -self._drift_angle

    def predicted_position(self, lead_time: float):
        return self.x + self.vx * lead_time, self.y + self.vy * lead_time

    def distance_to_origin(self) -> float:
        return math.hypot(self.x, self.y)

    def azimuth_from_origin(self) -> float:
        return math.degrees(math.atan2(self.x, self.y))


class TargetManager:
    def __init__(self):
        self._next_id = 1
        self.targets: list[Target] = []
        self._active_target_id = None
        self._cooldown_remaining = 0.0

    def add_target(self, ttype: str = None, x: float = None, y: float = None) -> Target:
        if len(self.targets) >= config.MAX_TARGETS:
            return None
        ttype = ttype or config.DEFAULT_TARGET_TYPE
        xmin, xmax, ymin, ymax = config.MAP_BOUNDS
        if x is None:
            x = random.uniform(xmin * 0.7, xmax * 0.7)
        if y is None:
            y = random.uniform(ymin + 1.0, ymax * 0.85)

        t = Target(self._next_id, x, y, ttype)
        self._next_id += 1
        self.targets.append(t)
        return t

    def remove_target(self, target_id: int) -> bool:
        if len(self.targets) <= config.MIN_TARGETS:
            return False
        before = len(self.targets)
        self.targets = [t for t in self.targets if t.id != target_id]
        removed = len(self.targets) < before
        if removed and self._active_target_id == target_id:
            self._active_target_id = None
            self._cooldown_remaining = 0.0
        return removed

    def remove_last(self) -> bool:
        if len(self.targets) <= config.MIN_TARGETS:
            return False
        removed = self.targets.pop()
        if removed is not None and self._active_target_id == removed.id:
            self._active_target_id = None
            self._cooldown_remaining = 0.0
        return removed is not None

    def get(self, target_id: int):
        for t in self.targets:
            if t.id == target_id:
                return t
        return None

    def step(self, dt: float):
        for t in self.targets:
            t.step(dt)
        if self._cooldown_remaining > 0.0:
            self._cooldown_remaining = max(0.0, self._cooldown_remaining - dt)

    def nearest_to_origin(self):
        if not self.targets:
            return None
        return min(self.targets, key=lambda t: t.distance_to_origin())

    def nearest_to_center(self):
        if not self.targets:
            return None
        return min(self.targets, key=lambda t: abs(t.azimuth_from_origin()))

    @staticmethod
    def _score(target: "Target", strategy: str) -> float:
        if strategy == config.AUTO_STRATEGY_CENTER:
            return abs(target.azimuth_from_origin())
        return target.distance_to_origin()

    @staticmethod
    def _margin_for(strategy: str) -> float:
        if strategy == config.AUTO_STRATEGY_CENTER:
            return config.AUTO_SWITCH_MARGIN_DEG
        return config.AUTO_SWITCH_MARGIN_M

    def pick_auto(self, strategy: str):
        if not self.targets:
            self._active_target_id = None
            return None

        candidates = {t.id: t for t in self.targets}
        active = candidates.get(self._active_target_id)

        if active is None:
            best = min(self.targets, key=lambda t: self._score(t, strategy))
            self._active_target_id = best.id
            self._cooldown_remaining = config.AUTO_SWITCH_COOLDOWN_SEC
            return best

        best = min(self.targets, key=lambda t: self._score(t, strategy))
        if best.id == active.id:
            return active

        if self._cooldown_remaining > 0.0:
            return active

        margin = self._margin_for(strategy)
        if self._score(active, strategy) - self._score(best, strategy) < margin:
            return active

        self._active_target_id = best.id
        self._cooldown_remaining = config.AUTO_SWITCH_COOLDOWN_SEC
        return best

    def reset(self):
        self.targets.clear()
        self._next_id = 1
        self._active_target_id = None
        self._cooldown_remaining = 0.0
        for _ in range(config.INITIAL_TARGET_COUNT):
            ttype = random.choice(config.TARGET_TYPES)
            self.add_target(ttype=ttype)