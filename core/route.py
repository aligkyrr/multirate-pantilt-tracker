import math
import config

class RouteManager:
    def __init__(self):
        self.points: list[tuple[float, float]] = []
        self.loop_mode = config.DEFAULT_ROUTE_LOOP_MODE
        self.speed = config.ROUTE_DEFAULT_SPEED_MPS
        self._seg_index = 0
        self._seg_progress = 0.0
        self._direction = 1
        self.finished = False

    def add_point(self, x: float, y: float):
        self.points.append((x, y))
        self.finished = False

    def clear(self):
        self.points = []
        self.reset_progress()

    def undo_last(self):
        if self.points:
            self.points.pop()
        self.reset_progress()

    def reset_progress(self):
        self._seg_index = 0
        self._seg_progress = 0.0
        self._direction = 1
        self.finished = False

    def set_loop_mode(self, mode: str):
        self.loop_mode = mode
        self.finished = False

    def set_speed(self, speed_mps: float):
        self.speed = max(0.01, speed_mps)

    def has_route(self) -> bool:
        return len(self.points) > 0

    def advance(self, dt: float):
        if not self.points:
            return None
        if len(self.points) == 1:
            return self.points[0]
        if self.finished:
            return self.points[self._seg_index]

        remaining = self.speed * dt
        max_iters = len(self.points) * 4 + 4
        iters = 0

        while remaining > 1e-9 and iters < max_iters:
            iters += 1
            i = self._seg_index
            j = i + self._direction
            p_i = self.points[i]
            p_j = self.points[j]
            seg_len = math.hypot(p_j[0] - p_i[0], p_j[1] - p_i[1])
            remaining_in_seg = max(seg_len - self._seg_progress, 0.0)

            if remaining < remaining_in_seg:
                self._seg_progress += remaining
                remaining = 0.0
                break

            remaining -= remaining_in_seg
            self._seg_progress = 0.0
            self._seg_index = j

            reached_end = (self._direction == 1 and j == len(self.points) - 1)
            reached_start = (self._direction == -1 and j == 0)

            if reached_end or reached_start:
                if self.loop_mode == config.ROUTE_LOOP_MODE_LOOP:
                    if reached_end:
                        self._seg_index = 0
                elif self.loop_mode == config.ROUTE_LOOP_MODE_PINGPONG:
                    self._direction *= -1
                else:
                    self.finished = True
                    remaining = 0.0
                    break

        return self._interpolated_point()

    def _interpolated_point(self):
        if self.finished:
            return self.points[self._seg_index]
        i = self._seg_index
        j = max(0, min(len(self.points) - 1, i + self._direction))
        p_i = self.points[i]
        p_j = self.points[j]
        seg_len = math.hypot(p_j[0] - p_i[0], p_j[1] - p_i[1])
        if seg_len < 1e-9:
            return p_i
        t = self._seg_progress / seg_len
        return (p_i[0] + (p_j[0] - p_i[0]) * t, p_i[1] + (p_j[1] - p_i[1]) * t)
