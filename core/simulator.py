import math

import config


class PanTiltTracker:
    def __init__(self):
        self.azimuth = 0.0
        self.elevation = 0.0
        self.mode = "COARSE"
        self.locked = False

        self._fine_confirm_ticks = 0
        self._coarse_confirm_ticks = 0
        self._lock_confirm_ticks = 0
        self._unlock_confirm_ticks = 0

        self.accel_limit_enabled = config.ACCEL_LIMIT_ENABLED_DEFAULT
        self.max_accel_az = config.MAX_ACCEL_AZ_DEFAULT_DEG_S2
        self.max_accel_el = config.MAX_ACCEL_EL_DEFAULT_DEG_S2

        self._last_v_az = 0.0
        self._last_v_el = 0.0

    def reset(self):
        self.azimuth = 0.0
        self.elevation = 0.0
        self.mode = "COARSE"
        self.locked = False
        self._fine_confirm_ticks = 0
        self._coarse_confirm_ticks = 0
        self._lock_confirm_ticks = 0
        self._unlock_confirm_ticks = 0
        self._last_v_az = 0.0
        self._last_v_el = 0.0

    # ------------------------------------------------------------
    @staticmethod
    def _wrap_angle_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def desired_angles(tx: float, ty: float):
        distance = math.hypot(tx, ty)
        distance = max(distance, 1e-6)
        azimuth = math.degrees(math.atan2(tx, ty))
        elevation = math.degrees(math.atan2(-config.CAMERA_HEIGHT, distance))
        return azimuth, elevation

    # ------------------------------------------------------------
    def step(self, tx: float, ty: float, pid_az: "PIDController", pid_el: "PIDController", dt: float):
        des_az, des_el = self.desired_angles(tx, ty)
        err_az = self._wrap_angle_deg(des_az - self.azimuth)
        err_el = des_el - self.elevation

        mode_threshold = (
            config.COARSE_REENTRY_THRESHOLD_DEG if self.mode == "FINE"
            else config.TRACK_ANGLE_THRESHOLD_DEG
        )
        within_fine_band = abs(err_az) <= mode_threshold and abs(err_el) <= mode_threshold

        if self.mode == "COARSE":
            self._fine_confirm_ticks = self._fine_confirm_ticks + 1 if within_fine_band else 0
            if self._fine_confirm_ticks >= config.MODE_SWITCH_CONFIRM_TICKS:
                self.mode = "FINE"
                self._fine_confirm_ticks = 0
                self._coarse_confirm_ticks = 0
        else:
            self._coarse_confirm_ticks = 0 if within_fine_band else self._coarse_confirm_ticks + 1
            if self._coarse_confirm_ticks >= config.MODE_SWITCH_CONFIRM_TICKS:
                self.mode = "COARSE"
                self._coarse_confirm_ticks = 0
                self._fine_confirm_ticks = 0

        if self.mode == "FINE":
            v_az = pid_az.update(err_az, dt)
            v_el = pid_el.update(err_el, dt)
        else:
            v_az = max(-config.COARSE_MAX_SPEED_DEG_PER_TICK, min(config.COARSE_MAX_SPEED_DEG_PER_TICK, err_az))
            v_el = max(-config.COARSE_MAX_SPEED_DEG_PER_TICK, min(config.COARSE_MAX_SPEED_DEG_PER_TICK, err_el))
            pid_az.sync_error(err_az)
            pid_el.sync_error(err_el)

        if self.accel_limit_enabled:
            v_az = self._apply_accel_limit(v_az, self._last_v_az, self.max_accel_az, dt, error_deg=err_az)
            v_el = self._apply_accel_limit(v_el, self._last_v_el, self.max_accel_el, dt, error_deg=err_el)

        self._last_v_az = v_az
        self._last_v_el = v_el
        self.azimuth += v_az
        self.azimuth = self._wrap_angle_deg(self.azimuth)
        self.elevation += v_el
        self.elevation = max(config.ELEVATION_DEG_MIN, min(config.ELEVATION_DEG_MAX, self.elevation))

        lock_threshold = (
            config.LOCK_EXIT_THRESHOLD_DEG if self.locked
            else config.LOCK_ANGLE_THRESHOLD_DEG
        )
        within_lock_band = abs(err_az) <= lock_threshold and abs(err_el) <= lock_threshold

        if not self.locked:
            self._lock_confirm_ticks = self._lock_confirm_ticks + 1 if within_lock_band else 0
            if self._lock_confirm_ticks >= config.LOCK_CONFIRM_TICKS:
                self.locked = True
                self._lock_confirm_ticks = 0
                self._unlock_confirm_ticks = 0
        else:
            self._unlock_confirm_ticks = 0 if within_lock_band else self._unlock_confirm_ticks + 1
            if self._unlock_confirm_ticks >= config.LOCK_CONFIRM_TICKS:
                self.locked = False
                self._unlock_confirm_ticks = 0
                self._lock_confirm_ticks = 0

        return self.azimuth, self.elevation, v_az, v_el, des_az, des_el, err_az, err_el

    @staticmethod
    def _apply_accel_limit(v_new: float, v_last: float, max_accel_deg_s2: float, dt: float,
                            error_deg: float = None) -> float:

        if max_accel_deg_s2 is None or max_accel_deg_s2 <= 0 or dt <= 0:
            return v_new

        if error_deg is not None:
            v_brake_max_deg_s = math.sqrt(max(0.0, 2.0 * max_accel_deg_s2 * abs(error_deg)))
            v_brake_max_deg_tick = v_brake_max_deg_s * dt
            v_new = max(-v_brake_max_deg_tick, min(v_brake_max_deg_tick, v_new))

        max_dv = max_accel_deg_s2 * dt * dt
        dv = v_new - v_last
        if dv > max_dv:
            dv = max_dv
        elif dv < -max_dv:
            dv = -max_dv
        return v_last + dv

    def aim_point(self, ref_y: float):
        ax = ref_y * math.tan(math.radians(self.azimuth))
        return ax, ref_y

    def aim_direction_vector(self):
        rad = math.radians(self.azimuth)
        return math.sin(rad), math.cos(rad)
