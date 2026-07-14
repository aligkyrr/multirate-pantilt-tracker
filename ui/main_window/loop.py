import math
import time
import config

class _LoopMixin:
    def _tick_target_mode(self, dt: float = None):
        if dt is None:
            dt = config.DT_PANTILT
        self._select_active_target()
        active = self.target_manager.get(self.active_target_id)

        if active is not None:
            if self.lead_enabled:
                tx, ty = active.predicted_position(config.LEAD_TIME_SEC)
            else:
                tx, ty = active.x, active.y

            self._advance_tracker(tx, ty, dt, lock_log_id=f"T{active.id}")

            radar_targets = self._radar_targets(self.target_manager.targets)
            self.radar.update_scene(
                radar_targets, self.active_target_id, self.tracker.locked,
                self.tracker.azimuth, self._pulse_phase,
            )
        else:
            radar_targets = self._radar_targets(self.target_manager.targets)
            self.radar.update_scene(radar_targets, None, False, self.tracker.azimuth, 0.0)

        self.radar.update_route(self.route_manager.points, None)

    def _tick_route_mode(self, dt: float = None):
        if dt is None:
            dt = config.DT_PANTILT
        pos = self.route_manager.advance(dt)

        if pos is not None:
            self._advance_tracker(pos[0], pos[1], dt, lock_log_id="ROTA")
        else:
            self.control_panel.set_lock(False)
            self._was_locked = False

        radar_targets = self._radar_targets(self.target_manager.targets)
        self.radar.update_scene(
            radar_targets, None, self.tracker.locked if pos is not None else False,
            self.tracker.azimuth, self._pulse_phase,
        )
        self.radar.update_route(self.route_manager.points, pos)

    def _advance_tracker(self, tx: float, ty: float, dt: float, lock_log_id: str):
        prev_mode = self.tracker.mode
        az, el, v_az, v_el, des_az, des_el, err_az, err_el = self.tracker.step(
            tx, ty, self.pid_az, self.pid_el, dt
        )

        if self.tracker.mode != prev_mode:
            self.control_panel.log_mode_changed(frm=prev_mode, to=self.tracker.mode)
            self.control_panel.set_mode(self.tracker.mode)

        locked = self.tracker.locked
        if locked and not self._was_locked:
            dist = math.hypot(tx, ty)
            self.control_panel.log_target_locked(lock_log_id, dist=dist)
        elif (not locked) and self._was_locked:
            self.control_panel.log_target_lost(lock_log_id)
        self._was_locked = locked
        self.control_panel.set_lock(locked)

        self.graphs.push(err_az, err_el, v_az, v_el)
        self.gl_view.set_orientation(az, el)

        if locked:
            self._pulse_phase = (self._pulse_phase + dt * 1.4) % 1.0
        else:
            self._pulse_phase = 0.0

    def _advance_accumulator(self, last_time: float, dt_fixed: float, now: float):
        max_steps = config.MAX_CATCHUP_STEPS
        if (now - last_time) > config.MAX_DT * max_steps:
            last_time = now - dt_fixed

        steps = 0
        while (now - last_time) >= dt_fixed and steps < max_steps:
            last_time += dt_fixed
            steps += 1
        return last_time, steps

    def on_tick(self):
        self._update_fps()

        if self.paused:
            return

        now = time.time()

        self._t_last_target, n_target = self._advance_accumulator(
            self._t_last_target, config.DT_TARGET, now
        )
        for _ in range(n_target):
            self.target_manager.step(config.DT_TARGET)
            self._sim_time += config.DT_TARGET
        self._target_step_count += n_target

        self._t_last_control, n_control = self._advance_accumulator(
            self._t_last_control, config.DT_CONTROL, now
        )
        for _ in range(n_control):
            kp, ki, kd = self.control_panel.gains()
            for pid in (self.pid_az, self.pid_el):
                pid.kp, pid.ki, pid.kd = kp, ki, kd
        self._control_step_count += n_control

        self._t_last_pantilt, n_pantilt = self._advance_accumulator(
            self._t_last_pantilt, config.DT_PANTILT, now
        )
        for _ in range(n_pantilt):
            if self.tracking_mode == config.TRACKING_MODE_ROUTE:
                self._tick_route_mode(config.DT_PANTILT)
            else:
                self._tick_target_mode(config.DT_PANTILT)
        self._pantilt_step_count += n_pantilt

        self._debug_real_hz(now)

    def _debug_real_hz(self, now: float):
        elapsed = now - self._debug_last_time
        if elapsed >= 1.0:
            target_hz = self._target_step_count / elapsed
            control_hz = self._control_step_count / elapsed
            pantilt_hz = self._pantilt_step_count / elapsed
            print(
                f"[DEBUG real_hz] target={target_hz:6.1f} (hedef {config.TARGET_UPDATE_HZ:.0f})  "
                f"control={control_hz:6.1f} (hedef {config.CONTROL_LOOP_HZ:.0f})  "
                f"pantilt={pantilt_hz:6.1f} (hedef {config.PANTILT_UPDATE_HZ:.0f})"
            )
            self.graphs.set_real_hz(target_hz, control_hz, pantilt_hz)
            self._target_step_count = 0
            self._control_step_count = 0
            self._pantilt_step_count = 0
            self._debug_last_time = now

    def _update_fps(self):
        self._fps_frame_count += 1
        now = time.perf_counter()
        elapsed = now - self._fps_last_time
        if elapsed >= 0.5:
            self._fps_value = self._fps_frame_count / elapsed
            self._fps_frame_count = 0
            self._fps_last_time = now
            self.control_panel.set_fps(self._fps_value)