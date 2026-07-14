import time
import config


class _HandlersMixin:
    def on_pause(self):
        self.paused = not self.paused
        self.control_panel.set_pause_label(self.paused)
        self.control_panel.logger.log(
            "INFO", "PAUSE_TOGGLED", state=("paused" if self.paused else "running")
        )

    def on_reset(self):
        self.target_manager.reset()
        self.tracker.reset()
        self.pid_az.reset()
        self.pid_el.reset()

        self.route_manager.reset_progress()
        self.radar.update_scene([], None, False, 0.0, 0.0)
        self.radar.update_route(self.route_manager.points, None)
        self.graphs.clear()
        self.control_panel.set_mode("COARSE")
        self.control_panel.set_lock(False)
        self._was_locked = False
        self._sim_time = 0.0
        self._last_switch_time = -999.0

        _now = time.time()
        self._t_last_target = _now
        self._t_last_control = _now
        self._t_last_pantilt = _now
        self.graphs.set_hz_values_silent(
            config.TARGET_UPDATE_HZ, config.CONTROL_LOOP_HZ, config.PANTILT_UPDATE_HZ
        )

        self._pick_initial_active_target()
        self.control_panel.logger.log("INFO", "SYSTEM_RESET")

    def on_add_target(self, ttype: str):
        t = self.target_manager.add_target(ttype=ttype)
        if t is not None:
            self.control_panel.logger.log("INFO", "TARGET_ADDED", id=t.id, type=ttype)
            self._refresh_target_list()
        else:
            self.control_panel.logger.log(
                "WARNING", "TARGET_ADD_FAILED", reason="max_targets", max=config.MAX_TARGETS
            )

    def on_remove_target(self, target_id: int):
        ok = self.target_manager.remove_target(target_id)
        if ok:
            self.control_panel.logger.log("INFO", "TARGET_REMOVED", id=target_id)
            if self.active_target_id == target_id:
                self.active_target_id = None
                self._was_locked = False
            self._refresh_target_list()
        else:
            self.control_panel.logger.log(
                "WARNING", "TARGET_REMOVE_FAILED", reason="min_targets"
            )

    def on_manual_select(self, target_id: int):
        if self.target_manager.get(target_id) is None:
            return
        if self.auto_mode:
            self.auto_mode = False
            self.control_panel.btn_auto_manual.setChecked(False)
            self.control_panel.auto_mode_label.setText("MANUAL")
            self.control_panel.auto_mode_label.setStyleSheet("font-weight: bold; color: #ff9f43;")

        if target_id != self.active_target_id:
            self.active_target_id = target_id
            self._was_locked = False
            self.tracker.mode = "COARSE"
            self.control_panel.log_target_selected(target_id, mode="manual")
            self._refresh_target_list()

    def on_auto_toggle(self, is_auto: bool):
        old_mode = "AUTO" if self.auto_mode else "MANUAL"
        self.auto_mode = is_auto
        new_mode = "AUTO" if is_auto else "MANUAL"
        self.control_panel.log_mode_changed(frm=old_mode, to=new_mode)

    def on_strategy_change(self, strategy: str):
        self.auto_strategy = strategy
        

    def on_lead_toggle(self, enabled: bool):
        self.lead_enabled = enabled
        self.control_panel.logger.log(
            "INFO", "LEAD_PREDICTION_TOGGLED", state=("on" if enabled else "off")
        )

    def on_tracking_mode_toggle(self, is_route: bool):
        old_mode = "ROUTE" if self.tracking_mode == config.TRACKING_MODE_ROUTE else "TARGET"
        self.tracking_mode = config.TRACKING_MODE_ROUTE if is_route else config.TRACKING_MODE_TARGET

        self._was_locked = False
        self.tracker.mode = "COARSE"
        self.control_panel.set_mode("COARSE")
        self.control_panel.set_lock(False)

        new_mode = "ROUTE" if is_route else "TARGET"
        self.control_panel.log_mode_changed(frm=old_mode, to=new_mode)

        if not is_route:
            self.control_panel.set_route_edit_checked(False)
            self.radar.set_route_edit_mode(False)
            self._refresh_target_list()

    def on_route_loop_mode_change(self, mode: str):
        self.route_manager.set_loop_mode(mode)

    def on_route_speed_change(self, speed: float):
        self.route_manager.set_speed(speed)

    def on_route_clear(self):
        self.route_manager.clear()
        self.control_panel.set_route_count(0)
        self.control_panel.logger.log("INFO", "ROUTE_CLEARED")

    def on_route_undo(self):
        if not self.route_manager.points:
            return
        self.route_manager.undo_last()
        self.control_panel.set_route_count(len(self.route_manager.points))
        self.control_panel.logger.log(
            "INFO", "WAYPOINT_REMOVED", remaining=len(self.route_manager.points)
        )

    def on_route_point_added(self, x: float, y: float):
        x = max(config.MAP_X_MIN, min(config.MAP_X_MAX, x))
        y = max(config.MAP_Y_MIN, min(config.MAP_Y_MAX, y))
        self.route_manager.add_point(x, y)
        self.control_panel.set_route_count(len(self.route_manager.points))
        self.control_panel.logger.log(
            "INFO", "WAYPOINT_ADDED",
            idx=len(self.route_manager.points), x=f"{x:.1f}", y=f"{y:.1f}",
        )

    def on_accel_limit_toggle(self, enabled: bool):
        self.tracker.accel_limit_enabled = enabled
        self.control_panel.logger.log(
            "INFO", "ACCEL_LIMIT_TOGGLED", state=("on" if enabled else "off")
        )

    def on_accel_az_change(self, value: float):
        self.tracker.max_accel_az = max(config.ACCEL_LIMIT_MIN_DEG_S2, value)

    def on_accel_el_change(self, value: float):
        self.tracker.max_accel_el = max(config.ACCEL_LIMIT_MIN_DEG_S2, value)

    def on_target_hz_change(self, hz: float):
        hz = max(1.0, hz)
        config.TARGET_UPDATE_HZ = hz
        config.DT_TARGET = 1.0 / hz
        self.control_panel.logger.log("INFO", "TARGET_HZ_CHANGED", hz=f"{hz:.1f}")

    def on_control_hz_change(self, hz: float):
        hz = max(1.0, hz)
        config.CONTROL_LOOP_HZ = hz
        config.DT_CONTROL = 1.0 / hz
        self.control_panel.logger.log("INFO", "CONTROL_HZ_CHANGED", hz=f"{hz:.1f}")

    def on_pantilt_hz_change(self, hz: float):
        hz = max(1.0, hz)
        config.PANTILT_UPDATE_HZ = hz
        config.DT_PANTILT = 1.0 / hz
        self.control_panel.logger.log("INFO", "PANTILT_HZ_CHANGED", hz=f"{hz:.1f}")
