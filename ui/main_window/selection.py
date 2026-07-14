import math
import config

class _SelectionMixin:
    def _pick_initial_active_target(self):
        t = self.target_manager.pick_auto(self.auto_strategy)
        self.active_target_id = t.id if t else None
        self._refresh_target_list()

    def _refresh_target_list(self):
        items = [(t.id, t.type) for t in self.target_manager.targets]
        self.control_panel.refresh_target_list(items, self.active_target_id)

    @staticmethod
    def _radar_targets(targets):
        out = []
        for t in targets:
            heading = math.degrees(math.atan2(t.vx, t.vy)) if (t.vx or t.vy) else 0.0
            out.append((t.id, t.x, t.y, t.type, heading))
        return out

    def _select_active_target(self):
        current = self.target_manager.get(self.active_target_id)

        if self.auto_mode:
            candidate = self._pick_auto_stable(current)
            new_id = candidate.id if candidate else None
            if new_id != self.active_target_id:
                self.active_target_id = new_id
                self._was_locked = False
                self.tracker.mode = "COARSE"
                self._last_switch_time = self._sim_time
                if new_id is not None:
                    self.control_panel.log_target_selected(new_id, mode="auto")
                self._refresh_target_list()
        elif current is None:
            candidate = self.target_manager.pick_auto(self.auto_strategy)
            self.active_target_id = candidate.id if candidate else None
            self._was_locked = False
            self._last_switch_time = self._sim_time
            if self.active_target_id is not None:
                self.control_panel.log_target_selected(self.active_target_id, mode="auto")
            self._refresh_target_list()

    def _strategy_metric(self, t):
        if self.auto_strategy == config.AUTO_STRATEGY_CENTER:
            return abs(t.azimuth_from_origin())
        return t.distance_to_origin()

    def _strategy_margin(self):
        if self.auto_strategy == config.AUTO_STRATEGY_CENTER:
            return config.AUTO_SWITCH_MARGIN_DEG
        return config.AUTO_SWITCH_MARGIN_M

    def _pick_auto_stable(self, current):
        candidate = self.target_manager.pick_auto(self.auto_strategy)
        if candidate is None:
            return None
        if current is None:
            return candidate
        if candidate.id == current.id:
            return candidate

        if (self._sim_time - self._last_switch_time) < config.AUTO_SWITCH_COOLDOWN_SEC:
            return current

        if self._strategy_metric(candidate) + self._strategy_margin() < self._strategy_metric(current):
            return candidate
        return current
