from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QListWidgetItem

import config


class _ControlPanelApiMixin:
    def log_target_selected(self, target_id, mode: str = "manual"):
        self.logger.log("INFO", "TARGET_SELECTED", id=target_id, mode=mode)

    def log_target_locked(self, target_id, dist: float = None):
        kwargs = {"id": target_id}
        if dist is not None:
            kwargs["dist"] = f"{dist:.1f}m"
        self.logger.log("INFO", "TARGET_LOCKED", **kwargs)

    def log_target_lost(self, target_id):
        self.logger.log("WARNING", "TARGET_LOST", id=target_id)

    def log_mode_changed(self, frm: str, to: str):
        self.logger.log("INFO", "MODE_CHANGED", frm=frm, to=to)

    def log_error(self, event: str, **kwargs):
        self.logger.log("ERROR", event, **kwargs)

    def set_pause_label(self, paused: bool):
        self.btn_pause.setText("Resume" if paused else "Pause")

    def set_mode(self, mode: str):
        self.mode_label.setText(mode)
        if mode == "FINE":
            self.mode_label.setStyleSheet("font-weight: bold; color: #28c76f;")
        else:
            self.mode_label.setStyleSheet("font-weight: bold; color: #ff9f43;")

    def set_lock(self, locked: bool):
        if locked:
            self.lock_label.setText("● LOCKED")
            self.lock_label.setStyleSheet(f"font-weight: bold; color: {config.COLOR_LOCK};")
        else:
            self.lock_label.setText("● NO LOCK")
            self.lock_label.setStyleSheet("font-weight: bold; color: #7a828c;")

    def set_fps(self, fps: float):
        self.fps_label.setText(f"FPS: {fps:.1f}")

    def refresh_target_list(self, targets, active_id):
        self._active_id = active_id
        self.target_list.blockSignals(True)
        self.target_list.clear()
        current_item = None
        for tid, ttype in targets:
            marker = "🎯 " if tid == active_id else "   "
            item = QListWidgetItem(f"{marker}T{tid}  [{ttype}]")
            item.setData(Qt.UserRole, tid)
            if tid == active_id:
                item.setSelected(True)
                current_item = item
                color = config.COLOR_TARGET_TYPES.get(ttype, config.COLOR_TARGET_NORMAL)
                item.setForeground(QColor(color))
            self.target_list.addItem(item)
        if current_item is not None:
            self.target_list.setCurrentItem(current_item)
        self.target_list.blockSignals(False)

    def gains(self):
        return self.s_kp.value(), self.s_ki.value(), self.s_kd.value()

    def set_route_count(self, n: int):
        self.route_count_label.setText(f"Waypoint: {n}")

    def set_route_edit_checked(self, checked: bool):
        self.btn_route_edit.blockSignals(True)
        self.btn_route_edit.setChecked(checked)
        self.btn_route_edit.setText("🖊 Rota Düzenle: AÇIK" if checked else "🖊 Rota Düzenle: KAPALI")
        self.btn_route_edit.blockSignals(False)