from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidgetItem

import config


class _ControlPanelInteractionsMixin:
    def _on_add(self):
        self.addTargetRequested.emit(self.type_combo.currentText())

    def _on_remove(self):
        item = self.target_list.currentItem()
        if item is not None:
            tid = item.data(Qt.UserRole)
        elif self._active_id is not None:
            tid = self._active_id
        else:
            return
        self.removeTargetRequested.emit(tid)

    def _on_list_click(self, item: QListWidgetItem):
        tid = item.data(Qt.UserRole)
        self.targetSelected.emit(tid)

    def _on_auto_toggle(self, checked: bool):
        self.auto_mode_label.setText("AUTO" if checked else "MANUAL")
        color = config.COLOR_TOGGLE_ON if checked else config.COLOR_TOGGLE_OFF
        self.auto_mode_label.setStyleSheet(f"font-weight: bold; color: {color};")
        self.autoModeToggled.emit(checked)

    def _on_strategy_change(self, _idx):
        self.autoStrategyChanged.emit(self.strategy_combo.currentData())

    def _on_tracking_mode_toggle(self, checked: bool):
        self.tracking_mode_label.setText("ROTA" if checked else "HEDEF TAKİP")
        color = config.COLOR_TOGGLE_ON if checked else config.COLOR_TOGGLE_OFF
        self.tracking_mode_label.setStyleSheet(f"font-weight: bold; color: {color};")
        self.trackingModeToggled.emit(checked)

    def _on_route_edit_toggle(self, checked: bool):
        self.btn_route_edit.setText("🖊 Rota Düzenle: AÇIK" if checked else "🖊 Rota Düzenle: KAPALI")
        self.routeEditToggled.emit(checked)

    def _on_loop_mode_change(self, _idx):
        self.routeLoopModeChanged.emit(self.loop_combo.currentData())