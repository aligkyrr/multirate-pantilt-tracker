import math

from PyQt5.QtCore import Qt

import config


class _MouseInteractionMixin:
    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        click_x, click_y = event.pos().x(), event.pos().y()

        if self._route_edit_mode:
            wx, wy = self._px_to_world(click_x, click_y)
            self.routePointAdded.emit(wx, wy)
            return

        best_id = None
        best_dist = None
        for (tid, x, y, _ttype, _heading) in self._targets:
            px, py, _scale = self._world_to_px(x, y)
            d = math.hypot(px - click_x, py - click_y)
            if d <= config.CLICK_TOLERANCE_PX and (best_dist is None or d < best_dist):
                best_dist = d
                best_id = tid

        if best_id is not None:
            self.targetClicked.emit(best_id)