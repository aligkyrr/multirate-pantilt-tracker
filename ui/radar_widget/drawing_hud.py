import math

from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QColor, QPen, QBrush, QPainter

from .style import _mono_font, _COLOR_LOCK, _COLOR_ACCENT, _COLOR_TEXT, _COLOR_STATUS_LOCK_TEXT

import config


class _HudDrawingMixin:
    def _draw_crosshair(self, p: QPainter):
        w = self.width()
        h = self.height()
        cx, cy = w / 2.0, h / 2.0
        arm = 9
        gap = 4

        color = QColor(_COLOR_ACCENT)
        color.setAlpha(140)
        pen = QPen(color)
        pen.setWidth(1)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)

        p.drawLine(int(cx - arm), int(cy), int(cx - gap), int(cy))
        p.drawLine(int(cx + gap), int(cy), int(cx + arm), int(cy))
        p.drawLine(int(cx), int(cy - arm), int(cx), int(cy - gap))
        p.drawLine(int(cx), int(cy + gap), int(cx), int(cy + arm))
        p.drawEllipse(QPointF(cx, cy), gap * 1.3, gap * 1.3)

    def _draw_lock_tint(self, p: QPainter):
        if not self._locked:
            return
        color = QColor(_COLOR_LOCK)
        color.setAlpha(6)
        p.fillRect(self.rect(), color)

    def _draw_status_bar(self, p: QPainter):
        if self._locked:
            text = "STATUS: LOCKED"
            color = QColor(_COLOR_STATUS_LOCK_TEXT)
        else:
            text = "STATUS: TRACKING"
            color = QColor(_COLOR_TEXT)
            color.setAlpha(150)

        p.setFont(_mono_font(9, bold=self._locked))
        p.setPen(QPen(color))
        metrics = p.fontMetrics()
        tw = metrics.horizontalAdvance(text)
        margin = 35
        p.drawText(int(self.width() - tw - margin), int(margin + metrics.ascent()), text)

    def _draw_bottom_status_bar(self, p: QPainter):
        active_target = None
        for (tid, x, y, _ttype, _heading) in self._targets:
            if tid == self._active_id:
                active_target = (tid, x, y)
                break

        if active_target is not None:
            tid, x, y = active_target
            target_str = f"T{tid}"
            dist_str = f"{math.hypot(x, y):.1f}m"
        else:
            target_str = "--"
            dist_str = "--"

        state_str = "LOCKED" if self._locked else "TRACKING"
        fps_str = f"{self._fps_ema:.0f}" if self._fps_ema > 0 else "--"

        text = (f"MODE: {self._mode} | TARGET: {target_str} | "
                f"DIST: {dist_str} | STATE: {state_str} | FPS: {fps_str}")

        color = QColor(_COLOR_TEXT)
        color.setAlpha(160)
        p.setFont(_mono_font(8))
        p.setPen(QPen(color))

        metrics = p.fontMetrics()
        margin = 10
        y_pos = self.height() - margin
        p.drawText(int(margin), int(y_pos), text)

    def _draw_route_edit_hint(self, p: QPainter):
        if not self._route_edit_mode:
            return
        p.setFont(_mono_font(9, bold=True))
        p.setPen(QPen(QColor(config.COLOR_ROUTE_EDIT_HINT)))
        p.drawText(10, 20, "ROTA DÜZENLEME MODU — tıklayarak waypoint ekleyin")
