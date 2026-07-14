import math

from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QColor, QPen, QBrush, QPolygonF, QPainter

from .style import _JET_SHAPE, _mono_font, _COLOR_LOCK, _COLOR_ACCENT, _COLOR_TEXT

import config


class _TargetDrawingMixin:
    def _draw_targets(self, p: QPainter):
        for (tid, x, y, ttype, heading_deg) in self._targets:
            px, py, scale = self._world_to_px(x, y)
            radius_px = max(6, config.TARGET_RADIUS_M * scale)
            color = QColor(config.COLOR_TARGET_TYPES.get(ttype, config.COLOR_TARGET_NORMAL))

            is_active = (tid == self._active_id)

            if is_active and self._locked:
                self._draw_lock_ring(p, px, py, radius_px)

            if is_active:
                self._draw_active_ring(p, px, py, radius_px)

            self._draw_jet_icon(p, px, py, radius_px, heading_deg, color, is_active)
            self._draw_target_label(p, px, py, radius_px, tid, x, y)

    def _draw_lock_ring(self, p: QPainter, px: float, py: float, radius_px: float):
        ring_r = radius_px * 2.0
        color = QColor(_COLOR_LOCK)
        color.setAlpha(150)
        pen = QPen(color)
        pen.setWidthF(3.4)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(int(px - ring_r), int(py - ring_r), int(2 * ring_r), int(2 * ring_r))

    def _draw_active_ring(self, p: QPainter, px: float, py: float, radius_px: float):
        color = QColor(_COLOR_ACCENT)
        color.setAlpha(170)
        pen = QPen(color)
        pen.setWidth(1)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        ring_r = radius_px * 1.55
        p.drawEllipse(int(px - ring_r), int(py - ring_r), int(2 * ring_r), int(2 * ring_r))

        center_r = 2
        p.setPen(QPen(color, 1))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(px, py), center_r, center_r)

    def _draw_jet_icon(self, p: QPainter, px: float, py: float, radius_px: float,
                        heading_deg: float, color: QColor, is_active: bool = False):
        size = radius_px * 1.7

        p.save()
        p.translate(px, py)
        p.rotate(heading_deg)

        poly = QPolygonF([QPointF(lx * size, ly * size) for (lx, ly) in _JET_SHAPE])

        if is_active:
            outline = QColor(_COLOR_ACCENT)
            p.setPen(QPen(outline, 1.3))
        else:
            p.setPen(QPen(color.darker(160), 1))
        p.setBrush(QBrush(color))

        p.drawPolygon(poly)

        p.restore()

    def _draw_target_label(self, p: QPainter, px: float, py: float, radius_px: float,
                            tid: int, x: float, y: float):
        distance = math.hypot(x, y)

        label = f"T{tid}"
        dist  = f"{distance:4.1f}m"

        base_x = int(px + radius_px + 4)
        base_y = int(py - radius_px * 0.5)

        font_id = _mono_font(7)
        font_id.setBold(True)
        p.setFont(font_id)
        p.setPen(QPen(QColor(_COLOR_TEXT)))
        p.drawText(base_x, base_y, label)

        font_dist = _mono_font(6)
        p.setFont(font_dist)
        p.setPen(QPen(QColor(150, 150, 150)))
        p.drawText(base_x + 26, base_y, dist)