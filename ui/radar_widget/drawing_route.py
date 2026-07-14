import math

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPen, QBrush, QPainter

from .style import _mono_font, _COLOR_TEXT, _COLOR_AIM_LINE

import config


class _RouteDrawingMixin:
    def _draw_aim_line(self, p: QPainter):
        ox, oy, _ = self._world_to_px(0.0, 0.0)
        rad = math.radians(self._aim_azimuth_deg)
        length = max(self._ymax - self._ymin, self._xmax - self._xmin) * 1.2 * 0.7
        tx = length * math.sin(rad)
        ty = length * math.cos(rad)
        ex, ey, _ = self._world_to_px(tx, ty)

        color = QColor(_COLOR_AIM_LINE)
        color.setAlpha(103)
        pen = QPen(color)
        pen.setWidth(1)
        pen.setStyle(Qt.CustomDashLine)
        pen.setDashPattern([6, 5])
        p.setPen(pen)
        p.drawLine(int(ox), int(oy), int(ex), int(ey))

    def _draw_route(self, p: QPainter):
        if not self._route_points:
            return

        pts_px = [self._world_to_px(x, y)[:2] for (x, y) in self._route_points]

        route_color = QColor(config.COLOR_ROUTE_LINE)
        route_color.setAlpha(190)
        pen = QPen(route_color)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        for i in range(len(pts_px) - 1):
            x1, y1 = pts_px[i]
            x2, y2 = pts_px[i + 1]
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        p.setFont(_mono_font(8))
        r = config.ROUTE_WAYPOINT_RADIUS_PX
        for idx, (px, py) in enumerate(pts_px):
            p.setPen(QPen(QColor(config.COLOR_ROUTE_POINT)))
            p.setBrush(QBrush(QColor(config.COLOR_ROUTE_POINT)))
            p.drawRect(int(px - r), int(py - r), int(2 * r), int(2 * r))
            p.setPen(QPen(QColor(_COLOR_TEXT)))
            p.drawText(int(px + r + 3), int(py - r), f"W{idx + 1}")

        if self._route_current_pos is not None:
            cx, cy, _ = self._world_to_px(*self._route_current_pos)
            p.setPen(QPen(QColor(config.COLOR_ROUTE_CURRENT)))
            p.setBrush(QBrush(QColor(config.COLOR_ROUTE_CURRENT)))
            p.drawEllipse(int(cx - 6), int(cy - 6), 12, 12)
