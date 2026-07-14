from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPen, QBrush, QPainter

from .style import _mono_font, _COLOR_GRID, _COLOR_AXIS_TEXT, _COLOR_ACCENT

import config


class _GridDrawingMixin:
    def _draw_grid(self, p: QPainter):
        sx, sy, margin, w, h = self._scales()
        step = 1.0 

        minor_color = QColor(_COLOR_GRID)
        minor_color.setAlpha(50)
        minor_pen = QPen(minor_color)
        minor_pen.setWidthF(0.6)

        major_color = QColor(_COLOR_GRID)
        major_color.setAlpha(150)
        major_pen = QPen(major_color)
        major_pen.setWidthF(1.1)

        cwx, cwy = self._px_to_world(w / 2.0, h / 2.0)

        major_x_values = []  
        major_y_values = []  

        def vline(k, x):
            is_major = (k % 5 == 0)
            p.setPen(major_pen if is_major else minor_pen)
            x1, y1, _ = self._world_to_px(x, self._ymin)
            x2, y2, _ = self._world_to_px(x, self._ymax)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))
            if is_major:
                major_x_values.append((x, x1))

        def hline(k, y):
            is_major = (k % 5 == 0)
            p.setPen(major_pen if is_major else minor_pen)
            x1, y1, _ = self._world_to_px(self._xmin, y)
            x2, y2, _ = self._world_to_px(self._xmax, y)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))
            if is_major:
                major_y_values.append((y, y1))

        k = 0
        while True:
            x = cwx + k * step
            if x > self._xmax:
                break
            if x >= self._xmin:
                vline(k, x)
            k += 1
        k = -1
        while True:
            x = cwx + k * step
            if x < self._xmin:
                break
            if x <= self._xmax:
                vline(k, x)
            k -= 1

        k = 0
        while True:
            y = cwy + k * step
            if y > self._ymax:
                break
            if y >= self._ymin:
                hline(k, y)
            k += 1
        k = -1
        while True:
            y = cwy + k * step
            if y < self._ymin:
                break
            if y <= self._ymax:
                hline(k, y)
            k -= 1

        x0, y0, _ = self._world_to_px(self._xmin, self._ymax)
        x1, y1, _ = self._world_to_px(self._xmax, self._ymin)
        border_color = QColor(_COLOR_GRID)
        border_color.setAlpha(130)
        border_pen = QPen(border_color)
        border_pen.setWidth(1)
        p.setPen(border_pen)
        p.setBrush(Qt.NoBrush)
        p.drawRect(int(x0), int(y0), int(x1 - x0), int(y1 - y0))

        self._draw_axis_numbers(p, major_x_values, major_y_values, margin, w, h)

    def _draw_axis_numbers(self, p: QPainter, major_x_values, major_y_values,
                            margin: float, w: int, h: int):
        p.setPen(QPen(QColor(_COLOR_AXIS_TEXT)))
        p.setFont(_mono_font(8))
        metrics = p.fontMetrics()

        clip_left = margin
        clip_right = w - margin
        clip_top = margin
        clip_bottom = h - margin

        center_offset_px = 3

        label_y = clip_top - 6
        for wx, px in major_x_values:
            if px < clip_left - 1 or px > clip_right + 1:
                continue
            val = int(round(wx))
            text = "0" if val == 0 else f"{val:+d}"
            tw = metrics.horizontalAdvance(text)
            tx = px - tw / 2.0
            if val == 0:
                tx += center_offset_px
            tx = max(min(tx, clip_right - tw), clip_left)
            p.drawText(int(tx), int(label_y), text)

        for wy, py in major_y_values:
            if py < clip_top - 1 or py > clip_bottom + 1:
                continue
            val = int(round(wy))
            text = "0" if val == 0 else f"{val:+d}"
            tw = metrics.horizontalAdvance(text)
            tx = max(clip_left - tw - 4, 2)
            ty = py + metrics.ascent() / 2.0 - 1
            if val == 0:
                ty -= center_offset_px
            p.drawText(int(tx), int(ty), text)

    def _draw_origin(self, p: QPainter):
        ox, oy, _ = self._world_to_px(0.0, 0.0)
        origin_color = QColor(getattr(config, "COLOR_ORIGIN", _COLOR_ACCENT))
        pen = QPen(origin_color)
        pen.setWidth(1)
        p.setPen(pen)
        p.setBrush(QBrush(origin_color))
        p.drawEllipse(int(ox) - 4, int(oy) - 4, 8, 8)
        p.setFont(_mono_font(8))
        p.drawText(int(ox) + 8, int(oy) - 8, "PAN/TILT")
