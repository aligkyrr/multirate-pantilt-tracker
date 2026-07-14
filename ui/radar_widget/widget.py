import time

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QPainter, QColor
from PyQt5.QtWidgets import QWidget

import config

from .coordinates import _CoordinateMixin
from .drawing_grid import _GridDrawingMixin
from .drawing_targets import _TargetDrawingMixin
from .drawing_route import _RouteDrawingMixin
from .drawing_hud import _HudDrawingMixin
from .interaction import _MouseInteractionMixin
from .style import _COLOR_BG


class RadarWidget(QWidget, _CoordinateMixin, _GridDrawingMixin, _TargetDrawingMixin,
                   _RouteDrawingMixin, _HudDrawingMixin, _MouseInteractionMixin):

    targetClicked = pyqtSignal(int)  # target id
    routePointAdded = pyqtSignal(float, float)  # dünya koordinatı (x, y)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(300)
        self.setMouseTracking(True)

        self._targets = []          # [(id, x, y, type, heading_deg), ...]
        self._active_id = None
        self._locked = False
        self._aim_azimuth_deg = 0.0
        self._pulse_phase = 0.0    

        self._route_edit_mode = False
        self._route_points = []     # [(x, y), ...]
        self._route_current_pos = None

        self._mode = "MANUAL"       # main_window update_scene(mode=...) ile güncellenebilir

        self._last_paint_ts = None
        self._fps_ema = 0.0

        self._xmin, self._xmax, self._ymin, self._ymax = config.MAP_BOUNDS

    def update_scene(self, targets, active_id, locked, aim_azimuth_deg, pulse_phase, mode=None):
        self._targets = targets
        self._active_id = active_id
        self._locked = locked
        self._aim_azimuth_deg = aim_azimuth_deg
        self._pulse_phase = pulse_phase
        if mode is not None:
            self._mode = mode
        self.update()

    def update_route(self, points, current_pos):
        self._route_points = points
        self._route_current_pos = current_pos
        self.update()

    def set_route_edit_mode(self, enabled: bool):
        self._route_edit_mode = enabled
        self.update()

    def paintEvent(self, _event):
        now = time.time()
        if self._last_paint_ts is not None:
            dt = now - self._last_paint_ts
            if dt > 0:
                inst_fps = 1.0 / dt
                alpha = 0.15
                self._fps_ema = (alpha * inst_fps + (1 - alpha) * self._fps_ema
                                  if self._fps_ema > 0 else inst_fps)
        self._last_paint_ts = now

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(_COLOR_BG))

        self._draw_grid(p)
        self._draw_origin(p)
        self._draw_aim_line(p)
        self._draw_targets(p)
        self._draw_route(p)
        self._draw_crosshair(p)
        self._draw_lock_tint(p)
        self._draw_status_bar(p)
        self._draw_bottom_status_bar(p)
        self._draw_route_edit_hint(p)

        p.end()
