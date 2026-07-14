import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow

import config
from core.target import TargetManager
from core.route import RouteManager
from core.simulator import PanTiltTracker
from core.controller import PIDController
from ui.control_panel import ControlPanel
from ui.radar_widget import RadarWidget
from ui.graphs_widget import GraphsWidget
from visualization.gl_view import GLModelView

from .layout import _LayoutMixin
from .selection import _SelectionMixin
from .handlers import _HandlersMixin
from .loop import _LoopMixin


class MainWindow(QMainWindow, _LayoutMixin, _SelectionMixin, _HandlersMixin, _LoopMixin):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Çoklu Hedef Takip & Kilitlenme Sistemi — Pan/Tilt Smart Tracker")
        self.resize(1700, 980)
        self.setStyleSheet(f"background-color: {config.COLOR_BG};")

        self.target_manager = TargetManager()
        self.target_manager.reset()

        self.route_manager = RouteManager()
        self.tracking_mode = config.DEFAULT_TRACKING_MODE  # "target" veya "route"

        self.tracker = PanTiltTracker()
        self.pid_az = PIDController(
            config.PID_KP, config.PID_KI, config.PID_KD,
            output_limit=config.PID_OUTPUT_LIMIT, integral_limit=config.PID_INTEGRAL_LIMIT,
        )
        self.pid_el = PIDController(
            config.PID_KP, config.PID_KI, config.PID_KD,
            output_limit=config.PID_OUTPUT_LIMIT, integral_limit=config.PID_INTEGRAL_LIMIT,
        )

        self.paused = False
        self._dt = config.TICK_MS / 1000.0 

        _now = time.time()
        self._t_last_target = _now
        self._t_last_control = _now
        self._t_last_pantilt = _now

        self._target_step_count = 0
        self._control_step_count = 0
        self._pantilt_step_count = 0
        self._debug_last_time = _now

        self.auto_mode = True
        self.auto_strategy = config.DEFAULT_AUTO_STRATEGY
        self.lead_enabled = False
        self.active_target_id = None
        self._was_locked = False
        self._pulse_phase = 0.0

        self._sim_time = 0.0
        self._last_switch_time = -999.0

        self._fps_last_time = time.perf_counter()
        self._fps_frame_count = 0
        self._fps_value = 0.0

        self.control_panel = ControlPanel()
        self.radar = RadarWidget()
        self.gl_view = GLModelView()
        self.graphs = GraphsWidget()

        self._build_layout()
        self._connect_signals()
        self._pick_initial_active_target()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start(config.TICK_MS)
