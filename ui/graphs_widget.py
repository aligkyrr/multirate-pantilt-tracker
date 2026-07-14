from collections import deque

import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QDoubleSpinBox,
)

import config

def _make_plot(title):
    p = pg.PlotWidget(background=config.COLOR_BG)
    p.setTitle(title, color=config.COLOR_TEXT)
    p.showGrid(x=True, y=True, alpha=0.3)
    p.setXRange(0, config.WINDOW, padding=0)
    p.addLegend(offset=(5, 5))
    return p


class HzRow(QWidget):
    valueChangedFloat = pyqtSignal(float)

    def __init__(self, name, init_value, min_value, max_value, step=1.0, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        label = QLabel(name)
        label.setFixedWidth(64)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(min_value, max_value)
        self.spin.setDecimals(1)
        self.spin.setSingleStep(step)
        self.spin.setValue(init_value)
        self.spin.setSuffix(" Hz")
        self.spin.valueChanged.connect(self.valueChangedFloat.emit)

        layout.addWidget(label)
        layout.addWidget(self.spin)

    def value(self):
        return self.spin.value()

    def set_value_silent(self, value):
        self.spin.blockSignals(True)
        self.spin.setValue(value)
        self.spin.blockSignals(False)


class GraphsWidget(QWidget):
    targetHzChanged = pyqtSignal(float)
    controlHzChanged = pyqtSignal(float)
    pantiltHzChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(6)

        self.plot_error = _make_plot("Açı Hatası (Error)")
        self.plot_error.setYRange(-config.TRACK_ANGLE_THRESHOLD_DEG * 6,
                                   config.TRACK_ANGLE_THRESHOLD_DEG * 6, padding=0)

        self.plot_vel = _make_plot("Motor Hızı (Velocity)")
        self.plot_vel.setYRange(-config.VEL_PLOT_RANGE, config.VEL_PLOT_RANGE, padding=0)

        layout.addWidget(self.plot_error)
        layout.addWidget(self.plot_vel)

        hz_box = QGroupBox("Döngü Hızları (Hz)")
        hz_box.setStyleSheet(f"""
        QGroupBox {{
            border: 1px solid {config.COLOR_GRID};
            border-radius: 6px;
            margin-top: 10px;
            font-weight: bold;
            color: {config.COLOR_TEXT};
            background-color: {config.COLOR_PANEL_BG};
        }}

        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
        }}

        QLabel {{
            color: {config.COLOR_TEXT};
            background-color: {config.COLOR_PANEL_BG};
        }}

        QDoubleSpinBox {{
            color: {config.COLOR_TEXT};
            background-color: #222831;
            border: 1px solid #3a4048;
            border-radius: 4px;
            padding: 3px;
        }}

        QDoubleSpinBox:hover {{
            background-color: #2c343d;
        }}

        QDoubleSpinBox::up-button,
        QDoubleSpinBox::down-button {{
            width: 14px;
            border: none;
            background: transparent;
        }}

        QDoubleSpinBox::up-button:hover,
        QDoubleSpinBox::down-button:hover {{
            background-color: #2c343d;
        }}
        """)
        hz_layout = QVBoxLayout(hz_box)
        hz_layout.setSpacing(4)

        self.hz_target = HzRow(
            "Target", config.TARGET_UPDATE_HZ, min_value=1.0, max_value=120.0, step=1.0,
        )
        self.hz_control = HzRow(
            "Control", config.CONTROL_LOOP_HZ, min_value=10.0, max_value=500.0, step=5.0,
        )
        self.hz_pantilt = HzRow(
            "Pan/Tilt", config.PANTILT_UPDATE_HZ, min_value=10.0, max_value=240.0, step=1.0,
        )

        self.hz_target.valueChangedFloat.connect(self.targetHzChanged.emit)
        self.hz_control.valueChangedFloat.connect(self.controlHzChanged.emit)
        self.hz_pantilt.valueChangedFloat.connect(self.pantiltHzChanged.emit)

        hz_layout.addWidget(self.hz_target)
        hz_layout.addWidget(self.hz_control)
        hz_layout.addWidget(self.hz_pantilt)

        self.real_hz_label = QLabel("real: -- / -- / -- Hz")
        self.real_hz_label.setStyleSheet(f"color: {config.COLOR_TEXT}; font-size: 10px;")
        hz_layout.addWidget(self.real_hz_label)

        layout.addWidget(hz_box)

        self._log_container = QVBoxLayout()
        self._log_container.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(self._log_container, 0)

        self.err_az_curve = self.plot_error.plot(pen=pg.mkPen(config.COLOR_CURRENT, width=2), name="Azimuth err")
        self.err_el_curve = self.plot_error.plot(pen=pg.mkPen(config.COLOR_TARGET, width=2), name="Elevation err")

        self.vel_az_curve = self.plot_vel.plot(pen=pg.mkPen(config.COLOR_ACCENT, width=2), name="Azimuth vel")
        self.vel_el_curve = self.plot_vel.plot(pen=pg.mkPen(config.COLOR_TARGET_SLOW, width=2), name="Elevation vel")

        n = config.WINDOW
        self.err_az = deque(maxlen=n)
        self.err_el = deque(maxlen=n)
        self.vel_az = deque(maxlen=n)
        self.vel_el = deque(maxlen=n)

    def add_log_widget(self, widget):
        self._log_container.addWidget(widget)

    def set_real_hz(self, target_hz: float, control_hz: float, pantilt_hz: float):
        self.real_hz_label.setText(
            f"real: {target_hz:.1f} / {control_hz:.1f} / {pantilt_hz:.1f} Hz"
        )

    def set_hz_values_silent(self, target_hz: float, control_hz: float, pantilt_hz: float):
        self.hz_target.set_value_silent(target_hz)
        self.hz_control.set_value_silent(control_hz)
        self.hz_pantilt.set_value_silent(pantilt_hz)

    def push(self, err_az, err_el, v_az, v_el):
        self.err_az.append(err_az)
        self.err_el.append(err_el)
        self.vel_az.append(v_az)
        self.vel_el.append(v_el)

        t = range(len(self.err_az))

        self.err_az_curve.setData(t, self.err_az)
        self.err_el_curve.setData(t, self.err_el)
        self.vel_az_curve.setData(t, self.vel_az)
        self.vel_el_curve.setData(t, self.vel_el)

    def clear(self):
        for d in (self.err_az, self.err_el, self.vel_az, self.vel_el):
            d.clear()
        for c in (self.err_az_curve, self.err_el_curve, self.vel_az_curve, self.vel_el_curve):
            c.setData([], [])
