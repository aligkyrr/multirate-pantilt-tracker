"""
ui/control_panel/hardware_panel.py
====================================

`pantilt_hardware.HARDWARE_MENU_SCHEMA` üzerinden otomatik olarak
oluşturulan, donanım gerçekçilik parametrelerini (min/max hız, açı
limiti, ivme, haberleşme hızı, hız dalgalanması, çözünürlük, accuracy,
repeatability, settling time) çalışma zamanında ayarlayabileceğin
PyQt5 paneli.

`ControlPanel` (panel.py) ile aynı statüde, bağımsız bir QWidget --
mevcut `ControlPanel`'in iç yapısına (sections/interactions/api
mixin'lerine) dokunmaz. `layout.py` bu iki paneli bir QTabWidget
içinde birleştirir.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QGroupBox, QDoubleSpinBox,
    QCheckBox, QPushButton, QLabel, QScrollArea, QHBoxLayout, QFrame
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

import config
from core.pantilt_hardware import HARDWARE_MENU_SCHEMA


# Şema satırlarını mantıksal gruplara ayırmak için basit bir eşleme
# (config anahtarı -> grup başlığı). Şemanın kendisi tek bir düz liste
# olduğu için, panelde daha okunur bir "menü" görünümü sağlamak adına
# burada gruplandırılıyor.
_GROUPS = [
    ("Azimuth", lambda key: key.startswith("AZ_")),
    ("Elevation", lambda key: key.startswith("EL_")),
    ("Haberleşme", lambda key: key.startswith("COMM_")),
    ("Yerleşme (Settling)", lambda key: key == "SETTLING_BAND_DEG"),
]


class HardwareProfilePanel(QWidget):
    """
    Donanım gerçekçilik parametrelerini gösteren/düzenleyen panel.

    Sinyaller:
        profile_changed: bir parametre değiştirilip "Uygula"ya
            basıldığında yayılır.
        realism_toggled(bool): üstteki ana açma/kapama kutusu
            değiştiğinde yayılır.
    """

    profile_changed = pyqtSignal()
    realism_toggled = pyqtSignal(bool)

    def __init__(self, tracker=None, parent=None):
        """
        tracker: opsiyonel `simulator.PanTiltTracker` referansı. Verilirse
        panel, "Uygula" ve "Aç/Kapat" işlemlerini doğrudan tracker
        üzerinde tetikler (update_hardware_profile /
        set_hardware_realism_enabled). Verilmezse sadece config.py
        değerlerini günceller ve sinyalleri yayınlar.
        """
        super().__init__(parent)
        self._tracker = tracker
        self._inputs = {}  # config_key -> (widget, kind)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        # ---- Üst: ana aç/kapat + telemetri ----
        header = QHBoxLayout()
        self.enable_checkbox = QCheckBox("Donanım Gerçekçiliğini Etkinleştir")
        self.enable_checkbox.setChecked(config.HARDWARE_REALISM_ENABLED_DEFAULT)
        self.enable_checkbox.toggled.connect(self._on_realism_toggled)
        header.addWidget(self.enable_checkbox)
        header.addStretch(1)
        outer.addLayout(header)

        self.telemetry_label = QLabel("—")
        self.telemetry_label.setStyleSheet(f"color: {config.COLOR_LOG_INFO};")
        outer.addWidget(self.telemetry_label)

        # ---- Scroll edilebilir parametre alanı ----
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(10)

        grouped_keys = set()
        for title, predicate in _GROUPS:
            group_box = self._build_group(title, predicate)
            if group_box is not None:
                scroll_layout.addWidget(group_box)
                for label, key, *_ in HARDWARE_MENU_SCHEMA:
                    if predicate(key):
                        grouped_keys.add(key)

        remaining = [row for row in HARDWARE_MENU_SCHEMA if row[1] not in grouped_keys]
        if remaining:
            other_box = self._build_group_from_rows("Diğer", remaining)
            scroll_layout.addWidget(other_box)

        scroll_layout.addStretch(1)
        scroll.setWidget(scroll_content)
        outer.addWidget(scroll, 1)

        btn_row = QHBoxLayout()
        self.apply_btn = QPushButton("Uygula")
        self.apply_btn.clicked.connect(self._on_apply)
        self.reset_btn = QPushButton("Varsayılana Dön")
        self.reset_btn.clicked.connect(self._on_reset_defaults)
        btn_row.addWidget(self.apply_btn)
        btn_row.addWidget(self.reset_btn)
        outer.addLayout(btn_row)

        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.timeout.connect(self._refresh_telemetry)
        self._telemetry_timer.start(250)

        self._apply_stylesheet()

    def _apply_stylesheet(self):
        self.setStyleSheet(f"""
            QWidget {{
                background: {config.COLOR_PANEL_BG};
                color: {config.COLOR_TEXT};
            }}
            QGroupBox {{
                border: 1px solid {config.COLOR_GRID};
                border-radius: 8px;
                margin-top: 10px;
                font-weight: 600;
                padding-top: 6px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: {config.COLOR_ACCENT};
            }}
            QPushButton {{
                background: {config.COLOR_ACCENT};
                color: #0d0f12;
                border: none;
                border-radius: 6px;
                padding: 6px 14px;
                font-weight: 600;
            }}
            QPushButton:hover {{
                background: #34e07d;
            }}
            QDoubleSpinBox, QCheckBox {{
                background: {config.COLOR_BG};
                border: 1px solid {config.COLOR_GRID};
                border-radius: 4px;
                padding: 2px 4px;
            }}
            QScrollArea {{
                border: none;
                background: transparent;
            }}
            QScrollBar:vertical {{
                background: {config.COLOR_PANEL_BG};
                width: 10px;
                margin: 0px;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical {{
                background: {config.COLOR_GRID};
                min-height: 24px;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: {config.COLOR_ACCENT};
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
                border: none;
                background: none;
            }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
                background: none;
            }}
            QScrollBar:horizontal {{
                background: {config.COLOR_PANEL_BG};
                height: 10px;
                margin: 0px;
                border-radius: 5px;
            }}
            QScrollBar::handle:horizontal {{
                background: {config.COLOR_GRID};
                min-width: 24px;
                border-radius: 5px;
            }}
            QScrollBar::handle:horizontal:hover {{
                background: {config.COLOR_ACCENT};
            }}
            QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
                width: 0px;
                border: none;
                background: none;
            }}
            QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
                background: none;
            }}
        """)

    # ------------------------------------------------------------
    def _build_group(self, title, predicate):
        rows = [row for row in HARDWARE_MENU_SCHEMA if predicate(row[1])]
        if not rows:
            return None
        return self._build_group_from_rows(title, rows)

    def _build_group_from_rows(self, title, rows):
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignLeft)

        for label, key, vmin, vmax, step, unit in rows:
            if unit == "bool":
                widget = QCheckBox()
                widget.setChecked(bool(getattr(config, key)))
                self._inputs[key] = (widget, "bool")
                form.addRow(label, widget)
            else:
                widget = QDoubleSpinBox()
                widget.setMinimum(vmin)
                widget.setMaximum(vmax)
                widget.setSingleStep(step)
                widget.setDecimals(self._decimals_for_step(step))
                widget.setSuffix(f" {unit}" if unit else "")
                widget.setValue(float(getattr(config, key)))
                self._inputs[key] = (widget, "float")
                form.addRow(label, widget)

        return box

    @staticmethod
    def _decimals_for_step(step: float) -> int:
        if step >= 1.0:
            return 0
        s = f"{step:.10f}".rstrip("0")
        return max(0, len(s.split(".")[-1]))

    # ------------------------------------------------------------
    def _on_realism_toggled(self, checked: bool):
        config.HARDWARE_REALISM_ENABLED_DEFAULT = checked
        if self._tracker is not None:
            self._tracker.set_hardware_realism_enabled(checked)
        self.realism_toggled.emit(checked)

    def _on_apply(self):
        for key, (widget, kind) in self._inputs.items():
            value = widget.isChecked() if kind == "bool" else widget.value()
            setattr(config, key, value)

        if self._tracker is not None:
            self._tracker.update_hardware_profile()
        self.profile_changed.emit()

    def _on_reset_defaults(self):
        for key, (widget, kind) in self._inputs.items():
            default_value = getattr(config, key)
            if kind == "bool":
                widget.setChecked(bool(default_value))
            else:
                widget.setValue(float(default_value))

    # ------------------------------------------------------------
    def _refresh_telemetry(self):
        if self._tracker is None or not hasattr(self._tracker, "device"):
            return
        dev = self._tracker.device
        settled_txt = "YERLEŞTİ" if dev.is_settled else "hareket halinde"
        self.telemetry_label.setText(
            f"Cihaz durumu: {settled_txt}\n"
            f"Komut kaçırma oranı: %{dev.drop_rate * 100:.1f}"
            f"({dev.commands_accepted}/{dev.commands_sent} kabul)"
        )