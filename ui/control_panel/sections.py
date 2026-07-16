from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QGroupBox,
    QListWidget, QComboBox, QCheckBox, QScrollArea, QWidget,
)

import config
from core.logger import Logger
from ui.components.labeled_slider import LabeledSlider


class _ControlPanelSectionsMixin:
    def _build_stylesheet(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {config.COLOR_PANEL_BG}; color: {config.COLOR_TEXT}; }}
            QPushButton {{
                background-color: #222831; border: 1px solid #3a4048;
                border-radius: 4px; padding: 6px;
            }}
            QPushButton:hover {{ background-color: #2c343d; }}
            QComboBox {{
                background-color: #222831; border: 1px solid #3a4048;
                border-radius: 4px; padding: 6px; color: {config.COLOR_TEXT};
            }}
            QComboBox:hover {{ background-color: #2c343d; }}
            QComboBox::drop-down {{
                border: none; width: 20px;
            }}
            QComboBox QAbstractItemView {{
                background-color: #222831; border: 1px solid #3a4048;
                color: {config.COLOR_TEXT};
                selection-background-color: #2c343d;
                outline: none;
            }}
            QGroupBox {{ border: 1px solid #2a2f36; margin-top: 8px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; }}
            QListWidget {{ background-color: #0d0f12; border: 1px solid #2a2f36; }}
        """)

    def _build_scroll_root(self):
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        outer_layout.addWidget(scroll)

        content = QWidget()
        scroll.setWidget(content)

        layout = QVBoxLayout(content)

        title = QLabel("KONTROL PANELİ")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(title)

        self.fps_label = QLabel("FPS: --")
        self.fps_label.setStyleSheet("color: #28c76f; font-weight: bold;")
        layout.addWidget(self.fps_label)
        return layout

    def _build_targets_section(self, layout):
        targets_box = QGroupBox("Hedefler")
        targets_layout = QVBoxLayout(targets_box)

        add_row = QHBoxLayout()
        self.type_combo = QComboBox()
        self.type_combo.addItems(list(config.TARGET_TYPES))
        self.btn_add = QPushButton("➕ Ekle")
        self.btn_remove = QPushButton("➖ Sil")
        self.btn_add.clicked.connect(self._on_add)
        self.btn_remove.clicked.connect(self._on_remove)
        add_row.addWidget(self.type_combo)
        add_row.addWidget(self.btn_add)
        add_row.addWidget(self.btn_remove)
        targets_layout.addLayout(add_row)

        self.target_list = QListWidget()
        self.target_list.setFixedHeight(120)
        self.target_list.itemClicked.connect(self._on_list_click)
        targets_layout.addWidget(self.target_list)

        layout.addWidget(targets_box)

    def _build_mode_section(self, layout):
        mode_box = QGroupBox("Hedef Seçim Modu")
        mode_layout = QVBoxLayout(mode_box)

        auto_row = QHBoxLayout()
        auto_row.addWidget(QLabel("Mod:"))
        self.auto_mode_label = QLabel("AUTO")
        self.auto_mode_label.setStyleSheet("font-weight: bold; color: #28c76f;")
        auto_row.addWidget(self.auto_mode_label)
        auto_row.addStretch(1)
        self.btn_auto_manual = QPushButton("Değiştir")
        self.btn_auto_manual.setCheckable(True)
        self.btn_auto_manual.setChecked(True)
        self.btn_auto_manual.clicked.connect(self._on_auto_toggle)
        auto_row.addWidget(self.btn_auto_manual)
        mode_layout.addLayout(auto_row)

        self.strategy_combo = QComboBox()
        self.strategy_combo.addItem("En yakın (mesafe)", config.AUTO_STRATEGY_NEAREST)
        self.strategy_combo.addItem("Ortalamaya en yakın", config.AUTO_STRATEGY_CENTER)
        self.strategy_combo.currentIndexChanged.connect(self._on_strategy_change)
        mode_layout.addWidget(self.strategy_combo)

        self.chk_lead = QCheckBox("Hedef tahmini (lead/prediction)")
        self.chk_lead.stateChanged.connect(lambda s: self.leadToggled.emit(s == Qt.Checked))
        mode_layout.addWidget(self.chk_lead)

        layout.addWidget(mode_box)

    def _build_route_section(self, layout):
        route_box = QGroupBox("Rota (Waypoint) Modu")
        route_layout = QVBoxLayout(route_box)

        tracking_row = QHBoxLayout()
        tracking_row.addWidget(QLabel("Takip:"))
        self.tracking_mode_label = QLabel("HEDEF TAKİP")
        self.tracking_mode_label.setStyleSheet("font-weight: bold; color: #ff9f43;")
        tracking_row.addWidget(self.tracking_mode_label)
        tracking_row.addStretch(1)
        self.btn_tracking_mode = QPushButton("Değiştir")
        self.btn_tracking_mode.setCheckable(True)
        self.btn_tracking_mode.setChecked(False)
        self.btn_tracking_mode.clicked.connect(self._on_tracking_mode_toggle)
        tracking_row.addWidget(self.btn_tracking_mode)
        route_layout.addLayout(tracking_row)

        self.btn_route_edit = QPushButton("🖊 Rota Düzenle: KAPALI")
        self.btn_route_edit.setCheckable(True)
        self.btn_route_edit.clicked.connect(self._on_route_edit_toggle)
        route_layout.addWidget(self.btn_route_edit)

        route_hint = QLabel("Düzenleme AÇIKKEN radara tıklayarak\nsırayla waypoint ekleyin.")
        route_hint.setStyleSheet(f"color: {config.COLOR_ROUTE_EDIT_HINT}; font-size: 10px;")
        route_layout.addWidget(route_hint)

        loop_row = QHBoxLayout()
        loop_row.addWidget(QLabel("Rotanın sonuna gelince:"))
        self.loop_combo = QComboBox()
        self.loop_combo.addItem("Döngü (loop)", config.ROUTE_LOOP_MODE_LOOP)
        self.loop_combo.addItem("Son noktada dur", config.ROUTE_LOOP_MODE_STOP)
        self.loop_combo.addItem("Gidip-gel (ping-pong)", config.ROUTE_LOOP_MODE_PINGPONG)
        self.loop_combo.currentIndexChanged.connect(self._on_loop_mode_change)
        loop_row.addWidget(self.loop_combo)
        route_layout.addLayout(loop_row)

        self.s_route_speed = LabeledSlider(
            "Hız", config.ROUTE_DEFAULT_SPEED_MPS, max_value=config.ROUTE_SPEED_MAX_MPS
        )
        self.s_route_speed.valueChangedFloat.connect(self.routeSpeedChanged.emit)
        route_layout.addWidget(self.s_route_speed)

        route_btn_row = QHBoxLayout()
        self.btn_route_undo = QPushButton("↩ Son Noktayı Sil")
        self.btn_route_clear = QPushButton("🗑 Temizle")
        self.btn_route_undo.clicked.connect(self.routeUndoRequested.emit)
        self.btn_route_clear.clicked.connect(self.routeClearRequested.emit)
        route_btn_row.addWidget(self.btn_route_undo)
        route_btn_row.addWidget(self.btn_route_clear)
        route_layout.addLayout(route_btn_row)

        self.route_count_label = QLabel("Waypoint: 0")
        route_layout.addWidget(self.route_count_label)

        layout.addWidget(route_box)

    def _build_gains_and_accel_sections(self, layout):
        gains_box = QGroupBox("Smart Controller (PID)")
        gains_layout = QVBoxLayout(gains_box)

        self.s_kp = LabeledSlider("KP", config.PID_KP, max_value=1.5)
        self.s_ki = LabeledSlider("KI", config.PID_KI, max_value=0.2)
        self.s_kd = LabeledSlider("KD", config.PID_KD, max_value=1.5)

        for s in (self.s_kp, self.s_ki, self.s_kd):
            gains_layout.addWidget(s)

        layout.addWidget(gains_box)

        accel_box = QGroupBox("Motor İvme Limiti (derece/s²)")
        accel_layout = QVBoxLayout(accel_box)

        self.chk_accel_limit = QCheckBox("İvme limiti aktif")
        self.chk_accel_limit.setChecked(config.ACCEL_LIMIT_ENABLED_DEFAULT)
        self.chk_accel_limit.stateChanged.connect(
            lambda s: self.accelLimitToggled.emit(s == Qt.Checked)
        )
        accel_layout.addWidget(self.chk_accel_limit)

        self.s_accel_az = LabeledSlider(
            "Pan", config.MAX_ACCEL_AZ_DEFAULT_DEG_S2, max_value=config.ACCEL_LIMIT_MAX_DEG_S2
        )
        self.s_accel_el = LabeledSlider(
            "Tilt", config.MAX_ACCEL_EL_DEFAULT_DEG_S2, max_value=config.ACCEL_LIMIT_MAX_DEG_S2
        )
        self.s_accel_az.valueChangedFloat.connect(self.accelAzChanged.emit)
        self.s_accel_el.valueChangedFloat.connect(self.accelElChanged.emit)
        accel_layout.addWidget(self.s_accel_az)
        accel_layout.addWidget(self.s_accel_el)

        layout.addWidget(accel_box)

    def _build_status_and_log_sections(self, layout):
        # ---------------- Durum göstergesi ----------------
        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Mod:"))
        self.mode_label = QLabel("COARSE")
        self.mode_label.setStyleSheet("font-weight: bold; color: #ff9f43;")
        status_row.addWidget(self.mode_label)

        self.lock_label = QLabel("● NO LOCK")
        self.lock_label.setStyleSheet("font-weight: bold; color: #7a828c;")
        status_row.addWidget(self.lock_label)
        status_row.addStretch(1)
        layout.addLayout(status_row)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(120)
        self.log_view.setStyleSheet(f"""
            QTextEdit {{
                background-color: #05070a;
                color: {getattr(config, "COLOR_LOG_INFO", "#8a97a8")};
                font-family: Consolas, monospace;
                font-size: 13px;
                border: 1px solid #3a4048;
                border-radius: 6px;
                padding: 6px;
            }}
            QScrollBar:vertical {{
                background: #05070a;
                width: 8px;
                margin: 0px;
            }}
            QScrollBar::handle:vertical {{
                background: #3a4048;
                border-radius: 4px;
                min-height: 20px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: #4d545d;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
        """)
        self.log_view.setLineWrapMode(QTextEdit.WidgetWidth)

        self.logger = Logger(
            self.log_view,
            max_lines=config.MAX_LOG_LINES,
            level_colors={
                "INFO": getattr(config, "COLOR_LOG_INFO", "#8a97a8"),
                "WARNING": getattr(config, "COLOR_LOG_WARNING", "#c9922f"),
                "ERROR": getattr(config, "COLOR_LOG_ERROR", "#a6453f"),
            },
            default_text_color=getattr(config, "COLOR_LOG_INFO", "#8a97a8"),
        )
        self._active_id = None
        self.logger.log("INFO", "SYSTEM_READY")
