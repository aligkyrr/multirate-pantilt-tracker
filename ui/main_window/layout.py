from PyQt5.QtWidgets import QWidget, QHBoxLayout, QTabWidget
import config

class _LayoutMixin:
    def _build_layout(self):
        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        center_tabs = QTabWidget()
        center_tabs.addTab(self.radar, "RADAR")
        center_tabs.addTab(self.gl_view, "GIMBAL 3D")

        center_tabs.setDocumentMode(True)
        center_tabs.setTabPosition(QTabWidget.North)

        center_tabs.setStyleSheet(f"""
        QTabWidget {{
            background: {config.COLOR_BG};
            border: none;
        }}
                                  
        QTabWidget::pane {{
            border: 1px solid {config.COLOR_GRID};
            border-radius: 12px;
            background-color: #fff;
            margin-top: 10px;
        }}

        QTabBar::tab {{
            background: {config.COLOR_PANEL_BG};
            color: #9aa4ad;
            padding: 10px 26px;
            margin-right: 8px;
            border-top-left-radius: 10px;
            min-width: 130px;
            border-top-right-radius: 10px;
            font-weight: 600;
        }}

        QTabBar::tab:selected {{
            background: {config.COLOR_BG};
            color: {config.COLOR_TEXT};
            border-bottom: 3px solid {config.COLOR_ACCENT};
        }}

        QTabBar::tab:hover {{
            background: #1c2128;
            color: {config.COLOR_TEXT};
        }}

        QTabBar::tab:!selected {{
            margin-top: 6px;
        }}

        QTabBar::tab:selected {{
            margin-top: 0px;
        }}
        """)

        center_tabs.tabBar().setFixedHeight(40)
        center_tabs.tabBar().setDrawBase(False)
        
        root.addWidget(self.control_panel, 22)
        root.addWidget(center_tabs, 48)
        root.addWidget(self.graphs, 30)

        self.graphs.add_log_widget(self.control_panel.log_view)

        self.setCentralWidget(central)

    def _connect_signals(self):
        self.control_panel.pauseClicked.connect(self.on_pause)
        self.control_panel.resetClicked.connect(self.on_reset)
        self.control_panel.addTargetRequested.connect(self.on_add_target)
        self.control_panel.removeTargetRequested.connect(self.on_remove_target)
        self.control_panel.targetSelected.connect(self.on_manual_select)
        self.control_panel.autoModeToggled.connect(self.on_auto_toggle)
        self.control_panel.autoStrategyChanged.connect(self.on_strategy_change)
        self.control_panel.leadToggled.connect(self.on_lead_toggle)

        self.control_panel.trackingModeToggled.connect(self.on_tracking_mode_toggle)
        self.control_panel.routeEditToggled.connect(self.radar.set_route_edit_mode)
        self.control_panel.routeLoopModeChanged.connect(self.on_route_loop_mode_change)
        self.control_panel.routeSpeedChanged.connect(self.on_route_speed_change)
        self.control_panel.routeClearRequested.connect(self.on_route_clear)
        self.control_panel.routeUndoRequested.connect(self.on_route_undo)

        self.control_panel.accelLimitToggled.connect(self.on_accel_limit_toggle)
        self.control_panel.accelAzChanged.connect(self.on_accel_az_change)
        self.control_panel.accelElChanged.connect(self.on_accel_el_change)

        self.radar.targetClicked.connect(self.on_manual_select)
        self.radar.routePointAdded.connect(self.on_route_point_added)

        self.graphs.targetHzChanged.connect(self.on_target_hz_change)
        self.graphs.controlHzChanged.connect(self.on_control_hz_change)
        self.graphs.pantiltHzChanged.connect(self.on_pantilt_hz_change)
