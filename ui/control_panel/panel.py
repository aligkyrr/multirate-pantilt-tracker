from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget

from .sections import _ControlPanelSectionsMixin
from .interactions import _ControlPanelInteractionsMixin
from .api import _ControlPanelApiMixin


class ControlPanel(QWidget, _ControlPanelSectionsMixin,
                    _ControlPanelInteractionsMixin, _ControlPanelApiMixin):
    pauseClicked = pyqtSignal()
    resetClicked = pyqtSignal()

    addTargetRequested = pyqtSignal(str)      # target type
    removeTargetRequested = pyqtSignal(int)   # target id
    targetSelected = pyqtSignal(int)          # target id (listeden seçim)
    autoModeToggled = pyqtSignal(bool)        # True = AUTO
    autoStrategyChanged = pyqtSignal(str)
    leadToggled = pyqtSignal(bool)

    trackingModeToggled = pyqtSignal(bool)    # True = ROTA modu
    routeEditToggled = pyqtSignal(bool)       # True = radar tıklaması waypoint ekler
    routeLoopModeChanged = pyqtSignal(str)
    routeSpeedChanged = pyqtSignal(float)
    routeClearRequested = pyqtSignal()
    routeUndoRequested = pyqtSignal()

    accelLimitToggled = pyqtSignal(bool)
    accelAzChanged = pyqtSignal(float)
    accelElChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_stylesheet()

        layout = self._build_scroll_root()

        self._build_targets_section(layout)
        self._build_mode_section(layout)
        self._build_route_section(layout)
        self._build_gains_and_accel_sections(layout)
        self._build_status_and_log_sections(layout)
