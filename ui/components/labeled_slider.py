from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QSlider


class LabeledSlider(QWidget):
    valueChangedFloat = pyqtSignal(float)

    def __init__(self, name, init_value, max_value=1.0, parent=None):
        super().__init__(parent)
        self._max_value = max_value
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.label = QLabel(f"{name}")
        self.label.setFixedWidth(48)
        self.value_label = QLabel(f"{init_value:.3f}")
        self.value_label.setFixedWidth(48)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setValue(int(init_value / max_value * 1000))
        self.slider.valueChanged.connect(self._on_change)

        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value_label)

    def _on_change(self, raw):
        val = raw / 1000.0 * self._max_value
        self.value_label.setText(f"{val:.3f}")
        self.valueChangedFloat.emit(val)

    def value(self):
        return self.slider.value() / 1000.0 * self._max_value
