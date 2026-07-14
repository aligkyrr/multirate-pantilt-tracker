from PyQt5.QtGui import QColor, QFont

import config

_JET_SHAPE = [
    (0.00, -1.00),   # burun

    (0.06, -0.55),   # gövde sağ
    (0.10, -0.20),

    (0.55,  0.00),   # sağ kanat ön
    (0.95,  0.25),   # sağ kanat uç
    (0.45,  0.15),   # sağ kanat arka

    (0.18,  0.35),   # gövde genişliyor
    (0.10,  0.70),

    (0.22,  0.95),   # sağ kuyruk kanadı
    (0.00,  0.85),   # kuyruk orta
    (-0.22, 0.95),   # sol kuyruk kanadı

    (-0.10, 0.70),
    (-0.18, 0.35),

    (-0.45, 0.15),   # sol kanat arka
    (-0.95, 0.25),   # sol kanat uç
    (-0.55, 0.00),   # sol kanat ön

    (-0.10, -0.20),
    (-0.06, -0.55),
]

_COLOR_LOCK = getattr(config, "COLOR_LOCK", "#a33f3f")          # mat/koyu kırmızı
_COLOR_AIM_LINE = getattr(config, "COLOR_AIM_LINE", "#5c6a63")   # mat, düşük doygunluk
_COLOR_GRID = getattr(config, "COLOR_GRID", "#2a2f36")
_COLOR_ACCENT = getattr(config, "COLOR_ACCENT", "#8a97a3")       # aktif hedef / crosshair için nötr accent
_COLOR_TEXT = getattr(config, "COLOR_TEXT", "#c7ccd1")
_COLOR_BG = getattr(config, "COLOR_BG", "#14171b")

_COLOR_AXIS_TEXT = getattr(config, "COLOR_AXIS_TEXT", QColor(140, 150, 160, 120))

_COLOR_STATUS_LOCK_TEXT = getattr(config, "COLOR_STATUS_LOCKED", QColor(_COLOR_LOCK).lighter(130).name())


def _mono_font(point_size: int, bold: bool = False) -> QFont:
    font = QFont("Consolas")
    font.setStyleHint(QFont.Monospace)
    font.setPointSize(point_size)
    font.setBold(bold)
    return font
