import html
from datetime import datetime
import config


DEFAULT_LEVEL_COLORS = {
    "INFO": config.COLOR_LOG_INFO,
    "WARNING": config.COLOR_LOG_WARNING,
    "ERROR": config.COLOR_LOG_ERROR,
}


class Logger:
    LEVELS = ("INFO", "WARNING", "ERROR")

    def __init__(self, ui_widget, max_lines=200, level_colors=None,
                 default_text_color=config.COLOR_LOG_INFO):
        self.ui_widget = ui_widget
        self.max_lines = max_lines

        self.level_colors = dict(DEFAULT_LEVEL_COLORS)
        if level_colors:
            self.level_colors.update(level_colors)

        self.default_text_color = default_text_color
        self._entries = []

    @staticmethod
    def _format_value(v):
        if isinstance(v, float):
            return f"{v:.3f}".rstrip("0").rstrip(".")
        return str(v)

    def _format_line(self, level, event, kwargs):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        parts = [f"[{ts}]", f"[{level}]", event]
        if kwargs:
            parts.append(" ".join(
                f"{k}={self._format_value(v)}" for k, v in kwargs.items()
            ))
        return " ".join(parts)

    # ------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------
    def log(self, level: str, event: str, **kwargs) -> str:
        level = level.upper()
        if level not in self.LEVELS:
            level = "INFO"

        line = self._format_line(level, event, kwargs)
        self._entries.append((line, level))

        if len(self._entries) > self.max_lines:
            self._entries.pop(0)

        self._render()
        return line

    def clear(self):
        self._entries.clear()
        self._render()

    # ------------------------------------------------------------
    # Render
    # ------------------------------------------------------------
    def _render(self):
        widget = self.ui_widget
        scrollbar = widget.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 2

        last_idx = len(self._entries) - 1
        html_lines = []

        for i, (text, level) in enumerate(self._entries):
            color = self.level_colors.get(level, self.default_text_color)
            escaped = html.escape(text)

            if i == last_idx:
                html_lines.append(
                    f'<div style="color:{color}; font-weight:bold; '
                    f'background-color:rgba(255,255,255,0.08); '
                    f'padding:2px 4px; border-radius:4px;">{escaped}</div>'
                )
            else:
                html_lines.append(
                    f'<div style="color:{color}; opacity:0.85;">{escaped}</div>'
                )

        widget.setHtml("".join(html_lines))

        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())