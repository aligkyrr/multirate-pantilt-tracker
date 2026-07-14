class _CoordinateMixin:
    def _scales(self):
        w = max(self.width(), 1)
        h = max(self.height(), 1)
        margin = 28
        span_x = max(self._xmax - self._xmin, 1e-6)
        span_y = max(self._ymax - self._ymin, 1e-6)
        sx = (w - 2 * margin) / span_x
        sy = (h - 2 * margin) / span_y
        return sx, sy, margin, w, h

    def _world_to_px(self, x, y):
        sx, sy, margin, w, h = self._scales()
        px = margin + (x - self._xmin) * sx
        py = h - margin - (y - self._ymin) * sy
        scale = (sx + sy) / 2.0
        return px, py, scale

    def _px_to_world(self, px, py):
        sx, sy, margin, w, h = self._scales()
        x = self._xmin + (px - margin) / sx
        y = self._ymin + (h - margin - py) / sy
        return x, y
