# SPDX-License-Identifier: Apache-2.0
"""ViewBox whose *zoom* (wheel, right-button drag) can be restricted per axis, independently of the pan.

pyqtgraph's ``setMouseEnabled`` switches zoom and pan together; here ``zoom_enabled`` only affects the zoom, so
with one axis unchecked it is still possible to drag the plot along that axis.
"""
from __future__ import annotations

import pyqtgraph as pg
from PySide6 import QtCore


class ZoomViewBox(pg.ViewBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.zoom_enabled = [True, True]          # [x, y]

    def _zooming(self, fn, *args, **kwargs):
        """Run a pyqtgraph zoom handler with the mouse mask limited to the axes allowed to zoom."""
        saved = list(self.state["mouseEnabled"])
        self.state["mouseEnabled"] = [bool(m and z) for m, z in zip(saved, self.zoom_enabled)]
        try:
            return fn(*args, **kwargs)
        finally:
            self.state["mouseEnabled"] = saved

    def wheelEvent(self, ev, axis=None):
        return self._zooming(super().wheelEvent, ev, axis=axis)

    def mouseDragEvent(self, ev, axis=None):
        if ev.button() & QtCore.Qt.MouseButton.RightButton:     # right-button drag = zoom, left = pan
            return self._zooming(super().mouseDragEvent, ev, axis=axis)
        return super().mouseDragEvent(ev, axis=axis)
