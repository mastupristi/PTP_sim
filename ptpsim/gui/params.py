# SPDX-License-Identifier: Apache-2.0
"""Parameter widgets: a spin box + slider pair bound to one or more configuration paths."""
from __future__ import annotations

import math

from PySide6 import QtCore, QtWidgets


class ParamRow(QtCore.QObject):
    """Numeric control (spin box + slider).  ``scale`` converts display units to config units.

    ``paths``: configuration keys written with the converted value (dotted).
    ``log``: slider mapped on a logarithmic scale (spin box stays linear).
    """
    changed = QtCore.Signal(object, float)    # (row, config-unit value)

    def __init__(self, label: str, paths: list[str], lo: float, hi: float, value: float, step: float,
                 decimals: int = 3, suffix: str = "", scale: float = 1.0, slider: bool = True,
                 log: bool = False, integer: bool = False, parent=None, tooltip: str = ""):
        super().__init__(parent)
        self.paths = paths
        self.scale = scale
        self.log = log and lo > 0
        self.integer = integer
        self.label = QtWidgets.QLabel(label)
        if integer:
            self.spin = QtWidgets.QSpinBox()
            self.spin.setRange(int(lo), int(hi))
            self.spin.setSingleStep(int(step))
        else:
            self.spin = QtWidgets.QDoubleSpinBox()
            self.spin.setDecimals(decimals)
            self.spin.setRange(lo, hi)
            self.spin.setSingleStep(step)
        self.spin.setKeyboardTracking(False)
        if suffix:
            self.spin.setSuffix(" " + suffix)
        self.spin.setValue(value)
        self.spin.setMinimumWidth(110)
        self.slider = None
        if slider:
            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, 1000)
            self.slider.setMinimumWidth(90)
            self._sync_slider()
            self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)
        for w in (self.label, self.spin, self.slider):
            if w is not None and tooltip:
                w.setToolTip(tooltip)

    # --- mapping slider <-> spin
    def _pos_from_value(self, v: float) -> int:
        lo, hi = self.spin.minimum(), self.spin.maximum()
        if hi <= lo:
            return 0
        if self.log:
            f = (math.log(max(v, lo)) - math.log(lo)) / (math.log(hi) - math.log(lo))
        else:
            f = (v - lo) / (hi - lo)
        return int(round(max(0.0, min(1.0, f)) * 1000))

    def _value_from_pos(self, p: int) -> float:
        lo, hi = self.spin.minimum(), self.spin.maximum()
        f = p / 1000.0
        if self.log:
            return math.exp(math.log(lo) + f * (math.log(hi) - math.log(lo)))
        return lo + f * (hi - lo)

    def _sync_slider(self):
        if self.slider is None:
            return
        self.slider.blockSignals(True)
        self.slider.setValue(self._pos_from_value(self.spin.value()))
        self.slider.blockSignals(False)

    def _on_spin(self, v):
        self._sync_slider()
        self.changed.emit(self, self.config_value())

    def _on_slider(self, p):
        v = self._value_from_pos(p)
        if self.integer:
            v = round(v)
        else:
            d = self.spin.decimals()
            v = round(v, d)
        self.spin.setValue(v)       # triggers _on_spin -> changed

    # --- API
    def config_value(self) -> float:
        return float(self.spin.value()) * self.scale

    def set_config_value(self, v: float, emit: bool = False):
        self.spin.blockSignals(True)
        self.spin.setValue(v / self.scale if not self.integer else int(round(v / self.scale)))
        self.spin.blockSignals(False)
        self._sync_slider()
        if emit:
            self.changed.emit(self, self.config_value())

    def add_to(self, form: QtWidgets.QGridLayout, row: int):
        form.addWidget(self.label, row, 0)
        form.addWidget(self.spin, row, 1)
        if self.slider is not None:
            form.addWidget(self.slider, row, 2)

    def set_enabled(self, on: bool):
        for w in (self.label, self.spin, self.slider):
            if w is not None:
                w.setEnabled(on)
