# SPDX-License-Identifier: Apache-2.0
"""Oscilloscope window: the GM and the slave 1 kHz square waves, triggered on the GM rising edge.

Everything is computed on demand from the exact slave-clock breakpoints (``ptpsim.scope``): the position of the
playback is just a sweep number, the maths does not depend on the playback speed, and nothing is stored per edge, so the
length is the simulated duration whatever it is.  Playback and drawing run in the GUI thread (a sweep costs a bisect).
"""
from __future__ import annotations

import time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from ..scope import (DEFAULT_PERIOD_NS, ScopeModel, edge_profile, persistence_samples, phosphor, pick_timebase_ns,
                     square_xy)
from .i18n import T

C_GM, C_SLAVE = "#F0E442", "#56B4E9"               # Okabe-Ito yellow / sky blue: legible on the dark screen
GM_LEVELS, SLAVE_LEVELS = (2.0, 3.0), (0.0, 1.0)
TIMEBASES_NS = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5, 2e5]     # per division
SPEEDS = [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0]                                          # simulated s / wall s
PERSIST_S = [0.0, 0.01, 0.1, 1.0, 10.0, 60.0, float('inf')]        # simulated time; inf = infinite persistence
DIVISIONS = 5                                        # per side of the trigger
N_SWEEPS = 2000                                      # most sweeps accumulated per frame (a longer window is thinned)
SCALE_WINDOW_S = 60.0                                # auto timebase: the sweeps of this much recent time must fit
RASTER_Y = (-0.25, 1.35)                             # screen range covered by the slave trace image (y units)
FRAME_MS = 33
ZOOM_IN_HOLD_S = 0.6                                 # auto timebase: wall time a smaller offset must last (playing)


def fmt_ns(v: float, signed: bool = True) -> str:
    a = abs(v)
    if a < 1e3:
        s = f"{v:+.2f} ns" if signed else f"{v:.2f} ns"
    elif a < 1e6:
        s = f"{v / 1e3:+.3f} µs" if signed else f"{v / 1e3:.3f} µs"
    elif a < 1e9:
        s = f"{v / 1e6:+.4f} ms" if signed else f"{v / 1e6:.4f} ms"
    else:
        s = f"{v / 1e9:+.4f} s" if signed else f"{v / 1e9:.4f} s"
    return s


def fmt_tb(ns: float) -> str:
    return f"{ns:g} ns/div" if ns < 1e3 else f"{ns / 1e3:g} µs/div"


class ScopeWindow(QtWidgets.QWidget):
    """Top-level window; closing it only hides it (the playback stops, the data stay)."""

    def __init__(self, period_ns: int = DEFAULT_PERIOD_NS):
        super().__init__(None, QtCore.Qt.Window)
        self.period_ns = int(period_ns)
        self.model: ScopeModel | None = None
        self.n_max = 0
        self.pos = 0.0                               # playback position in sweeps (float while playing)
        self.dir = 0                                 # 0 stopped, +1 forward, -1 reverse
        self._force = False
        self._last_wall = 0.0
        self._tb_auto = TIMEBASES_NS[-1]
        self._small_since: float | None = None
        self._t_end = 0.0
        self._acc: dict | None = None                # infinite persistence: accumulated profile, cleared like a scope does
        self.tb_ns = TIMEBASES_NS[-1]                # timebase of the last drawing
        self.setWindowTitle(T("scope_title"))
        self.resize(980, 600)
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(FRAME_MS)
        self.timer.timeout.connect(self._tick)
        self._set_enabled(False)

    # ------------------------------------------------------------------ UI
    def _build(self):
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        self.notice = QtWidgets.QLabel(T("scope_na"))
        self.notice.setAlignment(QtCore.Qt.AlignCenter)
        self.notice.setStyleSheet("color: gray")
        lay.addWidget(self.notice)

        self.pw = pg.PlotWidget(background="k")
        pi = self.pw.getPlotItem()
        self.pi = pi
        pi.setMenuEnabled(False)
        pi.hideButtons()
        pi.setMouseEnabled(False, False)
        pi.hideAxis("left")
        pi.setYRange(-0.25, 3.55, padding=0)
        ax = pi.getAxis("bottom")
        ax.setPen(pg.mkPen("w"))
        ax.setTextPen(pg.mkPen("w"))
        pi.setLabel("bottom", T("scope_ax"), units="s")
        self.grid_lines = []
        for m in range(-DIVISIONS, DIVISIONS + 1):
            ln = pg.InfiniteLine(angle=90, pen=pg.mkPen((90, 90, 90) if m else (200, 200, 200),
                                                        style=QtCore.Qt.DotLine if m else QtCore.Qt.DashLine))
            pi.addItem(ln, ignoreBounds=True)
            self.grid_lines.append(ln)
        self.gm_curve = pi.plot(pen=pg.mkPen(C_GM, width=2.5))
        self.img = pg.ImageItem()                        # persistence: intensity-graded accumulation of the slave sweeps
        self.img.setCompositionMode(QtGui.QPainter.CompositionMode_Plus)       # light adds up on the black screen
        self.img.setLookupTable(self._lut())
        self.img.setLevels((0.0, 1.0))
        self.img.setZValue(-10)
        self.img.setVisible(False)
        pi.addItem(self.img, ignoreBounds=True)
        self.slave_curve = pi.plot(pen=pg.mkPen(C_SLAVE, width=2.5))
        self.lbl_gm = pg.TextItem(T("scope_gm"), color=C_GM, anchor=(0, 1))
        self.lbl_slave = pg.TextItem(T("scope_slave"), color=C_SLAVE, anchor=(0, 1))
        self.lbl_trig = pg.TextItem("▼ " + T("scope_trig"), color=(255, 255, 255), anchor=(0.5, 1))
        for it in (self.lbl_gm, self.lbl_slave, self.lbl_trig):
            pi.addItem(it, ignoreBounds=True)
        lay.addWidget(self.pw, 1)

        self.info = QtWidgets.QLabel()
        self.info.setTextFormat(QtCore.Qt.RichText)
        self.info.setWordWrap(True)
        f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        self.info.setFont(f)
        lay.addWidget(self.info)

        # transport
        st = self.style()
        tr = QtWidgets.QHBoxLayout()
        self.btn_start = self._tbtn(st.standardIcon(QtWidgets.QStyle.SP_MediaSkipBackward), "scope_tip_start",
                                    self.go_start)
        self.btn_rev = self._tbtn(st.standardIcon(QtWidgets.QStyle.SP_MediaSeekBackward), "scope_tip_rev",
                                  lambda: self.set_direction(0 if self.dir < 0 else -1))
        self.btn_rev.setCheckable(True)                  # shows that the reverse playback is running
        self.btn_back = self._tbtn(None, "scope_tip_back", lambda: self.step(-1), text="−1")
        self.icon_play = st.standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        self.icon_pause = st.standardIcon(QtWidgets.QStyle.SP_MediaPause)
        self.btn_play = self._tbtn(self.icon_play, "scope_tip_play", lambda: self.set_direction(0 if self.dir > 0 else 1))
        self.btn_fwd = self._tbtn(None, "scope_tip_fwd", lambda: self.step(+1), text="+1")
        self.btn_end = self._tbtn(st.standardIcon(QtWidgets.QStyle.SP_MediaSkipForward), "scope_tip_end", self.go_end)
        for b in (self.btn_start, self.btn_rev, self.btn_back, self.btn_play, self.btn_fwd, self.btn_end):
            tr.addWidget(b)
        self.chk_loop = QtWidgets.QCheckBox(T("scope_loop"))
        tr.addWidget(self.chk_loop)
        tr.addStretch(1)
        tr.addWidget(QtWidgets.QLabel(T("scope_speed")))
        self.speed = QtWidgets.QComboBox()
        for s in SPEEDS:
            self.speed.addItem(f"{s:g}×", s)
        self.speed.setCurrentIndex(SPEEDS.index(10.0))
        self.speed.setToolTip(T("scope_speed_tip"))
        tr.addWidget(self.speed)
        tr.addWidget(QtWidgets.QLabel(T("scope_timebase")))
        self.tb = QtWidgets.QComboBox()
        self.tb.addItem(T("scope_tb_auto"), 0.0)
        for v in TIMEBASES_NS:
            self.tb.addItem(fmt_tb(v), float(v))
        self.tb.setToolTip(T("scope_tb_tip"))
        self.tb.currentIndexChanged.connect(lambda *_: self._draw())
        tr.addWidget(self.tb)
        tr.addWidget(QtWidgets.QLabel(T("scope_persist")))
        self.persist = QtWidgets.QComboBox()
        for v in PERSIST_S:
            self.persist.addItem(T("scope_persist_off") if v == 0 else "∞" if v == float("inf") else
                                 f"{v * 1e3:g} ms" if v < 1 else f"{v / 60:g} min" if v >= 60 else f"{v:g} s", v)
        self.persist.setToolTip(T("scope_persist_tip"))
        self.persist.currentIndexChanged.connect(lambda *_: (self._acc_clear(), self._draw()))
        tr.addWidget(self.persist)
        lay.addLayout(tr)

        sl = QtWidgets.QHBoxLayout()
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 0)
        self.slider.setPageStep(1000)
        self.slider.valueChanged.connect(self._on_slider)
        sl.addWidget(self.slider, 1)
        self.time_lbl = QtWidgets.QLabel("0.000 s")
        self.time_lbl.setMinimumWidth(170)
        self.time_lbl.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        sl.addWidget(self.time_lbl)
        lay.addLayout(sl)
        QtGui.QShortcut(QtGui.QKeySequence("Space"), self, activated=self.btn_play.click)

    @staticmethod
    def _lut() -> np.ndarray:
        """Intensity grading: black -> the channel colour (one fresh trace) -> white (hit again and again)."""
        v = np.linspace(0.0, 1.0, 256)[:, None]
        base = np.array(QtGui.QColor(C_SLAVE).getRgb()[:3], dtype=np.float64)[None, :]
        lo = base * np.minimum(1.0, v / 0.5)
        hi = base + (np.array([215.0, 240.0, 255.0]) - base) * np.clip((v - 0.5) / 0.5, 0.0, 1.0)
        return np.where(v <= 0.5, lo, hi).astype(np.uint8)

    def _tbtn(self, icon, tip_key, fn, text: str = "") -> QtWidgets.QToolButton:
        b = QtWidgets.QToolButton()
        if icon is not None:
            b.setIcon(icon)
        if text:
            b.setText(text)
        b.setToolTip(T(tip_key))
        b.setFocusPolicy(QtCore.Qt.NoFocus)
        b.clicked.connect(lambda *_: fn())
        return b

    def _set_enabled(self, on: bool):
        for w in (self.btn_start, self.btn_rev, self.btn_back, self.btn_play, self.btn_fwd, self.btn_end,
                  self.slider, self.speed, self.tb, self.persist, self.chk_loop):
            w.setEnabled(on)
        self.notice.setVisible(not on)
        self.pw.setVisible(on)
        self.info.setVisible(on)

    # ------------------------------------------------------------------ data
    def set_data(self, bp: dict | None, t_end_s: float = 0.0):
        """New slave-clock segments (an explore result), or ``None`` when there is no explore result to show
        (live mode, nothing computed yet).  The playback position is kept."""
        if bp is None:
            self.set_direction(0)
            self.model = None
            self.n_max = 0
            self._set_enabled(False)
            return
        self.model = ScopeModel.from_dict(bp, self.period_ns)
        self._acc_clear()
        self.n_max = int(t_end_s * 1e9 / self.period_ns)
        self.pos = min(self.pos, float(self.n_max))
        self.slider.blockSignals(True)
        self.slider.setRange(0, self.n_max)
        self.slider.blockSignals(False)
        self._t_end = t_end_s
        self._set_enabled(True)
        self._sync_slider()
        self._draw()

    # ------------------------------------------------------------------ transport
    @property
    def sweeps_per_s(self) -> float:
        return 1e9 / self.period_ns

    def set_direction(self, d: int):
        if self.model is None:
            d = 0
        if d != 0 and self.pos >= self.n_max and d > 0:
            self.pos = 0.0                             # play from the start when sitting on the end
        if d != 0 and self.pos <= 0 and d < 0:
            self.pos = float(self.n_max)
        self.dir = d
        self.btn_play.setIcon(self.icon_pause if d > 0 else self.icon_play)
        self.btn_rev.setChecked(d < 0)
        if d != 0:
            self._last_wall = time.perf_counter()
            self.timer.start()
        else:
            self.timer.stop()

    def go_start(self):
        self.set_direction(0)
        self.set_position(0)

    def go_end(self):
        self.set_direction(0)
        self.set_position(self.n_max)

    def step(self, n: int):
        self.set_direction(0)
        self.set_position(round(self.pos) + n)

    def set_position(self, k: float):
        self.pos = float(min(max(k, 0), self.n_max))
        self._sync_slider()
        self._draw()

    def _on_slider(self, v: int):
        self.pos = float(v)
        self._draw()

    def _sync_slider(self):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(self.pos)))
        self.slider.blockSignals(False)

    def _tick(self):
        if self.model is None or self.dir == 0:
            return
        now = time.perf_counter()
        dt, self._last_wall = now - self._last_wall, now
        self.pos += self.dir * self.speed.currentData() * dt * self.sweeps_per_s
        if self.pos >= self.n_max and self.dir > 0:
            if self.chk_loop.isChecked():
                self.pos = 0.0
            else:
                self.pos = float(self.n_max)
                self.set_direction(0)
        elif self.pos <= 0 and self.dir < 0:
            if self.chk_loop.isChecked():
                self.pos = float(self.n_max)
            else:
                self.pos = 0.0
                self.set_direction(0)
        self._sync_slider()
        self._draw()

    # ------------------------------------------------------------------ drawing
    def _timebase(self, max_edge_ns: float) -> float:
        """Fixed timebase, or auto: zoom out at once when an edge would leave the screen; zoom in at once when not
        playing (slider, steps) and, while playing, only after the offset stayed small for ``ZOOM_IN_HOLD_S`` -- the
        offset crosses zero during the transient and the scale must not jump there and back."""
        fixed = self.tb.currentData()
        if fixed:
            return float(fixed)
        need = float(pick_timebase_ns(max_edge_ns, TIMEBASES_NS))
        if need >= self._tb_auto or self.dir == 0:
            self._tb_auto, self._small_since = need, None
        else:
            now = time.perf_counter()
            if self._small_since is None:
                self._small_since = now
            elif now - self._small_since >= ZOOM_IN_HOLD_S:
                self._tb_auto, self._small_since = need, None
        return float(self._tb_auto)

    def _acc_clear(self):
        self._acc = None

    def _raster_size(self) -> tuple[int, int]:
        vb = self.pi.vb
        w = int(min(max(vb.width(), 300), 1600))
        h = int(min(max(vb.height() * (RASTER_Y[1] - RASTER_Y[0]) / 3.8, 60), 500))
        return w, h

    def _persistence_image(self, k, span, x0, x1, ks, ws, d, h) -> None:
        """Accumulate the sweeps of the persistence window (or, for the infinite one, all those seen since the last
        clear) into columns and draw them as an intensity-graded image."""
        wpx, hpx = self._raster_size()
        if span == float("inf"):
            key = (x0, x1, wpx, id(self.model))
            if self._acc is None or self._acc["key"] != key or k < self._acc["k"]:
                self._acc = {"key": key, "k": k - 1, "high": np.zeros(wpx), "edges": np.zeros(wpx), "total": 0.0}
            acc = self._acc
            n = k - acc["k"]
            step = max(1, -(-n // N_SWEEPS))
            kk = np.arange(k, acc["k"], -step, dtype=np.int64)
            dd, hh, _ = self.model.sweeps(kk)
            hi, ed, tot = edge_profile(dd, hh, np.ones(kk.size), x0, x1, wpx)
            acc["high"] += hi
            acc["edges"] += ed
            acc["total"] += tot
            acc["k"] = k
            high, edges, total = acc["high"], acc["edges"], acc["total"]
        else:
            high, edges, total = edge_profile(d, h, ws, x0, x1, wpx)
        y0, y1 = RASTER_Y
        row = lambda y: int(round((y - y0) / (y1 - y0) * (hpx - 1)))
        r_lo, r_hi = row(SLAVE_LEVELS[0]), row(SLAVE_LEVELS[1])
        img = np.zeros((wpx, hpx))
        thick = 2                                                        # rows per trace line
        img[:, r_lo:r_lo + thick] = (total - high)[:, None]
        img[:, r_hi:r_hi + thick] = high[:, None]
        img[:, r_lo + thick:r_hi] = edges[:, None]                      # the vertical edges join the two levels
        img[:, r_lo:r_lo + thick] += edges[:, None]
        img[:, r_hi:r_hi + thick] += edges[:, None]
        self.img.setImage(phosphor(img), autoLevels=False, levels=(0.0, 1.0))
        self.img.setRect(QtCore.QRectF(x0 * 1e-9, y0, (x1 - x0) * 1e-9, y1 - y0))
        self.img.setVisible(True)

    def _draw(self):
        if self.model is None:
            return
        k = int(round(self.pos))
        sw = self.model.sweep(k)
        # persistence (simulated time): the sweeps of the window, the newest at full weight and the older ones fading
        # linearly to black at the age of the persistence; infinite: nothing fades, all sweeps since the last clear
        span = self.persist.currentData() or 0.0
        sps = self.sweeps_per_s
        ks, ws = persistence_samples(k, min(span, SCALE_WINDOW_S) * sps, N_SWEEPS)      # (inf: the scale looks 60 s back)
        d, h, _ = self.model.sweeps(ks)
        tb = self._timebase(float(np.max(np.abs(d))))
        self.tb_ns = tb
        half_screen = DIVISIONS * tb
        x0, x1 = -half_screen, half_screen
        ns = 1e-9
        self.pi.setXRange(x0 * ns, x1 * ns, padding=0)
        for m, ln in zip(range(-DIVISIONS, DIVISIONS + 1), self.grid_lines):
            ln.setPos(m * tb * ns)
        gx, gy = square_xy(0.0, self.period_ns / 2.0, x0, x1, *GM_LEVELS)
        self.gm_curve.setData(gx * ns, gy)
        sx, sy = square_xy(sw.edge_ns, sw.half_ns, x0, x1, *SLAVE_LEVELS)
        self.slave_curve.setData(sx * ns, sy)
        if span > 0:
            self._persistence_image(k, span, x0, x1, ks, ws, d, h)
            self.slave_curve.setPen(pg.mkPen(C_SLAVE, width=1.5))
        else:
            self.img.setVisible(False)
            self.slave_curve.setPen(pg.mkPen(C_SLAVE, width=2.5))
        self.lbl_gm.setPos(x0 * ns, GM_LEVELS[1] + 0.05)
        self.lbl_slave.setPos(x0 * ns, SLAVE_LEVELS[1] + 0.05)
        self.lbl_trig.setPos(0.0, GM_LEVELS[1] + 0.4)
        warn = ""
        if sw.ambiguous:
            warn = (f'<br><span style="color:#E69F00">⚠ {T("scope_ambiguous", p=fmt_ns(self.period_ns, False))}</span>')
        self.info.setText(
            f"t = {sw.t_s:.3f} s · {T('scope_sweep')} #{k} · φ = {fmt_ns(sw.phi_ns)} · "
            f"{T('scope_edge')} = {fmt_ns(sw.edge_ns)} · {fmt_tb(tb)}{warn}")
        self.time_lbl.setText(f"{sw.t_s:.3f} s / {self._t_end:.3f} s")

    # ------------------------------------------------------------------ state (kept over a language change)
    def state(self) -> dict:
        return {"pos": self.pos, "speed": self.speed.currentIndex(), "tb": self.tb.currentIndex(),
                "persist": self.persist.currentIndex(), "loop": self.chk_loop.isChecked(),
                "geometry": self.saveGeometry()}

    def restore(self, st: dict) -> None:
        self.speed.setCurrentIndex(st["speed"])
        self.tb.setCurrentIndex(st["tb"])
        self.persist.setCurrentIndex(st["persist"])
        self.chk_loop.setChecked(st["loop"])
        self.restoreGeometry(st["geometry"])
        if self.model is not None:
            self.set_position(st["pos"])
        else:
            self.pos = st["pos"]

    # ------------------------------------------------------------------ closing
    def force_close(self):
        self._force = True
        self.timer.stop()
        self.close()

    def closeEvent(self, ev):
        self.set_direction(0)
        if self._force:
            ev.accept()
            return
        ev.ignore()
        self.hide()
