# SPDX-License-Identifier: Apache-2.0
"""Oscilloscope window: the GM and the slave FSYNC square waves (48 kHz by default), triggered on the GM rising edge.

Everything is computed on demand from the exact slave-clock breakpoints (``ptpsim.scope``): the position of the
playback is just a sweep number, the maths does not depend on the playback speed, and nothing is stored per edge, so the
length is the simulated duration whatever it is.  Playback and drawing run in the GUI thread (a sweep costs a bisect).

Channels below the GM trace: the wave locked to the PHC (the PTP servo's clock) and, when the run models it, the
follower's FSYNC (``fsync.mode`` shared or cascade); each has its own curve, persistence image and ambiguity flag.
"""
from __future__ import annotations

import time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from ..scope import (DEFAULT_FSYNC_HZ, ScopeModel, edge_profile, persistence_samples, phosphor, pick_timebase_ns,
                     square_xy)
from .i18n import T

C_GM, C_SLAVE, C_FSYNC = "#F0E442", "#56B4E9", "#009E73"   # Okabe-Ito yellow / sky blue / bluish green
GM_LEVELS, SLAVE_LEVELS = (2.0, 3.0), (0.0, 1.0)     # with one channel; ROW apart per extra channel
ROW = 2.0
TIMEBASES_NS = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5, 2e5]     # per division
SPEEDS = [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0]                                          # simulated s / wall s
FSYNC_HZ = [1_000, 8_000, 48_000]                    # selectable wave frequency (TDM frame syncs)
PERSIST_S = [0.0, 0.01, 0.1, 1.0, 10.0, 60.0, float('inf')]        # simulated time; inf = infinite persistence
DIVISIONS = 5                                        # per side of the trigger
N_SWEEPS = 2000                                      # most sweeps accumulated per frame (a longer window is thinned)
SCALE_WINDOW_S = 60.0                                # auto timebase: the sweeps of this much recent time must fit
RASTER_Y = (-0.25, 1.35)                             # screen range of a channel's image, relative to its low level
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

    def __init__(self, freq_hz: int = DEFAULT_FSYNC_HZ):
        super().__init__(None, QtCore.Qt.Window)
        self.freq_hz = int(freq_hz)
        self.period_ns = 1e9 / self.freq_hz
        self.ch: list[dict] = []                     # channels: key, label, color, bp, model, curve, img, lbl, acc
        self.n_max = 0
        self.pos = 0.0                               # playback position in sweeps (float while playing)
        self.dir = 0                                 # 0 stopped, +1 forward, -1 reverse
        self._force = False
        self._last_wall = 0.0
        self._tb_auto = TIMEBASES_NS[-1]
        self._small_since: float | None = None
        self._t_end = 0.0
        self._p_origin = 0                           # persistence memory starts here (a reset forgets the earlier sweeps)
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
        self.lbl_gm = pg.TextItem(T("scope_gm"), color=C_GM, anchor=(0, 1))
        self.lbl_trig = pg.TextItem("▼ " + T("scope_trig"), color=(255, 255, 255), anchor=(0.5, 1))
        for it in (self.lbl_gm, self.lbl_trig):
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
        tr.addWidget(QtWidgets.QLabel(T("scope_freq")))
        self.freq = QtWidgets.QComboBox()
        for hz in FSYNC_HZ:
            self.freq.addItem(f"{hz / 1e3:g} kHz", hz)
        self.freq.setCurrentIndex(FSYNC_HZ.index(self.freq_hz) if self.freq_hz in FSYNC_HZ else FSYNC_HZ.index(DEFAULT_FSYNC_HZ))
        self.freq.setToolTip(T("scope_freq_tip"))
        self.freq.currentIndexChanged.connect(lambda *_: self.set_frequency(self.freq.currentData()))
        tr.addWidget(self.freq)
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
        self.persist.currentIndexChanged.connect(lambda *_: self.reset_persistence())
        tr.addWidget(self.persist)
        self.btn_pclear = self._tbtn(st.standardIcon(QtWidgets.QStyle.SP_BrowserReload), "scope_tip_pclear",
                                     self.reset_persistence)
        tr.addWidget(self.btn_pclear)
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
    def _lut(color: str = C_SLAVE) -> np.ndarray:
        """Intensity grading: black -> the channel colour (one fresh trace) -> near white (hit again and again)."""
        v = np.linspace(0.0, 1.0, 256)[:, None]
        base = np.array(QtGui.QColor(color).getRgb()[:3], dtype=np.float64)[None, :]
        lo = base * np.minimum(1.0, v / 0.5)
        hi = base + (np.array([215.0, 240.0, 255.0]) - base) * np.clip((v - 0.5) / 0.5, 0.0, 1.0)
        return np.where(v <= 0.5, lo, hi).astype(np.uint8)

    # ------------------------------------------------------------------ channels
    @property
    def model(self) -> ScopeModel | None:
        """Model of the first channel (the PHC wave); ``None`` when there is nothing to show."""
        return self.ch[0]["model"] if self.ch else None

    @property
    def img(self):
        return self.ch[0]["img"] if self.ch else None

    @property
    def _acc(self) -> dict | None:
        return self.ch[0]["acc"] if self.ch else None

    def _clear_acc(self) -> None:
        for c in self.ch:
            c["acc"] = None

    def _levels(self, i: int) -> tuple[float, float]:
        y0 = (len(self.ch) - 1 - i) * ROW
        return y0, y0 + 1.0

    def _gm_levels(self) -> tuple[float, float]:
        y0 = max(1, len(self.ch)) * ROW
        return y0, y0 + 1.0

    def _make_channels(self, specs: list[tuple[str, str, str, dict]]) -> None:
        """(Re)create the graphics items when the set of channels changes; keep them otherwise."""
        keys = [k for k, *_ in specs]
        if keys != [c["key"] for c in self.ch]:
            for c in self.ch:
                for it in (c["img"], c["curve"], c["lbl"]):
                    self.pi.removeItem(it)
            self.ch = []
            for key, label, color, _bp in specs:
                img = pg.ImageItem()                 # persistence: intensity-graded accumulation of the sweeps
                img.setCompositionMode(QtGui.QPainter.CompositionMode_Plus)     # light adds up on the black screen
                img.setLookupTable(self._lut(color))
                img.setLevels((0.0, 1.0))
                img.setZValue(-10)
                img.setVisible(False)
                self.pi.addItem(img, ignoreBounds=True)
                curve = self.pi.plot(pen=pg.mkPen(color, width=2.5))
                lbl = pg.TextItem(label, color=color, anchor=(0, 1))
                self.pi.addItem(lbl, ignoreBounds=True)
                self.ch.append({"key": key, "label": label, "color": color, "bp": None, "model": None,
                                "img": img, "curve": curve, "lbl": lbl, "acc": None})
            self.pi.setYRange(-0.25, self._gm_levels()[1] + 0.55 + 0.3 * max(0, len(self.ch) - 1), padding=0)
        for c, (_k, _l, _c, bp) in zip(self.ch, specs):
            c["bp"] = bp
            c["model"] = ScopeModel.from_dict(bp, self.freq_hz)

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
                  self.slider, self.speed, self.freq, self.tb, self.persist, self.btn_pclear, self.chk_loop):
            w.setEnabled(on)
        self.notice.setVisible(not on)
        self.pw.setVisible(on)
        self.info.setVisible(on)

    # ------------------------------------------------------------------ data
    def set_data(self, bp: dict | None, t_end_s: float = 0.0, fsync_bp: dict | None = None):
        """New slave-clock segments (an explore result) and, if the run models it, the FSYNC segments; ``None`` when
        there is no explore result to show (live mode, nothing computed yet).  The playback position is kept."""
        if bp is None:
            self.set_direction(0)
            self._make_channels([])
            self.n_max = 0
            self._set_enabled(False)
            return
        specs = [("phc", T("scope_slave"), C_SLAVE, bp)]
        if fsync_bp is not None:
            specs.append(("fsync", T("scope_fsync"), C_FSYNC, fsync_bp))
        self._make_channels(specs)
        self._p_origin = int(round(self.pos))
        self._clear_acc()
        self.n_max = int(t_end_s * self.freq_hz)
        self.pos = min(self.pos, float(self.n_max))
        self.slider.blockSignals(True)
        self.slider.setRange(0, self.n_max)
        self.slider.blockSignals(False)
        self._t_end = t_end_s
        self._set_enabled(True)
        self._sync_slider()
        self._draw()

    def set_frequency(self, hz: int) -> None:
        """Wave frequency: the playback keeps its position in *time*, the sweep number is rescaled."""
        hz = int(hz)
        if hz == self.freq_hz:
            return
        t = self.pos / self.freq_hz
        self.freq_hz, self.period_ns = hz, 1e9 / hz
        self.freq.blockSignals(True)
        self.freq.setCurrentIndex(self.freq.findData(hz))
        self.freq.blockSignals(False)
        self._p_origin = 0
        self._clear_acc()
        if self.ch:
            self.set_data(self.ch[0]["bp"], self._t_end, self.ch[1]["bp"] if len(self.ch) > 1 else None)
        self.pos = float(min(max(round(t * hz), 0), self.n_max))
        self._p_origin = int(round(self.pos))
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

    def reset_persistence(self):
        """Forget the past: the persistence starts again from the current sweep, as if nothing had been drawn before."""
        self._p_origin = int(round(self.pos))
        self._clear_acc()
        self._draw()

    def _raster_size(self) -> tuple[int, int]:
        vb = self.pi.vb
        w = int(min(max(vb.width(), 300), 1600))
        h = int(min(max(vb.height() * (RASTER_Y[1] - RASTER_Y[0]) / 3.8, 60), 500))
        return w, h

    def _persistence_image(self, c: dict, i: int, k, span, x0, x1, ks, ws, d, h) -> None:
        """Accumulate the sweeps of the persistence window (or, for the infinite one, all those seen since the last
        clear) of channel ``c`` into columns and draw them as an intensity-graded image."""
        wpx, hpx = self._raster_size()
        model = c["model"]
        if span == float("inf"):
            key = (x0, x1, wpx, id(model))
            if c["acc"] is None or c["acc"]["key"] != key:                 # (new timebase: rebuilt from the origin)
                c["acc"] = {"key": key, "k": self._p_origin - 1, "high": np.zeros(wpx), "edges": np.zeros(wpx),
                            "total": 0.0}
            acc = c["acc"]
            n = k - acc["k"]
            step = max(1, -(-n // N_SWEEPS))
            kk = np.arange(k, acc["k"], -step, dtype=np.int64)
            dd, hh, _ = model.sweeps(kk)
            hi, ed, tot = edge_profile(dd, hh, np.ones(kk.size), x0, x1, wpx)
            acc["high"] += hi
            acc["edges"] += ed
            acc["total"] += tot
            acc["k"] = k
            high, edges, total = acc["high"], acc["edges"], acc["total"]
        else:
            high, edges, total = edge_profile(d, h, ws, x0, x1, wpx)
        base = self._levels(i)[0]
        y0, y1 = base + RASTER_Y[0], base + RASTER_Y[1]
        row = lambda y: int(round((y - y0) / (y1 - y0) * (hpx - 1)))
        r_lo, r_hi = row(base), row(base + 1.0)
        img = np.zeros((wpx, hpx))
        thick = 2                                                        # rows per trace line
        img[:, r_lo:r_lo + thick] = (total - high)[:, None]
        img[:, r_hi:r_hi + thick] = high[:, None]
        img[:, r_lo + thick:r_hi] = edges[:, None]                      # the vertical edges join the two levels
        img[:, r_lo:r_lo + thick] += edges[:, None]
        img[:, r_hi:r_hi + thick] += edges[:, None]
        c["img"].setImage(phosphor(img), autoLevels=False, levels=(0.0, 1.0))
        c["img"].setRect(QtCore.QRectF(x0 * 1e-9, y0, (x1 - x0) * 1e-9, y1 - y0))
        c["img"].setVisible(True)

    def _draw(self):
        if not self.ch:
            return
        k = int(round(self.pos))
        # persistence (simulated time): the sweeps of the window, the newest at full weight and the older ones fading
        # linearly to black at the age of the persistence; infinite: nothing fades, all sweeps since the last clear
        span = self.persist.currentData() or 0.0
        sps = self.sweeps_per_s
        if k < self._p_origin:                                   # moved back before the reset point: restart there
            self._p_origin = k
            self._clear_acc()
        for c in self.ch:
            if c["acc"] is not None and k < c["acc"]["k"]:       # moved back before what was accumulated
                self._p_origin = k
                self._clear_acc()
        ks, ws = persistence_samples(k, min(span, SCALE_WINDOW_S) * sps, N_SWEEPS, self._p_origin)   # (inf: 60 s back)
        per = []
        for c in self.ch:
            d, h, _ = c["model"].sweeps(ks)
            per.append((c["model"].sweep(k), d, h))
        tb = self._timebase(float(max(np.max(np.abs(d)) for _sw, d, _h in per)))
        self.tb_ns = tb
        half_screen = DIVISIONS * tb
        x0, x1 = -half_screen, half_screen
        ns = 1e-9
        self.pi.setXRange(x0 * ns, x1 * ns, padding=0)
        for m, ln in zip(range(-DIVISIONS, DIVISIONS + 1), self.grid_lines):
            ln.setPos(m * tb * ns)
        gm_lo, gm_hi = self._gm_levels()
        gx, gy = square_xy(0.0, self.period_ns / 2.0, x0, x1, gm_lo, gm_hi)
        self.gm_curve.setData(gx * ns, gy)
        warn, parts = "", []
        for i, (c, (sw, d, h)) in enumerate(zip(self.ch, per)):
            lo, hi = self._levels(i)
            sx, sy = square_xy(sw.edge_ns, sw.half_ns, x0, x1, lo, hi)
            c["curve"].setData(sx * ns, sy)
            if span > 0:
                self._persistence_image(c, i, k, span, x0, x1, ks, ws, d, h)
                c["curve"].setPen(pg.mkPen(c["color"], width=1.5))
            else:
                c["img"].setVisible(False)
                c["curve"].setPen(pg.mkPen(c["color"], width=2.5))
            c["lbl"].setPos(x0 * ns, hi + 0.05)
            name = "" if len(self.ch) == 1 else f'<span style="color:{c["color"]}">{c["label"]}</span>: '
            parts.append(f"{name}φ = {fmt_ns(sw.phi_ns)} · {T('scope_edge')} = {fmt_ns(sw.edge_ns)}")
            if sw.ambiguous and not warn:
                warn = (f'<br><span style="color:#E69F00">⚠ {T("scope_ambiguous", p=fmt_ns(self.period_ns, False))}'
                        f'</span>')
        self.lbl_gm.setPos(x0 * ns, gm_hi + 0.05)
        self.lbl_trig.setPos(0.0, gm_hi + 0.4)
        sw0 = per[0][0]
        self.info.setText(f"t = {sw0.t_s:.3f} s · {T('scope_sweep')} #{k} · " + " · ".join(parts) +
                          f" · {fmt_tb(tb)}{warn}")
        self.time_lbl.setText(f"{sw0.t_s:.3f} s / {self._t_end:.3f} s")

    # ------------------------------------------------------------------ state (kept over a language change)
    def state(self) -> dict:
        return {"pos": self.pos, "speed": self.speed.currentIndex(), "tb": self.tb.currentIndex(),
                "freq": self.freq.currentData(), "persist": self.persist.currentIndex(), "p_origin": self._p_origin, "loop": self.chk_loop.isChecked(),
                "geometry": self.saveGeometry()}

    def restore(self, st: dict) -> None:
        self.speed.setCurrentIndex(st["speed"])
        self.set_frequency(st["freq"])
        self.tb.setCurrentIndex(st["tb"])
        self.persist.setCurrentIndex(st["persist"])
        self.chk_loop.setChecked(st["loop"])
        self.restoreGeometry(st["geometry"])
        if self.model is not None:
            self.set_position(st["pos"])
        else:
            self.pos = st["pos"]
        self._p_origin = st["p_origin"]
        self._clear_acc()
        self._draw()

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
