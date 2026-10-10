# SPDX-License-Identifier: Apache-2.0
"""Interactive GUI (PySide6 + pyqtgraph), English (default) and Italian.

Two clearly separated modes:

* **Exploration**: every parameter change recomputes the *whole* trajectory from the same initial
  conditions and the same seed (debounced, computed in a worker *process*, stale results discarded).
* **Live**: the trajectory continues; parameter changes apply from the current instant (marked on the
  plots).  Playback speed only changes how much simulated time is requested per wall-clock tick; the
  mathematics of the simulation does not depend on it.

All simulation work happens in ``ptpsim.gui.worker``; this module only builds widgets and draws.
Controls live in tabs on the left; the plots share the time axis; the metrics table sits below them.
"""
from __future__ import annotations

import math
import multiprocessing as mp
import sys
import time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

from ..config import SimConfig, default_scenario, noisy_preset, set_path
from ..controllers import POLICIES, REGISTRY
from ..engine import interval_ps
from .i18n import LANGS, STRINGS, T, get_lang, set_lang
from .params import ParamRow
from .zoombox import ZoomViewBox
from .worker import worker_main

UNITS = {"ns": 1.0, "µs": 1e-3, "ms": 1e-6}
UNIT_DECIMALS = {"ns": 2, "µs": 4, "ms": 6}
# Okabe-Ito colour-blind safe palette
C_TRUE, C_EST, C_BASE, C_DELAY, C_REF, C_EVT, C_TRANS = ("#0072B2", "#E69F00", "#CC79A7", "#009E73", "#000000",
                                                         "#D55E00", "#555555")
MAX_LIVE_POINTS = 200_000
CLOCK_ROOTS = [("100 MHz (SYS_PLL1_DIV2/5, default)", 100_000_000), ("98.304 MHz (AUDIO_PLL/4)", 98_304_000),
               ("196.608 MHz (AUDIO_PLL/2)", 196_608_000), ("24 MHz (OSC_24M)", 24_000_000),
               ("25 MHz", 25_000_000)]
SETTINGS = ("PTP_sim", "ptpsim")


def step_xy(t: np.ndarray, y: np.ndarray, t_end: float):
    """Zero-order-hold polyline of a sampled estimate (the value is held until the next sample)."""
    if t.size == 0:
        return t, y
    xs = np.repeat(t, 2)[1:]
    ys = np.repeat(y, 2)[:-1]
    return np.append(xs, max(t_end, t[-1])), np.append(ys, y[-1])


def interval_text(n: int) -> str:
    s = interval_ps(n) / 1e12
    return f"2^{n} s = {s:g} s ({1 / s:g} Hz)"


class ResultReader(QtCore.QThread):
    """Moves worker replies from the multiprocessing queue to the GUI thread (signal)."""
    received = QtCore.Signal(object)

    def __init__(self, q, parent=None):
        super().__init__(parent)
        self.q = q
        self._stop = False

    def run(self):
        while not self._stop:
            try:
                msg = self.q.get(timeout=0.2)
            except Exception:       # queue.Empty
                continue
            self.received.emit(msg)

    def stop(self):
        self._stop = True


class MainWindow(QtWidgets.QMainWindow):
    DEBOUNCE_MS = 40
    LIVE_TICK_MS = 40

    def __init__(self, cfg: SimConfig | None = None, start_worker: bool = True, lang: str | None = None):
        super().__init__()
        if lang is None:
            lang = QtCore.QSettings(*SETTINGS).value("lang", "en")
        set_lang(lang)
        self.resize(1600, 980)
        self.base_cfg = (cfg or default_scenario()).copy()
        self._dirty: set[str] = set()        # widgets edited since the last load: only these override base_cfg
        self.rows: dict[str, ParamRow] = {}
        self.ctrl_rows: dict[str, ParamRow] = {}
        self.mode = "explore"
        self.gen = 0
        self.t_change: float | None = None
        self._pending_live: dict = {}
        self._first_autorange = True
        self.main_res: dict | None = None
        self.base_res: dict | None = None
        self.metrics: dict | None = None
        self.base_metrics: dict | None = None
        self.last_ms = {"gui": None, "compute": None, "sim": None}
        self.n_results = 0                   # explore results displayed (used by the latency benchmark)
        # live state
        self.live_state = "stopped"          # stopped | running | paused
        self.live_buf: dict[str, dict] = {}
        self.live_t = 0.0
        self.live_busy = False
        self.live_last_wall = 0.0
        self.live_resetting = False
        self._last_draw = 0.0
        self.evt_lines: list = []
        self.trans_lines: list = []
        self.lim_lines: list = []            # horizontal limit lines of the PI-terms plot

        if start_worker:
            self._start_worker()
        else:                                # widget/config tests without a worker process
            self.req_q = self.res_q = self.proc = self.reader = None
            self.latest = type("Gen", (), {"value": 0})()
        self.debounce = QtCore.QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(self.DEBOUNCE_MS)
        self.debounce.timeout.connect(self._debounced)
        self.live_timer = QtCore.QTimer(self)
        self.live_timer.setInterval(self.LIVE_TICK_MS)
        self.live_timer.timeout.connect(self._live_tick)
        self._build_ui()
        self._on_param_changed(None, 0.0)

    # ------------------------------------------------------------------ worker
    def _start_worker(self):
        ctx = mp.get_context("spawn")
        self.req_q = ctx.Queue()
        self.res_q = ctx.Queue()
        self.latest = ctx.Value("i", 0)
        self.proc = ctx.Process(target=worker_main, args=(self.req_q, self.res_q, self.latest), daemon=True)
        self.proc.start()
        self.reader = ResultReader(self.res_q, self)
        self.reader.received.connect(self._on_reply)
        self.reader.start()

    def _send(self, msg: dict) -> None:
        if self.req_q is not None:
            self.req_q.put(msg)

    def closeEvent(self, ev):
        try:
            self.live_timer.stop()
            self.latest.value = -1
            if self.proc is not None:
                self._send({"cmd": "quit"})
                self.reader.stop()
                self.reader.wait(1000)
                self.proc.join(1.0)
                if self.proc.is_alive():
                    self.proc.terminate()
        finally:
            super().closeEvent(ev)

    # ------------------------------------------------------------------ UI construction
    def _row(self, name, grid, r, label_key, *args, tip_key: str | None = None, **kw) -> ParamRow:
        if tip_key:
            kw["tooltip"] = T(tip_key)
        row = ParamRow(T(label_key), *args, **kw)
        row.name = name
        row.add_to(grid, r)
        row.changed.connect(self._on_param_changed)
        self.rows[name] = row
        return row

    @staticmethod
    def _group(title) -> tuple[QtWidgets.QGroupBox, QtWidgets.QGridLayout]:
        g = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(g)
        grid.setColumnStretch(2, 1)
        return g, grid

    @staticmethod
    def _tab(tabs: QtWidgets.QTabWidget, title: str) -> QtWidgets.QVBoxLayout:
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        scroll.setWidget(w)
        tabs.addTab(scroll, title)
        return lay

    def _build_ui(self):
        """(Re)build every widget in the current language from ``self.base_cfg``."""
        c = self.base_cfg
        self.rows.clear()
        self.ctrl_rows.clear()
        self.evt_lines.clear()
        self.trans_lines.clear()
        self.lim_lines.clear()
        self.setWindowTitle(T("title"))
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(split)

        # ================= left: tabs ====================================================
        tabs = QtWidgets.QTabWidget()
        tabs.setMinimumWidth(560)
        tabs.setMaximumWidth(640)
        self.tabs = tabs
        split.addWidget(tabs)

        # ---- tab Run -----------------------------------------------------------------
        lay = self._tab(tabs, T("tab_run"))
        g, grid = self._group(T("g_mode"))
        self.mode_explore = QtWidgets.QRadioButton(T("mode_explore"))
        self.mode_live = QtWidgets.QRadioButton(T("mode_live"))
        self.mode_explore.setChecked(self.mode == "explore")
        self.mode_live.setChecked(self.mode == "live")
        self.mode_explore.toggled.connect(self._on_mode_toggled)
        grid.addWidget(self.mode_explore, 0, 0, 1, 3)
        grid.addWidget(self.mode_live, 1, 0, 1, 3)
        btns = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton(T("btn_start"))
        self.btn_pause = QtWidgets.QPushButton(T("btn_pause"))
        self.btn_reset = QtWidgets.QPushButton(T("btn_reset"))
        for b, f in ((self.btn_start, self._live_start), (self.btn_pause, self._live_pause),
                     (self.btn_reset, self._live_reset)):
            b.clicked.connect(f)
            btns.addWidget(b)
        grid.addLayout(btns, 2, 0, 1, 3)
        self.speed = self._row("speed", grid, 3, "speed", [], 0.1, 500.0, 10.0, 0.5, decimals=1, suffix="×",
                               log=True, tip_key="speed_tip")
        self.speed.changed.disconnect(self._on_param_changed)
        self.live_policy = QtWidgets.QComboBox()
        self.live_policy.addItems([T("policy_keep"), T("policy_reset"), T("policy_bumpless")])
        grid.addWidget(QtWidgets.QLabel(T("policy_lbl")), 4, 0, 1, 2)
        grid.addWidget(self.live_policy, 4, 2)
        self.follow_win = self._row("follow", grid, 5, "follow_win", [], 5.0, 3600.0, 60.0, 5.0, decimals=0,
                                    suffix="s", slider=False)
        self.follow_win.changed.disconnect(self._on_param_changed)
        self.chk_follow = QtWidgets.QCheckBox(T("follow"))
        self.chk_follow.setChecked(True)
        grid.addWidget(self.chk_follow, 6, 0, 1, 3)
        lay.addWidget(g)

        g, grid = self._group(T("g_metrics"))
        self.m_band = self._row("m_band", grid, 0, "m_band", [], 1.0, 100_000.0, 1000.0, 10.0, decimals=0,
                                suffix="ns", log=True, tip_key="m_band_tip")
        self.m_dwell = self._row("m_dwell", grid, 1, "m_dwell", [], 0.0, 300.0, 10.0, 1.0, decimals=1, suffix="s",
                                 tip_key="m_dwell_tip")
        self.m_win = self._row("m_win", grid, 2, "m_win", [], 1.0, 3600.0, 60.0, 5.0, decimals=0, suffix="s")
        self.chk_ss_settle = QtWidgets.QCheckBox(T("m_from_settle"))
        grid.addWidget(self.chk_ss_settle, 3, 0, 1, 3)
        self.chk_ss_settle.toggled.connect(lambda *_: self._on_param_changed(None, 0.0, "__metrics__"))
        for r in (self.m_band, self.m_dwell, self.m_win):
            r.changed.disconnect(self._on_param_changed)
            r.changed.connect(lambda *_: self._on_param_changed(None, 0.0, "__metrics__"))
        lay.addWidget(g)

        g, grid = self._group(T("g_files"))
        for i, (key, fn) in enumerate((("btn_save", self._save_cfg), ("btn_load", self._load_cfg),
                                       ("btn_export", self._export))):
            b = QtWidgets.QPushButton(T(key))
            b.clicked.connect(fn)
            grid.addWidget(b, i // 2, i % 2, 1, 1 if i < 2 else 2)
        lay.addWidget(g)
        lay.addStretch(1)

        # ---- tab Controller -------------------------------------------------------------
        lay = self._tab(tabs, T("tab_ctrl"))
        g, grid = self._group(T("g_ctrl"))
        self.ctrl_combo = QtWidgets.QComboBox()
        self.ctrl_combo.addItems(list(REGISTRY))
        self.ctrl_combo.setCurrentText(c.controller.name)
        grid.addWidget(QtWidgets.QLabel(T("ctrl_lbl")), 0, 0)
        grid.addWidget(self.ctrl_combo, 0, 1, 1, 2)
        self.ctrl_grid = grid
        self.ctrl_combo.currentTextChanged.connect(self._on_controller_changed)
        self.ctrl_note = QtWidgets.QLabel()
        self.ctrl_note.setWordWrap(True)
        self.ctrl_note.setStyleSheet("color: gray")
        grid.addWidget(self.ctrl_note, 99, 0, 1, 3)
        lay.addWidget(g)
        g, grid = self._group(T("g_fw"))
        self._row("fw_clamp", grid, 0, "fw_clamp", ["firmware.cmd_clamp_ppm"], 0.0, 50_000.0,
                  c.firmware.cmd_clamp_ppm, 100.0, decimals=1, suffix="ppm", tip_key="fw_clamp_tip")
        self._row("fw_step", grid, 1, "fw_step", ["firmware.step_threshold_ns"], 1e-6, 1000.0,
                  c.firmware.step_threshold_ns / 1e9, 0.1, decimals=6, suffix="s", scale=1e9, log=True,
                  tip_key="fw_step_tip")
        lay.addWidget(g)
        lay.addStretch(1)
        self._build_ctrl_rows(c.controller.name, c.controller.params)

        # ---- tab PTP intervals ----------------------------------------------------------
        lay = self._tab(tabs, T("tab_ptp"))
        g, grid = self._group(T("g_int"))
        self._row("sync_log", grid, 0, "sync_n", ["intervals.sync_log"], -4, 2, c.intervals.sync_log, 1,
                  integer=True, tip_key="sync_tip")
        self.sync_txt = QtWidgets.QLabel()
        grid.addWidget(self.sync_txt, 1, 0, 1, 3)
        self.delay_mode = QtWidgets.QComboBox()
        self.delay_mode.addItems([T("dly_ind"), T("dly_every")])
        self.delay_mode.setCurrentIndex(0 if c.intervals.delay_mode == "interval" else 1)
        self.delay_mode.currentIndexChanged.connect(lambda *_: self._on_param_changed(None, 0.0, "intervals.delay_mode"))
        grid.addWidget(QtWidgets.QLabel(T("dly_lbl")), 2, 0)
        grid.addWidget(self.delay_mode, 2, 1, 1, 2)
        self._row("delay_log", grid, 3, "dly_n", ["intervals.delay_log"], -4, 2, c.intervals.delay_log, 1,
                  integer=True, tip_key="dly_tip")
        self.delay_txt = QtWidgets.QLabel()
        grid.addWidget(self.delay_txt, 4, 0, 1, 3)
        self._row("delay_n", grid, 5, "every_n", ["intervals.delay_every_n"], 1, 64, c.intervals.delay_every_n, 1,
                  integer=True)
        self.chk_rearm = QtWidgets.QCheckBox(T("rearm"))
        self.chk_rearm.setChecked(c.intervals.delay_rearm_from_handling)
        self.chk_rearm.toggled.connect(lambda *_: self._on_param_changed(None, 0.0, "intervals.delay_rearm_from_handling"))
        grid.addWidget(self.chk_rearm, 6, 0, 1, 3)
        lay.addWidget(g)
        lay.addStretch(1)

        # ---- tab Scenario ---------------------------------------------------------------
        lay = self._tab(tabs, T("tab_scen"))
        g, grid = self._group(T("g_init"))
        self._row("offset0", grid, 0, "off0", ["oscillator.initial_offset_ns"], -2.0e15, 2.0e15,
                  c.oscillator.initial_offset_ns / 1e3, 10.0, decimals=1, suffix="µs", scale=1e3, symlog=True,
                  decades=15.0, tip_key="off0_tip")
        self.btn_phc0 = QtWidgets.QPushButton(T("btn_phc0"))
        self.btn_phc0.clicked.connect(self._set_phc_zero)
        grid.addWidget(self.btn_phc0, 1, 0, 1, 3)
        self._row("freq0", grid, 2, "freq0", ["oscillator.freq_error_ppb"], -200.0, 200.0,
                  c.oscillator.freq_error_ppb / 1e3, 0.1, decimals=2, suffix="ppm", scale=1e3)
        self._row("duration", grid, 3, "duration", ["duration_s"], 10.0, 3600.0, c.duration_s, 10.0, decimals=0,
                  suffix="s")
        self._row("seed", grid, 4, "seed", ["seed"], 0, 99999, c.seed, 1, integer=True, slider=False)
        self._row("drift", grid, 5, "drift", ["oscillator.drift_ppb_per_s"], -50.0, 50.0,
                  c.oscillator.drift_ppb_per_s, 0.1, decimals=2, suffix="ppb/s")
        self._row("timer_err", grid, 6, "timer_err", ["oscillator.timer_error_ppb"], -100.0, 100.0,
                  c.oscillator.timer_error_ppb / 1e3, 0.5, decimals=1, suffix="ppm", scale=1e3, tip_key="timer_tip")
        lay.addWidget(g)
        lay.addStretch(1)

        # ---- tab Network & noise --------------------------------------------------------
        lay = self._tab(tabs, T("tab_net"))
        g, grid = self._group(T("g_net"))
        n = c.network
        self._row("d_mean", grid, 0, "d_mean", ["network.delay_ms_ns"], 0.0, 100_000.0,
                  (n.delay_ms_ns + n.delay_sm_ns) / 2, 10.0, decimals=0, suffix="ns")
        self._row("d_asym", grid, 1, "d_asym", ["network.delay_asymmetry_ns"], -20_000.0, 20_000.0,
                  n.delay_ms_ns - n.delay_sm_ns, 10.0, decimals=0, suffix="ns", tip_key="d_asym_tip")
        self._row("net_jit", grid, 2, "net_jit", ["network.jitter_ms.scale_ns", "network.jitter_sm.scale_ns"],
                  0.0, 5000.0, n.jitter_ms.scale_ns, 10.0, decimals=0, suffix="ns")
        self._row("tx_jit", grid, 3, "tx_jit", ["tx_jitter.sync.scale_ns", "tx_jitter.follow_up.scale_ns",
                                                "tx_jitter.delay_req.scale_ns", "tx_jitter.delay_resp.scale_ns"],
                  0.0, 200.0, c.tx_jitter.sync.scale_ns / 1e3, 1.0, decimals=1, suffix="µs", scale=1e3,
                  tip_key="tx_jit_tip")
        self._row("lat_fup", grid, 4, "lat_fup", ["latency.follow_up_ns"], 0.0, 50_000.0,
                  c.latency.follow_up_ns / 1e3, 10.0, decimals=0, suffix="µs", scale=1e3)
        self._row("lat_dresp", grid, 5, "lat_dresp", ["latency.delay_resp_ns"], 0.0, 50_000.0,
                  c.latency.delay_resp_ns / 1e3, 10.0, decimals=0, suffix="µs", scale=1e3)
        self._row("lat_cmd", grid, 6, "lat_cmd", ["latency.command_ns"], 0.0, 100_000.0,
                  c.latency.command_ns / 1e3, 10.0, decimals=0, suffix="µs", scale=1e3)
        self._row("lat_step", grid, 7, "lat_step", ["latency.step_ns"], 0.0, 1000.0, c.latency.step_ns / 1e3, 1.0,
                  decimals=1, suffix="µs", scale=1e3, tip_key="lat_step_tip")
        self._row("ts_noise", grid, 8, "ts_noise", ["timestamps.gm_noise_sigma_ns", "timestamps.slave_noise_sigma_ns"],
                  0.0, 100.0, c.timestamps.gm_noise_sigma_ns, 0.5, decimals=1, suffix="ns")
        self._row("gm_q", grid, 9, "gm_q", ["timestamps.gm_quantum_ns"], 0.0, 100.0, c.timestamps.gm_quantum_ns, 1.0,
                  decimals=0, suffix="ns")
        self._row("loss", grid, 10, "loss", ["loss.sync", "loss.follow_up", "loss.delay_req", "loss.delay_resp"],
                  0.0, 50.0, c.loss.sync * 100, 0.5, decimals=1, suffix="%", scale=0.01)
        self.btn_noisy = QtWidgets.QPushButton(T("btn_noisy"))
        self.btn_noisy.clicked.connect(self._apply_noisy_preset)
        grid.addWidget(self.btn_noisy, 11, 0, 1, 3)
        lay.addWidget(g)
        lay.addStretch(1)

        # ---- tab Actuator ---------------------------------------------------------------
        lay = self._tab(tabs, T("tab_act"))
        g, grid = self._group(T("g_act"))
        self.act_combo = QtWidgets.QComboBox()
        self.act_combo.addItems([T("act_ideal"), T("act_nxp")])
        self.act_combo.setCurrentIndex(0 if c.actuator.kind == "ideal" else 1)
        self.act_combo.currentIndexChanged.connect(lambda *_: self._on_param_changed(None, 0.0, "actuator.kind"))
        grid.addWidget(QtWidgets.QLabel(T("act_type")), 0, 0)
        grid.addWidget(self.act_combo, 0, 1, 1, 2)
        self.root_combo = QtWidgets.QComboBox()
        for name, hz in CLOCK_ROOTS:
            self.root_combo.addItem(name, hz)
        i = self.root_combo.findData(c.actuator.clock_hz)
        self.root_combo.setCurrentIndex(max(0, i))
        self.root_combo.currentIndexChanged.connect(lambda *_: self._on_param_changed(None, 0.0, "actuator.clock_hz"))
        grid.addWidget(QtWidgets.QLabel(T("act_root")), 1, 0)
        grid.addWidget(self.root_combo, 1, 1, 1, 2)
        self.act_info = QtWidgets.QLabel("")
        self.act_info.setWordWrap(True)
        grid.addWidget(self.act_info, 2, 0, 1, 3)
        lay.addWidget(g)
        lay.addStretch(1)

        # ================= right: toolbar, plots, table =================================
        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(4, 4, 4, 4)
        # two toolbar rows: in one row the toolbar alone needed ~1450 px and forced the window beyond a
        # 1920 px screen; row 1 = units, view, status, language; row 2 = what the plots show
        tb = QtWidgets.QHBoxLayout()
        tb_show = QtWidgets.QHBoxLayout()
        tb.addWidget(QtWidgets.QLabel(T("units")))
        self.units = QtWidgets.QComboBox()
        self.units.addItems(list(UNITS))
        self.units.setCurrentText("µs")
        self.units.currentTextChanged.connect(self._on_units_changed)
        tb.addWidget(self.units)
        self.chk_overlay = QtWidgets.QCheckBox(T("overlay"))
        self.chk_overlay.toggled.connect(self._on_overlay_toggled)
        tb_show.addWidget(self.chk_overlay)
        self.chk_est = QtWidgets.QCheckBox(T("show_est"))
        self.chk_est.setChecked(True)
        self.chk_est.toggled.connect(lambda *_: self._redraw())
        tb_show.addWidget(self.chk_est)
        self.chk_diag = QtWidgets.QCheckBox(T("show_diag"))
        self.chk_diag.toggled.connect(self._on_diag_toggled)
        tb_show.addWidget(self.chk_diag)
        self.chk_pi = QtWidgets.QCheckBox(T("show_pi"))
        self.chk_pi.toggled.connect(self._on_pi_toggled)
        tb_show.addWidget(self.chk_pi)
        self.chk_trans = QtWidgets.QCheckBox(T("show_trans"))
        self.chk_trans.setChecked(True)
        self.chk_trans.toggled.connect(lambda *_: self._update_transient_lines())
        tb_show.addWidget(self.chk_trans)
        tb.addWidget(QtWidgets.QLabel(T("view")))
        self.view_combo = QtWidgets.QComboBox()
        self.view_combo.addItems([T("view_full"), T("view_trans"), T("view_steady")])
        self.view_combo.currentIndexChanged.connect(lambda *_: self._apply_view())
        tb.addWidget(self.view_combo)
        btn_fit = QtWidgets.QPushButton(T("btn_fit"))
        btn_fit.clicked.connect(self._autorange)
        tb.addWidget(btn_fit)
        self.status = QtWidgets.QLabel(T("st_start"))
        # Ignored: the status text (its length changes with every message) never widens the window
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.status.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        tb.addWidget(self.status, 1)
        tb.addWidget(QtWidgets.QLabel(T("lang")))
        self.lang_combo = QtWidgets.QComboBox()
        for code, name in LANGS.items():
            self.lang_combo.addItem(name, code)
        self.lang_combo.setCurrentIndex(self.lang_combo.findData(get_lang()))
        self.lang_combo.currentIndexChanged.connect(self._on_lang_changed)
        tb.addWidget(self.lang_combo)
        rl.addLayout(tb)
        tb_show.addStretch(1)
        rl.addLayout(tb_show)

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        self.pw_delay, self.pw_off, self.pw_diag, self.pw_pi = (pg.PlotWidget(viewBox=ZoomViewBox()) for _ in range(4))
        self.p_delay, self.p_off, self.p_diag, self.p_pi = (w.getPlotItem() for w in (self.pw_delay, self.pw_off,
                                                                                      self.pw_diag, self.pw_pi))
        self.p_off.setXLink(self.p_delay)
        self.p_diag.setXLink(self.p_delay)
        self.p_pi.setXLink(self.p_delay)
        for p in (self.p_delay, self.p_off, self.p_diag, self.p_pi):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setMenuEnabled(True)
            p.getAxis("left").setWidth(78)                # aligned plot areas
            for ax in ("left", "bottom"):
                p.getAxis(ax).enableAutoSIPrefix(False)     # explicit units only, no "x0.001" scaling
            p.getViewBox().setAutoVisible(y=True)           # y autorange follows the visible x range
            # the PI panel is short: its 4 entries go in 2 columns so the legend is not cut
            p.addLegend(offset=(-10, 10), colCount=2 if p is self.p_pi else 1)
        self.p_delay.setTitle(T("pl_delay"))
        self.p_off.setTitle(T("pl_off"))
        self.p_diag.setTitle(T("pl_diag"))
        self.p_off.setLabel("bottom", T("ax_time"))
        self.p_diag.setLabel("bottom", T("ax_time"))
        self.p_diag.setLabel("left", "ppb")
        self.pw_diag.setVisible(False)
        self.p_pi.setTitle(T("pl_pi"))
        self.p_pi.setLabel("bottom", T("ax_time"))
        self.p_pi.setLabel("left", T("ax_pi"))
        self.pw_pi.setVisible(False)
        pen = lambda col, w=1.5, st=None: pg.mkPen(col, width=w, style=st or QtCore.Qt.SolidLine)
        self.cv = {
            "delay_step": self.p_delay.plot(pen=pen(C_DELAY, 1.5), name=T("lg_delay_step")),
            "delay_pts": self.p_delay.plot(pen=None, symbol="o", symbolSize=5, symbolBrush=C_DELAY, symbolPen=None,
                                           name=T("lg_delay_pts")),
            "delay_ref": self.p_delay.plot(pen=pen(C_REF, 1.5, QtCore.Qt.DashLine), name=T("lg_delay_ref")),
            "true": self.p_off.plot(pen=pen(C_TRUE, 2.0), name=T("lg_true")),
            "est": self.p_off.plot(pen=pen(C_EST, 1.0), symbol="o", symbolSize=4, symbolBrush=C_EST, symbolPen=None,
                                   name=T("lg_est")),
            "base_true": self.p_off.plot(pen=pen(C_BASE, 1.5, QtCore.Qt.DashLine), name=T("lg_base_true")),
            "base_est": self.p_off.plot(pen=pen(C_BASE, 1.0, QtCore.Qt.DotLine), name=T("lg_base_est")),
            "rate_cmd": self.p_diag.plot(pen=pen(C_EST, 1.5), name=T("lg_cmd")),
            "rate_eff": self.p_diag.plot(pen=pen(C_TRUE, 1.5), name=T("lg_eff")),
            # PI terms: colour AND line style/width tell the curves apart
            "pi_p": self.p_pi.plot(pen=pen(C_EST, 1.5, QtCore.Qt.DashLine), name=T("lg_p")),
            "pi_i": self.p_pi.plot(pen=pen(C_TRUE, 2.0), name=T("lg_i")),
            "pi_out": self.p_pi.plot(pen=pen(C_REF, 1.0), name=T("lg_out")),
            "pi_applied": self.p_pi.plot(pen=pen(C_DELAY, 2.5), name=T("lg_applied")),
        }
        # the thin output (= P + I for the PI laws) goes behind I and P, otherwise it hides them
        for z, k in enumerate(("pi_out", "pi_applied", "pi_i", "pi_p")):
            self.cv[k].setZValue(z)
        for k in ("true", "base_true"):
            self.cv[k].setDownsampling(auto=True, method="peak")
            self.cv[k].setClipToView(True)
        plots = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        for w in (self.pw_delay, self.pw_off, self.pw_diag, self.pw_pi):
            plots.addWidget(w)
        plots.setSizes([300, 420, 180, 240])
        plots.setChildrenCollapsible(False)
        self.plots_split = plots

        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(22)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.table.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self._set_table_header()
        rl.addLayout(self._build_zoom_row())
        rl.addWidget(plots, 1)                              # plots take all the height the table does not need
        rl.addWidget(self.table, 0)
        split.addWidget(right)
        split.setStretchFactor(1, 1)
        self._update_interval_labels()
        self._update_enabled()

    # ------------------------------------------------------------------ language
    def _on_lang_changed(self, _i):
        lang = self.lang_combo.currentData()
        if lang == get_lang():
            return
        QtCore.QSettings(*SETTINGS).setValue("lang", lang)
        # keep the configuration, restart the (live) session: the widgets are rebuilt in the new language
        cfg = self.build_config()
        self.live_timer.stop()
        self.live_state = "stopped"
        self.live_busy = False
        self.base_cfg = cfg
        self._dirty.clear()
        set_lang(lang)
        old = self.centralWidget()
        self.mode = "explore"
        self._build_ui()
        old.deleteLater()
        self._first_autorange = True
        self.main_res = self.base_res = self.metrics = self.base_metrics = None
        self._on_param_changed(None, 0.0)

    # ------------------------------------------------------------------ controller rows
    def _build_ctrl_rows(self, name: str, params: dict[str, float]):
        for r in self.ctrl_rows.values():
            for w in (r.label, r.spin, r.slider):
                if w is not None:
                    self.ctrl_grid.removeWidget(w)
                    w.deleteLater()
            self.rows.pop(r.name, None)
        self.ctrl_rows.clear()
        cls = REGISTRY[name]
        for i, (k, (dflt, lo, hi, desc)) in enumerate(cls.PARAMS.items(), start=1):
            val = params.get(k, dflt)
            tip = T("pd_" + k) if ("pd_" + k) in STRINGS else desc
            row = ParamRow(k, [f"controller.params.{k}"], lo, hi, val, (hi - lo) / 200.0 if hi < 100 else 1.0,
                           decimals=3 if hi <= 20 else 1, tooltip=tip)
            row.name = "ctrl." + k
            row.add_to(self.ctrl_grid, i)
            row.changed.connect(self._on_param_changed)
            self.rows["ctrl." + k] = row
            self.ctrl_rows[k] = row
        self.ctrl_note.setText(T("cn_" + name))

    def _on_controller_changed(self, name: str):
        self._build_ctrl_rows(name, {})
        self._on_param_changed(None, 0.0, "controller.name")

    # ------------------------------------------------------------------ configuration <-> widgets
    def _row_overrides(self, row: ParamRow) -> dict:
        v = row.config_value()
        out = {p: v for p in row.paths}
        if row.integer:
            out = {p: int(round(x)) for p, x in out.items()}
        if row is self.rows.get("d_mean") or row is self.rows.get("d_asym"):
            mean, asym = self.rows["d_mean"].config_value(), self.rows["d_asym"].config_value()
            out = {"network.delay_ms_ns": mean + asym / 2, "network.delay_sm_ns": mean - asym / 2,
                   "network.delay_asymmetry_ns": 0.0}
        if row is self.rows.get("fw_step"):              # an integer number of ns, as in clock.c
            out = {"firmware.step_threshold_ns": int(round(v))}
        return out

    def _all_overrides(self) -> dict:
        """Overrides of ``base_cfg`` coming from the widgets.

        Only widgets the user *edited since the last load* (``_dirty``) override their fields, so a loaded
        configuration is reproduced exactly even where a widget cannot represent it (clamped range,
        per-type jitter/loss collapsed into one control, ...).  Combos and the controller are always applied.
        """
        ov: dict = {}
        for name, row in self.rows.items():
            if row.paths and name in self._dirty and not name.startswith("ctrl."):
                ov.update(self._row_overrides(row))
        ov["intervals.delay_mode"] = "interval" if self.delay_mode.currentIndex() == 0 else "every_n_sync"
        ov["intervals.delay_rearm_from_handling"] = self.chk_rearm.isChecked()
        ov["actuator.kind"] = "ideal" if self.act_combo.currentIndex() == 0 else "nxp"
        if "root" in self._dirty:
            ov["actuator.clock_hz"] = int(self.root_combo.currentData())
        ov["controller.name"] = self.ctrl_combo.currentText()
        same = ov["controller.name"] == self.base_cfg.controller.name
        for k, row in self.ctrl_rows.items():
            if (not same) or ("ctrl." + k) in self._dirty:
                ov[f"controller.params.{k}"] = row.config_value()
        if "tx_jit" in self._dirty:
            for p in ("sync", "follow_up", "delay_req", "delay_resp"):
                ov[f"tx_jitter.{p}.kind"] = "normal" if ov[f"tx_jitter.{p}.scale_ns"] > 0 else "none"
        if "net_jit" in self._dirty:
            for p in ("jitter_ms", "jitter_sm"):
                ov[f"network.{p}.kind"] = "exponential" if ov[f"network.{p}.scale_ns"] > 0 else "none"
        return ov

    def build_config(self) -> SimConfig:
        cfg = self.base_cfg.copy()
        ov = self._all_overrides()
        if ov["controller.name"] != cfg.controller.name:
            cfg.controller.name = ov["controller.name"]
            cfg.controller.params = {}
        for k, v in ov.items():
            if k == "controller.name":
                continue
            set_path(cfg, k, v)
        return cfg

    def _metrics_cfg(self) -> dict:
        return {"band_ns": self.m_band.config_value(), "dwell_s": self.m_dwell.config_value(),
                "final_window_s": self.m_win.config_value(), "steady_from_settling": self.chk_ss_settle.isChecked()}

    def _apply_cfg_to_widgets(self, c: SimConfig):
        """Load a configuration: widgets show it, ``base_cfg`` keeps it exactly (nothing is dirty)."""
        self.base_cfg = c.copy()
        self._dirty.clear()
        self.ctrl_combo.blockSignals(True)
        self.ctrl_combo.setCurrentText(c.controller.name)
        self.ctrl_combo.blockSignals(False)
        self._build_ctrl_rows(c.controller.name, c.controller.params)
        vals = {"sync_log": c.intervals.sync_log, "delay_log": c.intervals.delay_log, "delay_n": c.intervals.delay_every_n,
                "offset0": c.oscillator.initial_offset_ns, "freq0": c.oscillator.freq_error_ppb,
                "duration": c.duration_s, "seed": c.seed, "drift": c.oscillator.drift_ppb_per_s,
                "timer_err": c.oscillator.timer_error_ppb,
                "d_mean": (c.network.delay_ms_ns + c.network.delay_sm_ns) / 2,
                "d_asym": c.network.delay_ms_ns - c.network.delay_sm_ns,
                "net_jit": c.network.jitter_ms.scale_ns, "tx_jit": c.tx_jitter.sync.scale_ns,
                "lat_fup": c.latency.follow_up_ns, "lat_dresp": c.latency.delay_resp_ns, "lat_cmd": c.latency.command_ns,
                "lat_step": c.latency.step_ns,
                "ts_noise": c.timestamps.gm_noise_sigma_ns, "gm_q": c.timestamps.gm_quantum_ns, "loss": c.loss.sync,
                "fw_clamp": c.firmware.cmd_clamp_ppm, "fw_step": c.firmware.step_threshold_ns}
        for k, v in vals.items():
            self.rows[k].set_config_value(v)
        for w, fn in ((self.delay_mode, lambda: self.delay_mode.setCurrentIndex(0 if c.intervals.delay_mode == "interval" else 1)),
                      (self.act_combo, lambda: self.act_combo.setCurrentIndex(0 if c.actuator.kind == "ideal" else 1)),
                      (self.chk_rearm, lambda: self.chk_rearm.setChecked(c.intervals.delay_rearm_from_handling))):
            w.blockSignals(True)
            fn()
            w.blockSignals(False)
        self.root_combo.blockSignals(True)
        i = self.root_combo.findData(c.actuator.clock_hz)
        if i >= 0:
            self.root_combo.setCurrentIndex(i)
        self.root_combo.blockSignals(False)
        self._first_autorange = True
        self._update_interval_labels()
        self._update_enabled()
        if self.mode == "live":
            self._live_reset()
        else:
            self.t_change = time.perf_counter()
            self.debounce.start()

    def _apply_noisy_preset(self):
        c = noisy_preset()
        self.rows["net_jit"].set_config_value(c.network.jitter_ms.scale_ns)
        self.rows["tx_jit"].set_config_value(c.tx_jitter.sync.scale_ns)
        self.rows["ts_noise"].set_config_value(c.timestamps.gm_noise_sigma_ns)
        self._dirty.update({"net_jit", "tx_jit", "ts_noise"})
        for name in ("net_jit", "tx_jit", "ts_noise"):
            self._on_param_changed(self.rows[name], 0.0)

    def _set_phc_zero(self):
        """Slave PHC at 0: offset = -epoch (the GM runs at the 1.7e18 ns epoch)."""
        self.rows["offset0"].set_config_value(-float(self.base_cfg.epoch_ns), emit=True)

    # ------------------------------------------------------------------ change handling
    def _update_interval_labels(self):
        sl = int(self.rows["sync_log"].config_value())
        self.sync_txt.setText(T("sync_txt", t=interval_text(sl)))
        t = T("dly_txt", t=interval_text(int(self.rows["delay_log"].config_value())))
        if self.delay_mode.currentIndex() == 1:
            n = int(self.rows["delay_n"].config_value())
            t += T("dly_txt_n", n=n, s=n * interval_ps(sl) / 1e12)
        self.delay_txt.setText(t)

    def _update_enabled(self):
        every_n = self.delay_mode.currentIndex() == 1
        self.rows["delay_log"].set_enabled(not every_n)
        self.rows["delay_n"].set_enabled(every_n)
        self.chk_rearm.setEnabled(not every_n)
        self.root_combo.setEnabled(self.act_combo.currentIndex() == 1)
        live = self.mode == "live"
        for k in ("offset0", "freq0", "seed", "duration"):
            self.rows[k].set_enabled(not live or self.live_state == "stopped")
        for b in (self.btn_start, self.btn_pause, self.btn_reset):
            b.setEnabled(live)
        self.speed.set_enabled(live)
        self.live_policy.setEnabled(live)
        self.follow_win.set_enabled(live)
        self.chk_follow.setEnabled(live)

    def _on_units_changed(self, *_):
        self._redraw()

    def _on_param_changed(self, row, value, tag: str | None = None):
        self._update_interval_labels()
        self._update_enabled()
        self.t_change = time.perf_counter()
        if row is not None and getattr(row, "paths", None):
            self._dirty.add(row.name)
        if tag == "actuator.clock_hz":
            self._dirty.add("root")
        if tag == "__metrics__":
            if self.mode == "explore":
                self.debounce.start()
            return
        if self.mode == "live":
            if self.live_state == "stopped":
                self.debounce.start()            # not started yet: rebuild the session from the widgets
                return
            ov = self._all_overrides()
            if row is not None:
                changed = self._row_overrides(row)
            else:
                changed = {k: ov[k] for k in ([tag] if tag and not tag.startswith("__") else [])}
            if tag == "controller.name":
                changed = {k: v for k, v in ov.items() if k.startswith("controller.")}
                changed["controller.name"] = ov["controller.name"]
            if tag == "actuator.clock_hz" or tag == "actuator.kind":
                changed = {"actuator.kind": ov["actuator.kind"],
                           "actuator.clock_hz": ov.get("actuator.clock_hz", self.base_cfg.actuator.clock_hz)}
            if row is not None:
                for p in row.paths:
                    base = p.rsplit(".", 1)[0]
                    if (p.startswith("tx_jitter") or p.startswith("network.jitter")) and (base + ".kind") in ov:
                        changed[base + ".kind"] = ov[base + ".kind"]
            self._pending_live.update(changed)
            self.debounce.start()
        else:
            self.debounce.start()

    def _debounced(self):
        if self.mode == "explore":
            self._request_explore()
        elif self.live_state == "stopped":
            self._live_reset()                   # changes before Start apply from t = 0
        elif self._pending_live:
            ov, self._pending_live = self._pending_live, {}
            policy = POLICIES[self.live_policy.currentIndex()]
            self._send({"cmd": "live_update", "gen": self.gen, "overrides": ov, "policy": policy})

    # ------------------------------------------------------------------ explore
    def _request_explore(self):
        self.gen += 1
        self.latest.value = self.gen
        cfg = self.build_config()
        self.status.setText(T("st_calc"))
        self._send({"cmd": "explore", "gen": self.gen, "cfg": cfg.to_dict(),
                    "overlay": self.chk_overlay.isChecked(), "mc": self._metrics_cfg()})

    def _on_overlay_toggled(self, on):
        if self.mode == "explore":
            self._request_explore()
        else:
            self._live_reset()

    def _build_zoom_row(self) -> QtWidgets.QHBoxLayout:
        """Which axes the wheel / right-drag zoom acts on: x is shared by all plots (they are x-linked),
        y is chosen per plot.  The pan (left drag) is not restricted."""
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(T("zoom_lbl")))
        self.chk_zoom_x = QtWidgets.QCheckBox(T("zoom_x"))
        self.chk_zoom_x.setToolTip(T("zoom_x_tip"))
        self.chk_zoom_x.setChecked(True)
        self.chk_zoom_x.toggled.connect(self._on_zoom_changed)
        row.addWidget(self.chk_zoom_x)
        self.chk_zoom_y = {}
        for key, pw in (("delay", self.pw_delay), ("off", self.pw_off), ("diag", self.pw_diag), ("pi", self.pw_pi)):
            c = QtWidgets.QCheckBox(T("zoom_y_" + key))
            c.setToolTip(T("zoom_y_tip"))
            c.setChecked(True)
            c.setVisible(key in ("delay", "off"))        # diag / PI follow their plot's visibility
            c.toggled.connect(self._on_zoom_changed)
            row.addWidget(c)
            self.chk_zoom_y[key] = c
        row.addStretch(1)
        self._on_zoom_changed()
        return row

    def _on_zoom_changed(self, *_):
        zx = self.chk_zoom_x.isChecked()
        for key, pw in (("delay", self.pw_delay), ("off", self.pw_off), ("diag", self.pw_diag), ("pi", self.pw_pi)):
            pw.getViewBox().zoom_enabled = [zx, self.chk_zoom_y[key].isChecked()]

    def _on_diag_toggled(self, on):
        self.chk_zoom_y["diag"].setVisible(on)
        self.pw_diag.setVisible(on)
        if on:
            self._share_plot_height()
        self._redraw()

    def _on_pi_toggled(self, on):
        self.chk_zoom_y["pi"].setVisible(on)
        self.pw_pi.setVisible(on)
        if on:
            self._share_plot_height()
        self._redraw()

    def _share_plot_height(self):
        """Split the plot column among the plots that are switched on (the splitter handles still resize them).

        Without this a plot shown after another one gets only the splitter's minimum height."""
        total = sum(self.plots_split.sizes())
        weights = [2, 3, 2 if self.chk_diag.isChecked() else 0, 3 if self.chk_pi.isChecked() else 0]
        self.plots_split.setSizes([int(total * w / sum(weights)) for w in weights])

    def _on_mode_toggled(self, explore_on: bool):
        self.mode = "explore" if explore_on else "live"
        self.live_timer.stop()
        self.live_state = "stopped"
        self._first_autorange = True
        self._clear_plots()
        self._update_enabled()
        if self.mode == "explore":
            self._request_explore()
        else:
            self._live_reset()

    # ------------------------------------------------------------------ replies
    def _on_reply(self, msg):
        t = msg.get("type")
        if t == "error":
            self.status.setText(T("st_err"))
            print(msg["msg"], file=sys.stderr)
            self.live_busy = False
            return
        if t == "exported":
            self.status.setText(T("st_exported", p=msg["path"]))
            return
        if msg.get("gen") != self.gen and t in ("explore", "cancelled", "live_reset", "live", "live_updated"):
            if t == "live":
                self.live_busy = False
            return                      # stale
        if t == "explore":
            self.main_res, self.base_res = msg["main"], msg.get("base")
            self.metrics, self.base_metrics = msg["metrics"], msg.get("base_metrics")
            self.last_ms.update(compute=msg["compute_ms"], sim=msg["metrics"]["sim_wall_ms"])
            self.act_info.setText(self._act_text(msg["main"].get("act")))
            self._redraw()
            if self._first_autorange:
                self._autorange()
                self._first_autorange = False
            elif self.view_combo.currentIndex() != 0:
                self._apply_view()
            self.glw_repaint()
            lat = (time.perf_counter() - self.t_change) * 1e3 if self.t_change else float("nan")
            self.last_ms["gui"] = lat
            self.n_results += 1
            self.status.setText(T("st_done", lat=lat, c=msg["compute_ms"], s=msg["metrics"]["sim_wall_ms"]))
            self._fill_table()
        elif t == "live_reset":
            self.live_resetting = False
            self.live_buf = {}
            self.live_t = 0.0
            self._ingest_live(msg["delta"])
            self._redraw_live(force=True)
            self.status.setText(T("st_live_ready"))
        elif t == "live":
            self.live_busy = False
            self._ingest_live(msg["delta"])
            self._redraw_live()

    def glw_repaint(self):
        """Paint now, so the measured latency includes the rendering."""
        for w in (self.pw_delay, self.pw_off, self.pw_diag, self.pw_pi):
            if w.isVisible():
                w.viewport().repaint()

    @staticmethod
    def _act_text(a):
        if not a:
            return ""
        if a["kind"] == "ideal":
            return T("act_info_ideal", r=a["ratio"])
        return T("act_info_nxp", inc=a["inc"], ic=a["inc_corr"], cor=a["cor"], r=a["ratio"])

    # ------------------------------------------------------------------ live
    def _live_reset(self):
        self.live_timer.stop()
        self.live_state = "stopped"
        self.live_busy = False
        self.live_resetting = True
        self.gen += 1
        self.latest.value = self.gen
        self._pending_live.clear()
        self._clear_plots()
        self._first_autorange = True
        self._update_enabled()
        self._send({"cmd": "live_reset", "gen": self.gen, "cfg": self.build_config().to_dict(),
                    "overlay": self.chk_overlay.isChecked()})

    def _live_start(self):
        if self.mode != "live" or self.live_resetting:
            return
        self.live_state = "running"
        self.live_last_wall = time.perf_counter()
        self.live_timer.start()
        self._update_enabled()

    def _live_pause(self):
        if self.live_state == "running":
            self.live_state = "paused"
            self.live_timer.stop()

    def _live_tick(self):
        if self.live_busy or self.live_state != "running":
            return
        now = time.perf_counter()
        dt = now - self.live_last_wall
        self.live_last_wall = now
        dur = self.rows["duration"].config_value()
        target = min(dur, self.live_t + self.speed.config_value() * dt)
        if target <= self.live_t:
            if self.live_t >= dur:
                self._live_pause()
            return
        self.live_busy = True
        self._send({"cmd": "live_advance", "gen": self.gen, "t_target": target})
        self.live_t = target

    def _ingest_live(self, delta: dict):
        keys = ("true_t", "true_x", "est_t", "est_x", "delay_t", "delay_x", "rate_t", "rate_cmd", "rate_eff",
                "pi_t", "pi_p", "pi_i", "pi_out")
        for key, d in delta.items():
            b = self.live_buf.setdefault(key, {k: np.empty(0) for k in keys})
            for k in keys:
                if len(d[k]):
                    b[k] = np.concatenate([b[k], d[k]])
                    if b[k].size > MAX_LIVE_POINTS:
                        b[k] = b[k][-MAX_LIVE_POINTS:]
            b["t_now"] = d["t_now"]
            b["delay_nominal"] = d["delay_nominal"]
            b["limits"] = d["limits"]                 # current limits (they can change live)
            b.setdefault("changes", [])
            b.setdefault("events", [])
            b["changes"] += d["changes"]
            b["events"] += d["events"]
            b["counters"] = d["counters"]

    def _redraw_live(self, force: bool = False):
        now = time.perf_counter()
        if not force and now - self._last_draw < 0.05:
            return
        self._last_draw = now
        self.main_res = self._live_view("main")
        self.base_res = self._live_view("base") if "base" in self.live_buf else None
        self.metrics = self.base_metrics = None
        self._redraw()
        t_now = self.live_buf["main"]["t_now"]
        if self.chk_follow.isChecked():
            w = self.follow_win.config_value()
            self.p_delay.setXRange(max(0.0, t_now - w), max(w, t_now), padding=0.01)
        elif self._first_autorange and t_now > 0:
            self._autorange()
            self._first_autorange = False
        self.status.setText(T("st_live", t=t_now))
        self._fill_live_table()

    def _live_view(self, key: str) -> dict:
        b = self.live_buf[key]
        return {"t_end": b["t_now"], "true_t": b["true_t"], "true_x": b["true_x"], "est_t": b["est_t"],
                "est_x": b["est_x"], "delay_t": b["delay_t"], "delay_x": b["delay_x"], "delay_nominal": b["delay_nominal"],
                "rate_t": b["rate_t"], "rate_cmd": b["rate_cmd"], "rate_eff": b["rate_eff"],
                "pi_t": b["pi_t"], "pi_p": b["pi_p"], "pi_i": b["pi_i"], "pi_out": b["pi_out"],
                "limits": b["limits"], "events": b["events"], "changes": b["changes"]}

    # ------------------------------------------------------------------ drawing
    def _clear_plots(self):
        for c in self.cv.values():
            c.setData([], [])
        for p, ln in self.evt_lines + self.trans_lines:
            p.removeItem(ln)
        self.evt_lines.clear()
        self.trans_lines.clear()
        self._remove_limit_lines()
        self.table.setRowCount(0)
        self.table.setFixedHeight(self.table.horizontalHeader().height() + 2)

    def _scale(self) -> float:
        return UNITS[self.units.currentText()]

    def _redraw(self):
        r = self.main_res
        if r is None:
            return
        s = self._scale()
        u = self.units.currentText()
        self.p_off.setLabel("left", T("ax_off", u=u))
        self.p_delay.setLabel("left", T("ax_delay", u=u))
        self.cv["true"].setData(r["true_t"], r["true_x"] * s)
        show_est = self.chk_est.isChecked()
        if show_est:
            self.cv["est"].setData(r["est_t"], r["est_x"] * s)
        else:
            self.cv["est"].setData([], [])
        sx, sy = step_xy(r["delay_t"], r["delay_x"] * s, r["t_end"])
        self.cv["delay_step"].setData(sx, sy)
        self.cv["delay_pts"].setData(r["delay_t"], r["delay_x"] * s)
        self.cv["delay_ref"].setData([0.0, r["t_end"]], [r["delay_nominal"] * s] * 2)
        b = self.base_res if self.chk_overlay.isChecked() else None
        if b is not None:
            self.cv["base_true"].setData(b["true_t"], b["true_x"] * s)
            if show_est:
                self.cv["base_est"].setData(b["est_t"], b["est_x"] * s)
            else:
                self.cv["base_est"].setData([], [])
        else:
            self.cv["base_true"].setData([], [])
            self.cv["base_est"].setData([], [])
        if self.chk_diag.isChecked():
            self.cv["rate_cmd"].setData(*step_xy(r["rate_t"], r["rate_cmd"], r["t_end"]))
            self.cv["rate_eff"].setData(*step_xy(r["rate_t"], r["rate_eff"], r["t_end"]))
        self._draw_pi_terms(r)
        for p, ln in self.evt_lines:
            p.removeItem(ln)
        self.evt_lines.clear()
        for t_, text in r.get("changes", []):
            for p in (self.p_delay, self.p_off):
                ln = pg.InfiniteLine(pos=t_, angle=90, pen=pg.mkPen("#666666", style=QtCore.Qt.DashLine, width=1.5),
                                     label=(text.replace("{", "{{").replace("}", "}}") if p is self.p_off else ""),
                                     labelOpts={"position": 0.9, "color": "#444444"})
                p.addItem(ln)
                self.evt_lines.append((p, ln))
        marks = [(t_, k) for t_, k in r.get("events", []) if k in ("step", "servo_reset")]
        for t_, kind in (marks if len(marks) <= 30 else [m for m in marks if m[1] == "step"]):   # a reset every
            if kind in ("step", "servo_reset"):                                                    # sample is a hatch
                ln = pg.InfiniteLine(pos=t_, angle=90, pen=pg.mkPen(C_EVT, width=1, style=QtCore.Qt.DotLine))
                self.p_off.addItem(ln)
                self.evt_lines.append((self.p_off, ln))
        self._update_transient_lines()

    def _remove_limit_lines(self):
        for ln in self.lim_lines:
            self.p_pi.removeItem(ln)
        self.lim_lines.clear()

    def _draw_pi_terms(self, r):
        """PI-terms plot (ppm): P, I and controller output at each servo update (steps and rejected
        outliers have no update, so no point), the command actually applied (held, includes the clamp
        and the resets to nominal) and horizontal lines at the active limits.  Skipped while hidden."""
        self._remove_limit_lines()
        if not self.chk_pi.isChecked():
            return
        k = 1e-3                                         # ppb -> ppm
        self.cv["pi_p"].setData(r["pi_t"], r["pi_p"] * k)
        self.cv["pi_i"].setData(r["pi_t"], r["pi_i"] * k)
        self.cv["pi_out"].setData(r["pi_t"], r["pi_out"] * k)
        self.cv["pi_applied"].setData(*step_xy(r["rate_t"], r["rate_cmd"] * k, r["t_end"]))
        limits = r.get("limits", {})
        # each kind has its own label position along the line: close limits must not overprint
        for key, text_key, col, style, label_pos in (
                ("actuator", "lim_act", C_EVT, QtCore.Qt.DashDotLine, 0.04),
                ("clamp", "lim_clamp", C_EVT, QtCore.Qt.DashLine, 0.20),
                ("i_max", "lim_imax", C_TRUE, QtCore.Qt.DashLine, 0.36),
                ("sat", "lim_sat", C_REF, QtCore.Qt.DashLine, 0.52)):
            v = limits.get(key, 0.0)
            if v <= 0.0:
                continue
            for sign in (1.0, -1.0):
                ln = pg.InfiniteLine(pos=sign * v, angle=0, pen=pg.mkPen(col, width=1, style=style),
                                     label=T(text_key, v=v) if sign > 0 else None,
                                     labelOpts={"position": label_pos, "color": col})
                self.p_pi.addItem(ln, ignoreBounds=True)  # limits must not drive the y autorange
                self.lim_lines.append(ln)

    def _update_transient_lines(self):
        """Dashed vertical line at the end of the transient (settling time) on every plot."""
        for p, ln in self.trans_lines:
            p.removeItem(ln)
        self.trans_lines.clear()
        if not self.chk_trans.isChecked() or self.mode != "explore":
            return
        items = [(self.metrics, C_TRANS, "tr_line", 0.5)]
        if self.chk_overlay.isChecked():
            items.append((self.base_metrics, C_BASE, "tr_line_base", 0.2))
        plots = [self.p_delay, self.p_off] + ([self.p_diag] if self.chk_diag.isChecked() else []) \
            + ([self.p_pi] if self.chk_pi.isChecked() else [])
        for mm, col, key, pos in items:
            ts = None if mm is None else mm["true_offset"]["settling_s"]
            if ts is None:
                continue
            for p in plots:
                kw = {"label": T(key, t=ts), "labelOpts": {"position": pos, "color": col}} if p is self.p_off else {}
                ln = pg.InfiniteLine(pos=ts, angle=90, pen=pg.mkPen(col, width=1.5, style=QtCore.Qt.DashLine), **kw)
                p.addItem(ln)
                self.trans_lines.append((p, ln))

    def _autorange(self):
        r = self.main_res
        if r is None:
            return
        if self.mode == "live":
            for p in (self.p_delay, self.p_off, self.p_diag, self.p_pi):
                p.enableAutoRange(axis="x")
        else:
            self.view_combo.blockSignals(True)
            self.view_combo.setCurrentIndex(0)
            self.view_combo.blockSignals(False)
            self.p_delay.setXRange(0, max(1e-3, r["t_end"]), padding=0.01)
        for p in (self.p_off, self.p_delay, self.p_diag, self.p_pi):
            p.enableAutoRange(axis="y")

    def _apply_view(self):
        """Full run / transient only / steady state only (x range; y follows the visible data)."""
        r, m = self.main_res, self.metrics
        if r is None or self.mode != "explore":
            return
        idx = self.view_combo.currentIndex()
        t_end = r["t_end"]
        ts = None if m is None else m["true_offset"]["settling_s"]
        ss = None if m is None else m["true_offset"]["steady_start_s"]
        if idx == 0:
            lo, hi = 0.0, t_end
        elif idx == 1:
            # after a forced alignment (clock step) the huge pre-step offset is left out of the view
            stepped = m is not None and m["saturation"]["steps"] > 0
            lo = max(0.0, m["servo_start_s"] - 1.0) if stepped else 0.0
            hi = min(t_end, ts * 1.3 + 0.5) if ts is not None else t_end
        else:                                   # steady state: after the transient end, else the steady window
            lo = ts if ts is not None else (ss if ss is not None else 0.0)
            hi = t_end
        self.p_delay.setXRange(lo, max(hi, lo + 1e-3), padding=0.01)
        for p in (self.p_off, self.p_delay, self.p_diag, self.p_pi):
            p.enableAutoRange(axis="y")

    # ------------------------------------------------------------------ tables
    def _set_table_header(self):
        self.table.setHorizontalHeaderLabels([T("th_metric"), T("th_true"), T("th_est"), T("th_btrue"), T("th_best")])

    def _fit_table(self):
        h = self.table.horizontalHeader().height() + sum(self.table.rowHeight(i) for i in range(self.table.rowCount()))
        self.table.setFixedHeight(h + 4)

    def _fill_live_table(self):
        c = self.live_buf["main"].get("counters", {})
        rows = [("c_pairs", c.get("pairs")), ("c_delay", c.get("delay_samples")), ("c_steps", c.get("steps")),
                ("c_resets", c.get("resets")), ("c_range", c.get("range_resets")), ("c_outl", c.get("outliers")),
                ("c_lost", c.get("lost"))]
        self.table.clearSpans()
        self.table.setRowCount(len(rows))
        self.table.setHorizontalHeaderLabels([T("live_hdr"), T("live_val"), "", "", ""])
        for i, (k, v) in enumerate(rows):
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(T(k)))
            self.table.setItem(i, 1, QtWidgets.QTableWidgetItem(str(v)))
        self._fit_table()

    @staticmethod
    def _fmt_unit(v_ns):
        """Engineering format with the unit that suits the magnitude (the table ignores the plot unit)."""
        if v_ns is None or (isinstance(v_ns, float) and math.isnan(v_ns)):
            return "–"
        a = abs(v_ns)
        if a < 1e3:
            return f"{v_ns:.2f} ns"
        if a < 1e6:
            return f"{v_ns / 1e3:.3f} µs"
        if a < 1e9:
            return f"{v_ns / 1e6:.3f} ms"
        return f"{v_ns / 1e9:.3f} s"

    def _fill_table(self):
        m, bm = self.metrics, self.base_metrics
        if m is None:
            return
        self._set_table_header()
        f = self._fmt_unit

        def col(mm, kind):
            if mm is None:
                return None
            d = mm[kind]
            if d["settling_s"] is not None:
                tr = f"{d['settling_s']:.2f} s"
            else:
                tr = T("never", why=T("rs_" + d.get("settling_code", "outside_end")))
            ov = (f"{d['overshoot_pct']:.1f} % ({f(d['overshoot_ns'])})" if d["overshoot_pct"] is not None
                  else T("v_ov_undef"))
            return [tr, ov, f(d["peak_abs_ns"]), f(d["median_ns"]), f(d["min_ns"]), f(d["max_ns"]),
                    f(d["median_abs_ns"]), f(d["rms_ns"]), f(d["mean_ns"]), T("v_yes") if d["diverged"] else T("v_no")]

        keys = ["m_transient", "m_overshoot", "m_peak", "m_ss_median", "m_ss_min", "m_ss_max", "m_ss_medabs",
                "m_rms", "m_bias", "m_diverged"]
        cols = [col(m, "true_offset"), col(m, "estimated_offset"), col(bm, "true_offset"), col(bm, "estimated_offset")]
        d = m["delay"]
        s = m["saturation"]
        spans = [("m_delay", f"{f(d['median_ns'])} / {f(d['min_ns'])} / {f(d['max_ns'])}   (n = {d['n']}, "
                            f"{T('lg_delay_ref')}: {f(d['nominal_ns'])})"),
                 ("m_sat", T("sat_txt", a=s["controller_clamped"], b=s["range_resets"], c=s["resets"], d=s["steps"],
                             e=s["outliers_rejected"])),
                 ("m_time", T("time_txt", a=m["sim_wall_ms"], b=m["metrics_wall_ms"], n=m["n_events"])),
                 ("m_gui", T("gui_txt", a=self.last_ms["gui"] or 0.0, b=self.last_ms["compute"] or 0.0))]
        self.table.clearSpans()
        self.table.setRowCount(len(keys) + len(spans))
        for i, k in enumerate(keys):
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(T(k)))
            for j, c in enumerate(cols):
                self.table.setItem(i, j + 1, QtWidgets.QTableWidgetItem("" if c is None else c[i]))
        for i, (k, txt) in enumerate(spans, start=len(keys)):
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(T(k)))
            self.table.setItem(i, 1, QtWidgets.QTableWidgetItem(txt))
            self.table.setSpan(i, 1, 1, 4)
        for j in (3, 4):
            self.table.setColumnHidden(j, False)            # a hidden column is not sized to its contents
        self.table.resizeColumnsToContents()
        for j in (3, 4):
            self.table.setColumnHidden(j, bm is None)       # baseline columns only with the overlay
        self._fit_table()

    # ------------------------------------------------------------------ files
    def _save_cfg(self):
        p, _ = QtWidgets.QFileDialog.getSaveFileName(self, T("dlg_save"), "configs/scenario.json", "JSON (*.json)")
        if p:
            self.build_config().save(p)

    def _load_cfg(self):
        p, _ = QtWidgets.QFileDialog.getOpenFileName(self, T("dlg_load"), "configs", "JSON (*.json)")
        if p:
            self._apply_cfg_to_widgets(SimConfig.load(p))

    def _export(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, T("dlg_export"), "results")
        if d:
            self.status.setText(T("st_export"))
            self._send({"cmd": "export", "cfg": self.build_config().to_dict(), "dir": d, "mc": self._metrics_cfg()})


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(prog="ptpsim-gui")
    ap.add_argument("config", nargs="?", help="scenario JSON to start from")
    ap.add_argument("--lang", choices=sorted(LANGS), help="interface language (default: last used, else en)")
    args, qt_args = ap.parse_known_args(sys.argv[1:] if argv is None else argv)
    app = QtWidgets.QApplication([sys.argv[0]] + qt_args)
    cfg = SimConfig.load(args.config) if args.config else None
    w = MainWindow(cfg, lang=args.lang)
    if args.lang:
        QtCore.QSettings(*SETTINGS).setValue("lang", args.lang)
    w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
