# SPDX-License-Identifier: Apache-2.0
"""Interactive GUI (PySide6 + pyqtgraph).

Two clearly separated modes:

* **Esplorazione**: every parameter change recomputes the *whole* trajectory from the same initial
  conditions and the same seed (debounced, computed in a worker *process*, stale results discarded).
* **Live**: the trajectory continues; parameter changes apply from the current instant (marked on the
  plots).  Playback speed only changes how much simulated time is requested per wall-clock tick; the
  mathematics of the simulation does not depend on it.

All simulation work happens in ``ptpsim.gui.worker``; this module only builds widgets and draws.
"""
from __future__ import annotations

import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from ..config import JitterSpec, SimConfig, noisy_preset, set_path
from ..controllers import POLICIES, REGISTRY
from ..engine import interval_ps
from .params import ParamRow
from .worker import worker_main

UNITS = {"ns": 1.0, "µs": 1e-3, "ms": 1e-6}
# Okabe-Ito colour-blind safe palette
C_TRUE, C_EST, C_BASE, C_DELAY, C_REF, C_EVT = "#0072B2", "#E69F00", "#CC79A7", "#009E73", "#000000", "#D55E00"
MAX_LIVE_POINTS = 200_000
CLOCK_ROOTS = [("100 MHz (SYS_PLL1_DIV2/5, default)", 100_000_000), ("98.304 MHz (AUDIO_PLL/4)", 98_304_000),
               ("196.608 MHz (AUDIO_PLL/2)", 196_608_000), ("24 MHz (OSC_24M)", 24_000_000),
               ("25 MHz", 25_000_000)]


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
    DEBOUNCE_MS = 80
    LIVE_TICK_MS = 40

    def __init__(self, cfg: SimConfig | None = None):
        super().__init__()
        self.setWindowTitle("PTP_sim — simulatore PTPv2 closed-loop (Zephyr time receiver)")
        self.resize(1500, 950)
        self.base_cfg = (cfg or SimConfig()).copy()
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

        self._start_worker()
        self._build_ui()
        self.debounce = QtCore.QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(self.DEBOUNCE_MS)
        self.debounce.timeout.connect(self._debounced)
        self.live_timer = QtCore.QTimer(self)
        self.live_timer.setInterval(self.LIVE_TICK_MS)
        self.live_timer.timeout.connect(self._live_tick)
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

    def closeEvent(self, ev):
        try:
            self.live_timer.stop()
            self.latest.value = -1
            self.req_q.put({"cmd": "quit"})
            self.reader.stop()
            self.reader.wait(1000)
            self.proc.join(1.0)
            if self.proc.is_alive():
                self.proc.terminate()
        finally:
            super().closeEvent(ev)

    # ------------------------------------------------------------------ UI construction
    def _row(self, name, grid, r, *args, **kw) -> ParamRow:
        row = ParamRow(*args, **kw)
        row.add_to(grid, r)
        row.changed.connect(self._on_param_changed)
        self.rows[name] = row
        return row

    def _group(self, title) -> tuple[QtWidgets.QGroupBox, QtWidgets.QGridLayout]:
        g = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(g)
        grid.setColumnStretch(2, 1)
        return g, grid

    def _build_ui(self):
        c = self.base_cfg
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(split)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(430)
        panel = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(panel)
        scroll.setWidget(panel)
        split.addWidget(scroll)

        # ---- mode and run control
        g, grid = self._group("Modalità e controllo")
        self.mode_explore = QtWidgets.QRadioButton("Esplorazione (ricalcola tutta la traiettoria, stesso seed)")
        self.mode_live = QtWidgets.QRadioButton("Simulazione live (i parametri valgono da ora in poi)")
        self.mode_explore.setChecked(True)
        self.mode_explore.toggled.connect(self._on_mode_toggled)
        grid.addWidget(self.mode_explore, 0, 0, 1, 3)
        grid.addWidget(self.mode_live, 1, 0, 1, 3)
        btns = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("▶ Start")
        self.btn_pause = QtWidgets.QPushButton("⏸ Pausa")
        self.btn_reset = QtWidgets.QPushButton("⟲ Reset")
        for b, f in ((self.btn_start, self._live_start), (self.btn_pause, self._live_pause), (self.btn_reset, self._live_reset)):
            b.clicked.connect(f)
            btns.addWidget(b)
        grid.addLayout(btns, 2, 0, 1, 3)
        self.speed = self._row("speed", grid, 3, "Velocità riproduzione", [], 0.1, 500.0, 10.0, 0.5, decimals=1,
                               suffix="×", log=True, tooltip="Solo live: tempo simulato per secondo reale. Non modifica la matematica.")
        self.speed.changed.disconnect(self._on_param_changed)
        self.live_policy = QtWidgets.QComboBox()
        self.live_policy.addItems(["keep (mantieni integratore)", "reset (azzera integratore)",
                                   "bumpless (uscita continua)"])
        grid.addWidget(QtWidgets.QLabel("Politica integratore al cambio guadagni"), 4, 0, 1, 2)
        grid.addWidget(self.live_policy, 4, 2)
        self.chk_follow = QtWidgets.QCheckBox("Segui (finestra scorrevole)")
        self.chk_follow.setChecked(True)
        self.follow_win = self._row("follow", grid, 5, "Finestra live", [], 5.0, 3600.0, 60.0, 5.0, decimals=0,
                                    suffix="s", slider=False)
        self.follow_win.changed.disconnect(self._on_param_changed)
        grid.addWidget(self.chk_follow, 6, 0, 1, 3)
        pl.addWidget(g)

        # ---- controller
        g, grid = self._group("Controllore")
        self.ctrl_combo = QtWidgets.QComboBox()
        self.ctrl_combo.addItems(list(REGISTRY))
        self.ctrl_combo.setCurrentText(c.controller.name)
        grid.addWidget(QtWidgets.QLabel("Controllore"), 0, 0)
        grid.addWidget(self.ctrl_combo, 0, 1, 1, 2)
        self.ctrl_grid = grid
        self.ctrl_combo.currentTextChanged.connect(self._on_controller_changed)
        self.ctrl_note = QtWidgets.QLabel()
        self.ctrl_note.setWordWrap(True)
        self.ctrl_note.setStyleSheet("color: gray")
        grid.addWidget(self.ctrl_note, 99, 0, 1, 3)
        pl.addWidget(g)
        self._build_ctrl_rows(c.controller.name, c.controller.params)

        # ---- intervals
        g, grid = self._group("Intervalli PTP (1 s × 2ⁿ)")
        self._row("sync_log", grid, 0, "Sync: esponente n", ["intervals.sync_log"], -4, 2, c.intervals.sync_log, 1,
                  integer=True, tooltip="logMessageInterval di Sync/Follow_Up (default -2 = 0.25 s)")
        self.sync_txt = QtWidgets.QLabel()
        grid.addWidget(self.sync_txt, 1, 0, 1, 3)
        self.delay_mode = QtWidgets.QComboBox()
        self.delay_mode.addItems(["Intervallo indipendente (come il firmware)", "Ogni N Sync (NON nel firmware)"])
        self.delay_mode.setCurrentIndex(0 if c.intervals.delay_mode == "interval" else 1)
        self.delay_mode.currentIndexChanged.connect(lambda *_: self._on_param_changed(None, 0.0, "intervals.delay_mode"))
        grid.addWidget(QtWidgets.QLabel("Delay_Req"), 2, 0)
        grid.addWidget(self.delay_mode, 2, 1, 1, 2)
        self._row("delay_log", grid, 3, "Delay_Req: esponente n", ["intervals.delay_log"], -4, 2, c.intervals.delay_log, 1,
                  integer=True, tooltip="Intervallo indipendente: timer del client (k_timer, non disciplinato)")
        self.delay_txt = QtWidgets.QLabel()
        grid.addWidget(self.delay_txt, 4, 0, 1, 3)
        self._row("delay_n", grid, 5, "Ogni N Sync: N", ["intervals.delay_every_n"], 1, 64, c.intervals.delay_every_n, 1,
                  integer=True)
        self.chk_rearm = QtWidgets.QCheckBox("Timer Delay_Req riarmato alla gestione (deriva cumulativa, firmware)")
        self.chk_rearm.setChecked(c.intervals.delay_rearm_from_handling)
        self.chk_rearm.toggled.connect(lambda *_: self._on_param_changed(None, 0.0, "intervals.delay_rearm_from_handling"))
        grid.addWidget(self.chk_rearm, 6, 0, 1, 3)
        pl.addWidget(g)

        # ---- initial conditions
        g, grid = self._group("Condizioni iniziali / durata")
        self._row("offset0", grid, 0, "Offset iniziale", ["oscillator.initial_offset_ns"], -2000.0, 2000.0,
                  c.oscillator.initial_offset_ns / 1e3, 1.0, decimals=1, suffix="µs", scale=1e3)
        self._row("freq0", grid, 1, "Errore di frequenza", ["oscillator.freq_error_ppb"], -200.0, 200.0,
                  c.oscillator.freq_error_ppb / 1e3, 0.1, decimals=2, suffix="ppm", scale=1e3)
        self._row("duration", grid, 2, "Durata", ["duration_s"], 10.0, 3600.0, c.duration_s, 10.0, decimals=0,
                  suffix="s")
        self._row("seed", grid, 3, "Seed", ["seed"], 0, 99999, c.seed, 1, integer=True, slider=False)
        self._row("drift", grid, 4, "Deriva oscillatore", ["oscillator.drift_ppb_per_s"], -50.0, 50.0,
                  c.oscillator.drift_ppb_per_s, 0.1, decimals=2, suffix="ppb/s")
        self._row("timer_err", grid, 5, "Errore timer k_timer", ["oscillator.timer_error_ppb"], -100.0, 100.0,
                  c.oscillator.timer_error_ppb / 1e3, 0.5, decimals=1, suffix="ppm", scale=1e3,
                  tooltip="Il timer dei Delay_Req è un clock monotono indipendente dal PHC disciplinato")
        pl.addWidget(g)

        # ---- network
        g, grid = self._group("Rete e disturbi")
        n = c.network
        self._row("d_mean", grid, 0, "Ritardo medio", ["network.delay_ms_ns"], 0.0, 100_000.0,
                  (n.delay_ms_ns + n.delay_sm_ns) / 2, 10.0, decimals=0, suffix="ns")
        self._row("d_asym", grid, 1, "Asimmetria (GM→S − S→GM)", ["network.delay_asymmetry_ns"], -20_000.0, 20_000.0,
                  n.delay_ms_ns - n.delay_sm_ns, 10.0, decimals=0, suffix="ns",
                  tooltip="Bias dell'offset stimato = asimmetria/2")
        self._row("net_jit", grid, 2, "Jitter di rete (esp., ogni direzione)", ["network.jitter_ms.scale_ns", "network.jitter_sm.scale_ns"],
                  0.0, 5000.0, n.jitter_ms.scale_ns, 10.0, decimals=0, suffix="ns")
        self._row("tx_jit", grid, 3, "Jitter di invio (σ, tutti i messaggi)", [
            "tx_jitter.sync.scale_ns", "tx_jitter.follow_up.scale_ns", "tx_jitter.delay_req.scale_ns",
            "tx_jitter.delay_resp.scale_ns"], 0.0, 200.0, c.tx_jitter.sync.scale_ns / 1e3, 1.0, decimals=1,
                  suffix="µs", scale=1e3, tooltip="Attorno all'istante nominale (non cumulativo)")
        self._row("lat_fup", grid, 4, "Latenza Follow_Up", ["latency.follow_up_ns"], 0.0, 50_000.0,
                  c.latency.follow_up_ns / 1e3, 10.0, decimals=0, suffix="µs", scale=1e3)
        self._row("lat_dresp", grid, 5, "Latenza Delay_Resp", ["latency.delay_resp_ns"], 0.0, 50_000.0,
                  c.latency.delay_resp_ns / 1e3, 10.0, decimals=0, suffix="µs", scale=1e3)
        self._row("lat_cmd", grid, 6, "Latenza applicazione comando", ["latency.command_ns"], 0.0, 100_000.0,
                  c.latency.command_ns / 1e3, 10.0, decimals=0, suffix="µs", scale=1e3)
        self._row("ts_noise", grid, 7, "Rumore timestamp (σ, GM e slave)", ["timestamps.gm_noise_sigma_ns",
                  "timestamps.slave_noise_sigma_ns"], 0.0, 100.0, c.timestamps.gm_noise_sigma_ns, 0.5, decimals=1, suffix="ns")
        self._row("gm_q", grid, 8, "Quantizzazione timestamp GM", ["timestamps.gm_quantum_ns"], 0.0, 100.0,
                  c.timestamps.gm_quantum_ns, 1.0, decimals=0, suffix="ns")
        self._row("loss", grid, 9, "Perdita messaggi (ogni tipo)", ["loss.sync", "loss.follow_up", "loss.delay_req",
                  "loss.delay_resp"], 0.0, 50.0, c.loss.sync * 100, 0.5, decimals=1, suffix="%", scale=0.01)
        self.btn_noisy = QtWidgets.QPushButton("Preset rumoroso (jitter 20 µs, rete 200 ns, ts 3 ns)")
        self.btn_noisy.clicked.connect(self._apply_noisy_preset)
        grid.addWidget(self.btn_noisy, 10, 0, 1, 3)
        pl.addWidget(g)

        # ---- actuator
        g, grid = self._group("Attuatore")
        self.act_combo = QtWidgets.QComboBox()
        self.act_combo.addItems(["ideale", "NXP ENET (PR #121108)"])
        self.act_combo.setCurrentIndex(0 if c.actuator.kind == "ideal" else 1)
        self.act_combo.currentIndexChanged.connect(lambda *_: self._on_param_changed(None, 0.0, "actuator.kind"))
        grid.addWidget(QtWidgets.QLabel("Tipo"), 0, 0)
        grid.addWidget(self.act_combo, 0, 1, 1, 2)
        self.root_combo = QtWidgets.QComboBox()
        for name, hz in CLOCK_ROOTS:
            self.root_combo.addItem(name, hz)
        self.root_combo.currentIndexChanged.connect(lambda *_: self._on_param_changed(None, 0.0, "actuator.clock_hz"))
        grid.addWidget(QtWidgets.QLabel("Clock root timer 1588"), 1, 0)
        grid.addWidget(self.root_combo, 1, 1, 1, 2)
        self.act_info = QtWidgets.QLabel("")
        self.act_info.setWordWrap(True)
        grid.addWidget(self.act_info, 2, 0, 1, 3)
        pl.addWidget(g)

        # ---- metrics settings
        g, grid = self._group("Metriche")
        self.m_band = self._row("m_band", grid, 0, "Banda di assestamento (±)", [], 1.0, 100_000.0, 1000.0, 10.0, decimals=0,
                                suffix="ns", log=True)
        self.m_dwell = self._row("m_dwell", grid, 1, "Permanenza", [], 0.0, 300.0, 10.0, 1.0, decimals=1, suffix="s")
        self.m_win = self._row("m_win", grid, 2, "Finestra finale RMS/bias", [], 1.0, 3600.0, 60.0, 5.0, decimals=0, suffix="s")
        for r in (self.m_band, self.m_dwell, self.m_win):
            r.changed.disconnect(self._on_param_changed)
            r.changed.connect(lambda *_: self._on_param_changed(None, 0.0, "__metrics__"))
        pl.addWidget(g)

        # ---- files
        g, grid = self._group("File")
        b1 = QtWidgets.QPushButton("Salva config/seed…")
        b2 = QtWidgets.QPushButton("Carica config…")
        b3 = QtWidgets.QPushButton("Esporta CSV risultati…")
        b1.clicked.connect(self._save_cfg)
        b2.clicked.connect(self._load_cfg)
        b3.clicked.connect(self._export)
        grid.addWidget(b1, 0, 0)
        grid.addWidget(b2, 0, 1)
        grid.addWidget(b3, 1, 0, 1, 2)
        pl.addWidget(g)
        pl.addStretch(1)

        # ---- right side: toolbar, plots, metrics
        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        tb = QtWidgets.QHBoxLayout()
        tb.addWidget(QtWidgets.QLabel("Unità:"))
        self.units = QtWidgets.QComboBox()
        self.units.addItems(list(UNITS))
        self.units.setCurrentText("µs")
        self.units.currentTextChanged.connect(lambda *_: self._redraw())
        tb.addWidget(self.units)
        self.chk_overlay = QtWidgets.QCheckBox("Sovrapponi baseline (PI firmware)")
        self.chk_overlay.toggled.connect(self._on_overlay_toggled)
        tb.addWidget(self.chk_overlay)
        self.chk_est = QtWidgets.QCheckBox("Offset stimato")
        self.chk_est.setChecked(True)
        self.chk_est.toggled.connect(lambda *_: self._redraw())
        tb.addWidget(self.chk_est)
        self.chk_diag = QtWidgets.QCheckBox("Diagnostica rate")
        self.chk_diag.toggled.connect(self._on_diag_toggled)
        tb.addWidget(self.chk_diag)
        btn_fit = QtWidgets.QPushButton("Adatta vista")
        btn_fit.clicked.connect(self._autorange)
        tb.addWidget(btn_fit)
        tb.addStretch(1)
        self.status = QtWidgets.QLabel("avvio worker…")
        tb.addWidget(self.status)
        rl.addLayout(tb)

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        self.glw = pg.GraphicsLayoutWidget()
        rl.addWidget(self.glw, 1)
        self.p_delay = self.glw.addPlot(row=0, col=0)
        self.p_off = self.glw.addPlot(row=1, col=0)
        self.p_diag = self.glw.addPlot(row=2, col=0)
        self.p_off.setXLink(self.p_delay)
        self.p_diag.setXLink(self.p_delay)
        self.glw.ci.layout.setRowStretchFactor(0, 3)
        self.glw.ci.layout.setRowStretchFactor(1, 4)
        self.glw.ci.layout.setRowStretchFactor(2, 2)
        for p in (self.p_delay, self.p_off, self.p_diag):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setMenuEnabled(True)
            for ax in ("left", "bottom"):
                p.getAxis(ax).enableAutoSIPrefix(False)     # explicit units only, no "x0.001" scaling
        self.p_diag.setVisible(False)
        self.p_delay.setTitle("Delay medio (stima del firmware vs riferimento fisico)")
        self.p_off.setTitle("Offset slave − GM")
        self.p_diag.setTitle("Rate: comandato dal servo e effettivo del clock")
        self.p_off.setLabel("bottom", "tempo fisico (GM) [s]")
        self.p_diag.setLabel("left", "ppb")
        for p in (self.p_delay, self.p_off):
            p.addLegend(offset=(-10, 10))
        self.p_diag.addLegend(offset=(-10, 10))
        pen = lambda col, w=1.5, st=None: pg.mkPen(col, width=w, style=st or QtCore.Qt.SolidLine)
        self.cv = {
            "delay_step": self.p_delay.plot(pen=pen(C_DELAY, 1.5), name="delay stimato (tenuto fino al campione successivo)"),
            "delay_pts": self.p_delay.plot(pen=None, symbol="o", symbolSize=5, symbolBrush=C_DELAY, symbolPen=None,
                                           name="campioni (Delay_Resp elaborate)"),
            "delay_ref": self.p_delay.plot(pen=pen(C_REF, 1.5, QtCore.Qt.DashLine), name="delay fisico (rete)"),
            "true": self.p_off.plot(pen=pen(C_TRUE, 2.0), name="offset reale"),
            "est": self.p_off.plot(pen=pen(C_EST, 1.0), symbol="o", symbolSize=4, symbolBrush=C_EST, symbolPen=None,
                                   name="offset stimato (campioni)"),
            "base_true": self.p_off.plot(pen=pen(C_BASE, 1.5, QtCore.Qt.DashLine), name="baseline: offset reale"),
            "base_est": self.p_off.plot(pen=pen(C_BASE, 1.0, QtCore.Qt.DotLine), name="baseline: offset stimato"),
            "rate_cmd": self.p_diag.plot(pen=pen(C_EST, 1.5), name="comandato (ppb)"),
            "rate_eff": self.p_diag.plot(pen=pen(C_TRUE, 1.5), name="rate effettivo clock vs GM (ppb)"),
        }
        for k in ("true", "base_true"):
            self.cv[k].setDownsampling(auto=True, method="peak")
            self.cv[k].setClipToView(True)
        self.evt_lines: list = []

        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Metrica", "Offset reale", "Offset stimato", "Baseline reale", "Baseline stimato"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMaximumHeight(250)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        rl.addWidget(self.table)
        split.addWidget(right)
        split.setStretchFactor(1, 1)
        self._update_interval_labels()
        self._update_enabled()

    # ------------------------------------------------------------------ controller rows
    def _build_ctrl_rows(self, name: str, params: dict[str, float]):
        for r in self.ctrl_rows.values():
            for w in (r.label, r.spin, r.slider):
                if w is not None:
                    self.ctrl_grid.removeWidget(w)
                    w.deleteLater()
            self.rows.pop("ctrl." + r.paths[0].split(".")[-1], None)
        self.ctrl_rows.clear()
        cls = REGISTRY[name]
        for i, (k, (dflt, lo, hi, desc)) in enumerate(cls.PARAMS.items(), start=1):
            val = params.get(k, dflt)
            row = ParamRow(k, [f"controller.params.{k}"], lo, hi, val, (hi - lo) / 200.0 if hi < 100 else 1.0,
                           decimals=3 if hi <= 20 else 1, tooltip=desc)
            row.add_to(self.ctrl_grid, i)
            row.changed.connect(self._on_param_changed)
            self.rows["ctrl." + k] = row
            self.ctrl_rows[k] = row
        self.ctrl_note.setText((cls.__doc__ or "").strip().split("\n\n")[0].replace("\n", " "))

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
        return out

    def _all_overrides(self) -> dict:
        ov: dict = {}
        for row in self.rows.values():
            if row.paths:
                ov.update(self._row_overrides(row))
        ov["intervals.delay_mode"] = "interval" if self.delay_mode.currentIndex() == 0 else "every_n_sync"
        ov["intervals.delay_rearm_from_handling"] = self.chk_rearm.isChecked()
        ov["actuator.kind"] = "ideal" if self.act_combo.currentIndex() == 0 else "nxp"
        ov["actuator.clock_hz"] = int(self.root_combo.currentData())
        ov["controller.name"] = self.ctrl_combo.currentText()
        # jitter kinds follow the scale
        for p in ("sync", "follow_up", "delay_req", "delay_resp"):
            ov[f"tx_jitter.{p}.kind"] = "normal" if ov[f"tx_jitter.{p}.scale_ns"] > 0 else "none"
        for p in ("jitter_ms", "jitter_sm"):
            ov[f"network.{p}.kind"] = "exponential" if ov[f"network.{p}.scale_ns"] > 0 else "none"
        return ov

    def build_config(self) -> SimConfig:
        cfg = self.base_cfg.copy()
        ov = self._all_overrides()
        ctrl_params = {k.split(".")[-1]: v for k, v in ov.items() if k.startswith("controller.params.")}
        for k, v in ov.items():
            if k.startswith("controller.params."):
                continue
            set_path(cfg, k, v)
        cfg.controller.params = ctrl_params
        return cfg

    def _metrics_cfg(self) -> dict:
        return {"band_ns": self.m_band.config_value(), "dwell_s": self.m_dwell.config_value(),
                "final_window_s": self.m_win.config_value()}

    def _apply_cfg_to_widgets(self, c: SimConfig):
        self.base_cfg = c.copy()
        self.ctrl_combo.blockSignals(True)
        self.ctrl_combo.setCurrentText(c.controller.name)
        self.ctrl_combo.blockSignals(False)
        self._build_ctrl_rows(c.controller.name, c.controller.params)
        vals = {"sync_log": c.intervals.sync_log, "delay_log": c.intervals.delay_log, "delay_n": c.intervals.delay_every_n,
                "offset0": c.oscillator.initial_offset_ns, "freq0": c.oscillator.freq_error_ppb,
                "duration": c.duration_s, "seed": c.seed, "drift": c.oscillator.drift_ppb_per_s,
                "timer_err": c.oscillator.timer_error_ppb,
                "d_mean": (c.network.delay_ms_ns + c.network.delay_sm_ns) / 2,
                "d_asym": c.network.delay_ms_ns - c.network.delay_sm_ns + c.network.delay_asymmetry_ns * 0,
                "net_jit": c.network.jitter_ms.scale_ns, "tx_jit": c.tx_jitter.sync.scale_ns,
                "lat_fup": c.latency.follow_up_ns, "lat_dresp": c.latency.delay_resp_ns, "lat_cmd": c.latency.command_ns,
                "ts_noise": c.timestamps.gm_noise_sigma_ns, "gm_q": c.timestamps.gm_quantum_ns, "loss": c.loss.sync}
        for k, v in vals.items():
            self.rows[k].set_config_value(v)
        self.delay_mode.blockSignals(True)
        self.delay_mode.setCurrentIndex(0 if c.intervals.delay_mode == "interval" else 1)
        self.delay_mode.blockSignals(False)
        self.chk_rearm.setChecked(c.intervals.delay_rearm_from_handling)
        self.act_combo.blockSignals(True)
        self.act_combo.setCurrentIndex(0 if c.actuator.kind == "ideal" else 1)
        self.act_combo.blockSignals(False)
        i = self.root_combo.findData(c.actuator.clock_hz)
        if i >= 0:
            self.root_combo.setCurrentIndex(i)
        self._first_autorange = True
        self._on_param_changed(None, 0.0, "__all__")

    def _apply_noisy_preset(self):
        c = noisy_preset()
        cur = self.build_config()
        c.duration_s, c.seed = cur.duration_s, cur.seed
        c.controller, c.actuator, c.intervals = cur.controller, cur.actuator, cur.intervals
        self.rows["net_jit"].set_config_value(c.network.jitter_ms.scale_ns)
        self.rows["tx_jit"].set_config_value(c.tx_jitter.sync.scale_ns)
        self.rows["ts_noise"].set_config_value(c.timestamps.gm_noise_sigma_ns)
        self._on_param_changed(None, 0.0, "__all__")

    # ------------------------------------------------------------------ change handling
    def _update_interval_labels(self):
        self.sync_txt.setText("Intervallo Sync: " + interval_text(int(self.rows["sync_log"].config_value())))
        t = "Intervallo Delay_Req: " + interval_text(int(self.rows["delay_log"].config_value()))
        if self.delay_mode.currentIndex() == 1:
            n = int(self.rows["delay_n"].config_value())
            t += f"   (attivo: ogni {n} Sync = {n * interval_ps(int(self.rows['sync_log'].config_value())) / 1e12:g} s)"
        self.delay_txt.setText(t)

    def _update_enabled(self):
        every_n = self.delay_mode.currentIndex() == 1
        self.rows["delay_log"].set_enabled(not every_n)
        self.rows["delay_n"].set_enabled(every_n)
        self.chk_rearm.setEnabled(not every_n)
        nxp = self.act_combo.currentIndex() == 1
        self.root_combo.setEnabled(nxp)
        live = self.mode == "live"
        for k in ("offset0", "freq0", "seed", "duration"):
            self.rows[k].set_enabled(not live or self.live_state == "stopped")
        for b in (self.btn_start, self.btn_pause, self.btn_reset):
            b.setEnabled(live)
        self.speed.set_enabled(live)
        self.live_policy.setEnabled(live)
        self.follow_win.set_enabled(live)
        self.chk_follow.setEnabled(live)

    def _on_param_changed(self, row, value, tag: str | None = None):
        self._update_interval_labels()
        self._update_enabled()
        self.t_change = time.perf_counter()
        if tag == "__metrics__":
            if self.mode == "explore":
                self.debounce.start()
            return
        if self.mode == "live":
            if self.live_state == "stopped":
                return
            ov = self._all_overrides()
            if row is not None:
                changed = self._row_overrides(row)
            else:
                changed = {k: ov[k] for k in ([tag] if tag and not tag.startswith("__") else [])}
            if tag == "controller.name":
                changed = {k: v for k, v in ov.items() if k.startswith("controller.")}
            if tag == "actuator.clock_hz" or tag == "actuator.kind":
                changed = {"actuator.kind": ov["actuator.kind"], "actuator.clock_hz": ov["actuator.clock_hz"]}
            if row is not None:
                for p in row.paths:
                    if p.startswith("tx_jitter") or p.startswith("network.jitter"):
                        base = p.rsplit(".", 1)[0]
                        changed[base + ".kind"] = ov[base + ".kind"]
            self._pending_live.update(changed)
            self.debounce.start()
        else:
            self.debounce.start()

    def _debounced(self):
        if self.mode == "explore":
            self._request_explore()
        elif self._pending_live and self.live_state != "stopped":
            ov, self._pending_live = self._pending_live, {}
            policy = POLICIES[self.live_policy.currentIndex()]
            self.req_q.put({"cmd": "live_update", "gen": self.gen, "overrides": ov, "policy": policy})

    # ------------------------------------------------------------------ explore
    def _request_explore(self):
        self.gen += 1
        self.latest.value = self.gen
        cfg = self.build_config()
        self.status.setText("calcolo in corso…")
        self.req_q.put({"cmd": "explore", "gen": self.gen, "cfg": cfg.to_dict(),
                        "overlay": self.chk_overlay.isChecked(), "mc": self._metrics_cfg()})

    def _on_overlay_toggled(self, on):
        if self.mode == "explore":
            self._request_explore()
        else:
            self._live_reset()

    def _on_diag_toggled(self, on):
        self.p_diag.setVisible(on)
        self._redraw()

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
            self.status.setText("ERRORE nel worker (vedi console)")
            print(msg["msg"], file=sys.stderr)
            self.live_busy = False
            return
        if t == "exported":
            self.status.setText(f"esportato in {msg['path']}")
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
            lat = (time.perf_counter() - self.t_change) * 1e3 if self.t_change else float("nan")
            self.last_ms["gui"] = lat
            self.n_results += 1
            self.status.setText(f"aggiornato in {lat:.0f} ms (calcolo {msg['compute_ms']:.0f} ms, sim {msg['metrics']['sim_wall_ms']:.0f} ms)")
            self._fill_table()
        elif t == "live_reset":
            self.live_resetting = False
            self.live_buf = {}
            self.live_t = 0.0
            self._ingest_live(msg["delta"])
            self._redraw_live(force=True)
            self.status.setText("live pronto (t = 0)")
        elif t == "live":
            self.live_busy = False
            self._ingest_live(msg["delta"])
            self._redraw_live()
        elif t == "live_updated":
            pass

    @staticmethod
    def _act_text(a):
        if not a:
            return ""
        if a["kind"] == "ideal":
            return f"attuatore ideale — rate ratio {a['ratio']:.12f}"
        return f"NXP: INC={a['inc']}  INC_CORR={a['inc_corr']}  ATCOR={a['cor']}  (ratio effettivo {a['ratio']:.9f})"

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
        self.req_q.put({"cmd": "live_reset", "gen": self.gen, "cfg": self.build_config().to_dict(),
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
        self.req_q.put({"cmd": "live_advance", "gen": self.gen, "t_target": target})
        self.live_t = target

    def _ingest_live(self, delta: dict):
        for key, d in delta.items():
            keys = ("true_t", "true_x", "est_t", "est_x", "delay_t", "delay_x", "rate_t", "rate_cmd", "rate_eff")
            b = self.live_buf.setdefault(key, {k: np.empty(0) for k in keys})
            for k in keys:
                if len(d[k]):
                    b[k] = np.concatenate([b[k], d[k]])
                    if b[k].size > MAX_LIVE_POINTS:
                        b[k] = b[k][-MAX_LIVE_POINTS:]
            b["t_now"] = d["t_now"]
            b["delay_nominal"] = d["delay_nominal"]
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
        self._redraw()
        t_now = self.live_buf["main"]["t_now"]
        if self.chk_follow.isChecked():
            w = self.follow_win.config_value()
            self.p_delay.setXRange(max(0.0, t_now - w), max(w, t_now), padding=0.01)
        elif self._first_autorange and t_now > 0:
            self._autorange()
            self._first_autorange = False
        self.status.setText(f"live  t = {t_now:.2f} s")
        self._fill_live_table()

    def _live_view(self, key: str) -> dict:
        b = self.live_buf[key]
        return {"t_end": b["t_now"], "true_t": b["true_t"], "true_x": b["true_x"], "est_t": b["est_t"],
                "est_x": b["est_x"], "delay_t": b["delay_t"], "delay_x": b["delay_x"], "delay_nominal": b["delay_nominal"],
                "rate_t": b["rate_t"], "rate_cmd": b["rate_cmd"], "rate_eff": b["rate_eff"],
                "events": b["events"], "changes": b["changes"]}

    # ------------------------------------------------------------------ drawing
    def _clear_plots(self):
        for c in self.cv.values():
            c.setData([], [])
        for ln in self.evt_lines:
            ln[0].removeItem(ln[1])
        self.evt_lines.clear()
        self.table.setRowCount(0)

    def _scale(self) -> float:
        return UNITS[self.units.currentText()]

    def _redraw(self):
        r = self.main_res
        if r is None:
            return
        s = self._scale()
        u = self.units.currentText()
        self.p_off.setLabel("left", f"offset [{u}]")
        self.p_delay.setLabel("left", f"delay [{u}]")
        self.cv["true"].setData(r["true_t"], r["true_x"] * s)
        show_est = self.chk_est.isChecked()
        self.cv["est"].setData(r["est_t"], r["est_x"] * s) if show_est else self.cv["est"].setData([], [])
        sx, sy = step_xy(r["delay_t"], r["delay_x"] * s, r["t_end"])
        self.cv["delay_step"].setData(sx, sy)
        self.cv["delay_pts"].setData(r["delay_t"], r["delay_x"] * s)
        self.cv["delay_ref"].setData([0.0, r["t_end"]], [r["delay_nominal"] * s] * 2)
        b = self.base_res if self.chk_overlay.isChecked() else None
        if b is not None:
            self.cv["base_true"].setData(b["true_t"], b["true_x"] * s)
            self.cv["base_est"].setData(b["est_t"], b["est_x"] * s) if show_est else self.cv["base_est"].setData([], [])
        else:
            self.cv["base_true"].setData([], [])
            self.cv["base_est"].setData([], [])
        if self.chk_diag.isChecked():
            self.cv["rate_cmd"].setData(*step_xy(r["rate_t"], r["rate_cmd"], r["t_end"]))
            self.cv["rate_eff"].setData(*step_xy(r["rate_t"], r["rate_eff"], r["t_end"]))
        # markers: parameter changes (live) and servo events
        for p, ln in self.evt_lines:
            p.removeItem(ln)
        self.evt_lines.clear()
        for t_, text in r.get("changes", []):
            for p in (self.p_delay, self.p_off):
                ln = pg.InfiniteLine(pos=t_, angle=90, pen=pg.mkPen("#666666", style=QtCore.Qt.DashLine, width=1.5),
                                     label=(text.replace("{", "{{").replace("}", "}}") if p is self.p_off else ""), labelOpts={"position": 0.9, "color": "#444444"})
                p.addItem(ln)
                self.evt_lines.append((p, ln))
        for t_, kind in r.get("events", [])[-200:]:
            if kind in ("step", "servo_reset"):
                ln = pg.InfiniteLine(pos=t_, angle=90, pen=pg.mkPen(C_EVT, width=1, style=QtCore.Qt.DotLine))
                self.p_off.addItem(ln)
                self.evt_lines.append((self.p_off, ln))

    def _autorange(self):
        r = self.main_res
        if r is None:
            return
        if self.mode == "live":
            for p in (self.p_delay, self.p_off, self.p_diag):
                p.enableAutoRange(axis="x")
        else:
            self.p_delay.setXRange(0, max(1e-3, r["t_end"]), padding=0.01)
        self.p_off.enableAutoRange(axis="y")
        self.p_delay.enableAutoRange(axis="y")
        self.p_diag.enableAutoRange(axis="y")

    def _fill_live_table(self):
        c = self.live_buf["main"].get("counters", {})
        rows = [("Sync/Follow_Up accoppiati", c.get("pairs")), ("campioni di delay", c.get("delay_samples")),
                ("step di clock", c.get("steps")), ("reset del servo", c.get("resets")),
                ("reset per comando fuori range", c.get("range_resets")), ("outlier rifiutati", c.get("outliers")),
                ("messaggi persi", c.get("lost"))]
        self.table.clearSpans()
        self.table.setRowCount(len(rows))
        for i, (k, v) in enumerate(rows):
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(k))
            self.table.setItem(i, 1, QtWidgets.QTableWidgetItem(str(v)))
            for j in (2, 3, 4):
                self.table.setItem(i, j, QtWidgets.QTableWidgetItem(""))
        self.table.setHorizontalHeaderLabels(["Contatori live (le metriche di assestamento/RMS: modalità Esplorazione)",
                                              "valore", "", "", ""])

    def _fill_table(self):
        m, bm = self.metrics, self.base_metrics
        if m is None:
            return

        def f(v, nd=2, unit=""):
            if v is None:
                return "–"
            if isinstance(v, float) and v != v:
                return "–"
            return f"{v:.{nd}f}{unit}"

        def col(mm, kind):
            if mm is None:
                return [""] * 9
            d = mm[kind]
            return [f(d["settling_s"], 2, " s") if d["settling_s"] is not None else "– (" + d["settling_reason"] + ")",
                    f(d["overshoot_pct"], 1, " %") if d["overshoot_pct"] is not None else "– (offset iniziale nullo)",
                    f(d["peak_abs_ns"] * self._scale(), 3, " " + self.units.currentText()),
                    f(d["rms_final_ns"], 2, " ns"), f(d["bias_final_ns"], 2, " ns"),
                    "DIVERGE" if d["diverged"] else "no", "", "", ""]

        names = ["tempo di assestamento", "sovraelongazione", "picco |errore|", "RMS (finestra finale)",
                 "bias (finestra finale)", "divergente", "saturazione/reset", "tempo di calcolo", "aggiornamento GUI"]
        cols = [col(m, "true_offset"), col(m, "estimated_offset"), col(bm, "true_offset"), col(bm, "estimated_offset")]
        self.table.setHorizontalHeaderLabels(["Metrica", "Offset reale", "Offset stimato", "Baseline reale", "Baseline stimato"])
        self.table.clearSpans()
        s = m["saturation"]
        sat = f"clamp {s['controller_clamped']}, range-reset {s['range_resets']}, reset {s['resets']}, step {s['steps']}, outlier {s['outliers_rejected']}"
        self.table.setRowCount(len(names))
        for i, nme in enumerate(names):
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(nme))
            for j, c in enumerate(cols):
                txt = c[i] if c else ""
                if i == 6 and j == 0:
                    txt = sat
                if i == 7 and j == 0:
                    txt = f"sim {m['sim_wall_ms']:.1f} ms + metriche {m['metrics_wall_ms']:.1f} ms; {m['n_events']} eventi"
                if i == 8 and j == 0:
                    txt = f"{self.last_ms['gui']:.0f} ms dalla modifica (calcolo totale {self.last_ms['compute']:.0f} ms)"
                if j == 2 and bm is None:
                    pass
                self.table.setItem(i, j + 1, QtWidgets.QTableWidgetItem(txt))
        for r in (6, 7, 8):
            self.table.setSpan(r, 1, 1, 4)
        self.table.resizeColumnsToContents()

    # ------------------------------------------------------------------ files
    def _save_cfg(self):
        p, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Salva configurazione", "configs/scenario.json", "JSON (*.json)")
        if p:
            self.build_config().save(p)

    def _load_cfg(self):
        p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Carica configurazione", "configs", "JSON (*.json)")
        if p:
            self._apply_cfg_to_widgets(SimConfig.load(p))

    def _export(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Cartella di esportazione", "results")
        if d:
            self.status.setText("esportazione…")
            self.req_q.put({"cmd": "export", "cfg": self.build_config().to_dict(), "dir": d, "mc": self._metrics_cfg()})


def main(argv=None):
    app = QtWidgets.QApplication(sys.argv if argv is None else argv)
    cfg = None
    if len(sys.argv) > 1 and sys.argv[1].endswith(".json"):
        cfg = SimConfig.load(sys.argv[1])
    w = MainWindow(cfg)
    w.show()
    code = app.exec()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
