# SPDX-License-Identifier: Apache-2.0
"""Regenerate the GUI screenshots used by the manuals and the README (docs/img/gui_<scene>_<lang>.png).

The real window and worker process are driven offscreen (``pip install '.[gui]'``, no display needed).  Every scene is a
documented scenario (see the manuals, section "Recipes"), run with the real worker, and every image is taken after the
result has arrived.  Re-read the images and their tables before committing: the manuals quote their numbers.

    python scripts/make_gui_figures.py                       # all scenes, en + it
    python scripts/make_gui_figures.py --scene run live --lang en --out /tmp/shots

Live scenes depend on the wall clock (the instant of the parameter change varies slightly between runs).
"""
import argparse
import os
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "img"
SIZE = (1500, 950)             # the metrics table needs the width: at 1400 px a column is truncated

# name -> scenario.  ``cfg``: SimConfig overrides (``config.with_overrides`` keys); ``base``: JSON config to start from.
SCENES = {
    # hero / "The window": baseline vs pi_per_second at Sync 62.5 ms, 100 us step (numbers in docs/model.md section 6)
    "run": dict(cfg={"duration_s": 90.0, "intervals.sync_log": -4, "oscillator.initial_offset_ns": 100e3,
                     "oscillator.freq_error_ppb": 0.0, "controller.name": "pi_per_second",
                     "controller.params": {"kp": 0.7, "ki": 0.3}}, overlay=True),
    # tab Controller
    "controller": dict(cfg={"duration_s": 90.0, "intervals.sync_log": -4, "oscillator.initial_offset_ns": 100e3,
                            "oscillator.freq_error_ppb": 0.0, "controller.name": "pi_per_second",
                            "controller.params": {"kp": 0.7, "ki": 0.3}}, tab=1),
    # recipe "Compare controllers": noisy network, same seed, variant over the firmware baseline
    "noisy": dict(base="noisy_seed1.json", cfg={"controller.name": "pi_time_aware", "controller.params": {}},
                  overlay=True),
    # recipe "Size the anti-windup": 100 ms offset, command clamp 1000 ppm, PI terms
    "windup": dict(cfg={"duration_s": 300.0, "oscillator.initial_offset_ns": 100e6, "oscillator.freq_error_ppb": 20000.0,
                        "firmware.cmd_clamp_ppm": 1000.0, "controller.name": "baseline_pi",
                        "controller.params": {"kp": 0.7, "ki": 0.3}}, pi_terms=True, units="ms", size=(1500, 1050)),
    "antiwindup": dict(cfg={"duration_s": 300.0, "oscillator.initial_offset_ns": 100e6,
                            "oscillator.freq_error_ppb": 20000.0, "firmware.cmd_clamp_ppm": 1000.0,
                            "controller.name": "pi_anti_windup",
                            "controller.params": {"kp": 0.7, "ki": 0.3, "i_max_ppm": 100.0}}, pi_terms=True, units="ms",
                       size=(1500, 1050)),
    # Zoom row: x zoomed on the transient in all plots, y of the delay plot left alone
    "zoom": dict(cfg={"duration_s": 90.0, "intervals.sync_log": -4, "oscillator.initial_offset_ns": 100e3,
                      "oscillator.freq_error_ppb": 0.0, "controller.name": "pi_per_second",
                      "controller.params": {"kp": 0.7, "ki": 0.3}}, overlay=True, zoom=True),
    # recipe "Live tuning": kp raised while the run is going, change marked on the plots
    "live": dict(cfg={"duration_s": 120.0, "intervals.sync_log": -2, "oscillator.initial_offset_ns": 100e3,
                      "oscillator.freq_error_ppb": 20000.0, "controller.name": "baseline_pi",
                      "controller.params": {"kp": 0.7, "ki": 0.3}}, live=True),
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scene", nargs="+", choices=sorted(SCENES), default=list(SCENES))
    ap.add_argument("--lang", nargs="+", choices=("en", "it"), default=["en", "it"])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for each result")
    args = ap.parse_args(argv)

    from PySide6 import QtCore, QtWidgets
    from ptpsim.config import SimConfig
    from ptpsim.gui.app import MainWindow

    args.out.mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication([])

    def spin(ms):
        loop = QtCore.QEventLoop()                   # a real loop: deleteLater and queued signals need it
        QtCore.QTimer.singleShot(ms, loop.quit)
        loop.exec()

    def until(cond, what):
        t0 = time.time()
        while not cond():
            spin(20)
            if time.time() - t0 > args.timeout:
                raise TimeoutError(what)

    failed = 0
    for lang in args.lang:
        for name in args.scene:
            sc = SCENES[name]
            cfg = SimConfig.load(ROOT / "configs" / sc["base"]) if "base" in sc else SimConfig()
            cfg = cfg.with_overrides(**sc["cfg"])
            w = MainWindow(cfg, lang=lang)
            try:
                w.resize(*sc.get("size", SIZE))
                w.show()
                assert w.build_config().to_dict() == cfg.to_dict(), "widgets do not reproduce the scene config"
                until(lambda: w.n_results > 0, "first result")
                if sc.get("live"):
                    _live_scene(w, spin, until)
                else:
                    if sc.get("overlay"):
                        n = w.n_results
                        w.chk_overlay.setChecked(True)
                        until(lambda: w.n_results > n and not w.debounce.isActive(), "overlay result")
                    if "units" in sc:
                        w.units.setCurrentText(sc["units"])
                    if sc.get("pi_terms"):
                        w.chk_pi.setChecked(True)
                    if sc.get("zoom"):
                        _zoom_scene(w, spin)
                    w.tabs.setCurrentIndex(sc.get("tab", 0))
                spin(400)                            # redraw, deferred deletes, layout
                path = args.out / f"gui_{name}_{lang}.png"
                w.grab().save(str(path))
                print(f"{path.relative_to(ROOT) if path.is_relative_to(ROOT) else path}  {w.status.text()}")
            except Exception as e:                   # keep going: report and fail at the end
                failed += 1
                print(f"{name}/{lang}: FAILED {type(e).__name__}: {e}")
            finally:
                w.close()
                spin(200)
    return 1 if failed else 0


def _zoom_scene(w, spin):
    """Wheel-zoom the time axis (all plots) at the transient, y of the delay plot excluded from the zoom."""
    from PySide6 import QtCore, QtGui, QtWidgets
    w.chk_zoom_y["delay"].setChecked(False)
    w.p_delay.setXRange(2.0, 22.0, padding=0)          # a deterministic x window instead of a wheel position
    spin(300)
    vp = w.pw_off.viewport()
    pos = QtCore.QPointF(vp.width() * 0.3, vp.height() / 2)
    ev = QtGui.QWheelEvent(pos, vp.mapToGlobal(pos), QtCore.QPoint(0, 0), QtCore.QPoint(0, 120), QtCore.Qt.NoButton,
                           QtCore.Qt.NoModifier, QtCore.Qt.NoScrollPhase, False)
    QtWidgets.QApplication.sendEvent(vp, ev)           # one wheel notch: x zoomed, y of the offset plot too


def _live_scene(w, spin, until):
    w.mode_live.setChecked(True)
    until(lambda: not w.live_resetting and w.live_state == "stopped", "live reset")
    spin(300)
    w.speed.spin.setValue(10.0)
    w._live_start()
    until(lambda: w.live_t >= 4.5, "live t=4.5")      # inside the transient
    until(lambda: not w.live_busy, "live idle")
    w.ctrl_rows["kp"].spin.setValue(1.4)
    until(lambda: bool(w.live_buf.get("main", {}).get("changes")), "change applied")
    until(lambda: w.live_t >= 30.0, "live t=30")
    w._live_pause()
    until(lambda: not w.live_busy, "live idle")
    spin(300)
    w._redraw_live(force=True)


if __name__ == "__main__":                              # the guard is required: the worker is spawned
    raise SystemExit(main())
