# SPDX-License-Identifier: Apache-2.0
"""Drive the real GUI (worker process included) offscreen and save screenshots.

Needs the GUI extra (``pip install '.[gui]'``); no display is required (Qt ``offscreen`` platform).
Qt is driven from code, not with a mouse, and the rendering is not that of a desktop session.

    python scripts/gui_screenshots.py --out /tmp/shots --controller pi_per_second --sync -4 -2 0
    python scripts/gui_screenshots.py --out /tmp/shots --controller pi_time_aware --offset-us 100 --overlay

One ``<controller>_sync<n>.png`` per Sync exponent (Execution tab: both plots and the metrics table) plus
``<controller>_controller_tab.png``.  Waiting uses a real Qt event loop: ``processEvents()`` alone does not
run ``deleteLater``, so widgets removed on a controller switch would linger in the screenshot.
"""
import argparse
import os
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, required=True, help="output directory (created)")
    ap.add_argument("--config", help="scenario JSON to start from (default: SimConfig defaults)")
    ap.add_argument("--controller", default="baseline_pi")
    ap.add_argument("--sync", type=int, nargs="+", default=[-2], help="Sync exponents n (interval 2^n s)")
    ap.add_argument("--offset-us", type=float, default=100.0, help="initial offset [us]")
    ap.add_argument("--duration", type=float, default=60.0, help="simulated seconds")
    ap.add_argument("--overlay", action="store_true", help="draw the unmodified-firmware baseline overlay")
    ap.add_argument("--lang", choices=("en", "it"), default="en")
    ap.add_argument("--size", type=int, nargs=2, default=(1500, 950), metavar=("W", "H"))
    ap.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for each result")
    args = ap.parse_args(argv)

    from PySide6 import QtCore, QtWidgets
    from ptpsim.config import SimConfig
    from ptpsim.gui.app import MainWindow

    args.out.mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication([])
    cfg = SimConfig.load(args.config) if args.config else SimConfig()
    cfg.duration_s = args.duration
    w = MainWindow(cfg, lang=args.lang)
    w.resize(*args.size)
    w.show()

    def spin(ms: int) -> None:
        loop = QtCore.QEventLoop()
        QtCore.QTimer.singleShot(ms, loop.quit)
        loop.exec()

    def wait_result(n_before: int) -> bool:
        t0 = time.time()
        while time.time() - t0 < args.timeout:
            spin(20)
            if w.n_results > n_before and not w.debounce.isActive():
                spin(200)                       # let the redraw and the deferred deletes run
                return True
        return False

    if not wait_result(-1):
        print("no initial result from the worker")
        w.close()
        return 1
    w.rows["offset0"].spin.setValue(args.offset_us)
    w.ctrl_combo.setCurrentText(args.controller)
    w.chk_overlay.setChecked(args.overlay)
    rc = 0
    for n in args.sync:
        before = w.n_results
        w.rows["sync_log"].spin.setValue(n)
        ok = wait_result(before)
        path = args.out / f"{args.controller}_sync{n}.png"
        w.grab().save(str(path))
        print(f"{path}  sync 2^{n} s  {'ok' if ok else 'TIMEOUT'}  {w.status.text()}")
        rc |= 0 if ok else 1
    w.tabs.setCurrentIndex(1)                    # Controller tab
    spin(300)
    path = args.out / f"{args.controller}_controller_tab.png"
    w.tabs.grab().save(str(path))
    print(path)
    w.close()
    return rc


if __name__ == "__main__":                       # the guard is required: the worker is spawned
    raise SystemExit(main())
