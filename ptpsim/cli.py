# SPDX-License-Identifier: Apache-2.0
"""Command line: ``ptpsim-run run|compare``."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import SimConfig, noisy_preset
from .engine import simulate
from .export import export_result
from .metrics import compute_metrics


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ptpsim-run", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run one configuration and export CSV/JSON")
    r.add_argument("--config", help="JSON configuration (default: built-in deterministic scenario)")
    r.add_argument("--preset", choices=["default", "noisy"], default="default")
    r.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override, e.g. --set intervals.sync_log=-3 (JSON values)")
    r.add_argument("--out", default="results/run", help="output directory")
    c = sub.add_parser("compare", help="baseline vs variant comparison (writes comparison.csv/.md)")
    c.add_argument("--out", default="results")
    c.add_argument("--seeds", type=int, default=10)
    c.add_argument("--quick", action="store_true")
    sub.add_parser("gui", help="start the GUI")
    a = ap.parse_args(argv)

    if a.cmd == "run":
        cfg = SimConfig.load(a.config) if a.config else (noisy_preset() if a.preset == "noisy" else SimConfig())
        ov = {}
        for kv in a.set:
            k, v = kv.split("=", 1)
            ov[k] = json.loads(v)
        cfg = cfg.with_overrides(**ov) if ov else cfg
        res = simulate(cfg)
        export_result(res, a.out)
        m = compute_metrics(res)
        print(json.dumps({k: m[k] for k in ("true_offset", "estimated_offset", "saturation")}, indent=2, default=float))
        print(f"wall {res.wall_s * 1e3:.1f} ms, {res.n_events} events; written to {Path(a.out).resolve()}")
    elif a.cmd == "compare":
        from .compare import run_comparison
        out = run_comparison(a.out, seeds=a.seeds, quick=a.quick)
        print(f"written {out}/comparison.csv and comparison.md")
    elif a.cmd == "gui":
        from .gui.app import main as gui_main
        gui_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
