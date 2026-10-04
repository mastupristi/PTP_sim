# SPDX-License-Identifier: Apache-2.0
"""Table of the NXP ENET timer behaviour per clock root (run: python scripts/nxp_table.py)."""
import numpy as np

from ptpsim import fwport

ROOTS = [("100 MHz (SYS_PLL1_DIV2/5, SoC default)", 100_000_000), ("98.304 MHz (AUDIO_PLL/4)", 98_304_000),
         ("196.608 MHz (AUDIO_PLL/2)", 196_608_000), ("24 MHz (OSC_24M)", 24_000_000), ("25 MHz", 25_000_000)]


def row(name, hz):
    t = fwport.NxpTimer(hz)
    inc, ic, cor = t.inc, t.inc_corr, t.cor
    nominal_resid_ppm = t.effective_delta * 1e6
    # reachable rates within +-200 ppm and the worst error of the average tick (as PR #121108's test)
    reach, worst = set(), 0.0
    for ppm in np.arange(-200.0, 200.0, 0.25):
        tt = fwport.NxpTimer(hz)
        tt.rate_adjust(1.0 + ppm * 1e-6)
        reach.add(round(tt.effective_delta * 1e6, 4))
        worst = max(worst, abs(tt.effective_delta * 1e6 - ppm))
    r = np.array(sorted(reach))
    gap = float(np.max(np.diff(r[(r > -200) & (r < 200)]))) if r.size > 1 else float("nan")
    saw = float(ic - inc) if cor else 0.0
    return (f"| {name} | {inc} | {ic} | {cor} | {nominal_resid_ppm:+.3f} | {saw:.0f} | {gap:.2f} | {worst:.2f} |")


if __name__ == "__main__":
    print("| clock root | INC | INC_CORR (ratio 1.0) | ATCOR | nominal-pair error [ppm] | counter step pattern pk-pk [ns] "
          "| largest gap between reachable rates in ±200 ppm [ppm] | worst request error in ±200 ppm [ppm] |")
    print("|---|---|---|---|---|---|---|---|")
    for n, h in ROOTS:
        print(row(n, h))
