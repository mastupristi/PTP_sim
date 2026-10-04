# Vendored C reference sources

Verbatim copies (Apache-2.0, SPDX headers preserved) of Zephyr files, used only by
`tests/test_c_reference.py` to compare the Python port bit-for-bit against the real C code:

| File | Origin |
|---|---|
| `precision_pi.c`, `precision_pi.h` | `subsys/precision_timing/`, `include/zephyr/precision_timing/` |
| `precision_clock.c`, `precision_clock.h` | idem (`precision_clock_ppb_to_scaled_ppm`) |
| `ptp_clock_nxp_enet_rate_math.c/.h` | `drivers/ptp_clock/` (PR #121108) |

Source commit: `zmagnifico-integration-2` @ `553d973f1b78f64a9f7e49bbb2c8b7acee8701db`.
`ref_wrapper.c` is the only file written for this project. The test builds everything with `gcc`
into a temporary shared library (a tiny stub of `precision_time.h` is generated at build time).
