# Median |offset| [ns] vs oscillator error, NXP actuator, noisy preset, baseline PI

Last 200 s of 300 s, median over 5 seeds of the median |true offset|. Last row: values reported on hardware in the commit message of PR #121108 (conditions unknown).

| clock root | 0 ppm | 0.001 ppm | 0.003 ppm | 0.01 ppm | 0.03 ppm | 0.1 ppm | 0.5 ppm | 2 ppm | 20 ppm |
|---|---|---|---|---|---|---|---|---|---|
| 24 MHz | 0 | 200 | 733 | 2741 | 6616 | 3496 | 4017 | 3739 | 2953 |
| 98.304 MHz | 158 | 153 | 150 | 157 | 154 | 157 | 159 | 161 | 162 |
| 100 MHz | 159 | 159 | 159 | 159 | 159 | 159 | 159 | 159 | 158 |
| 196.608 MHz | 159 | 156 | 157 | 155 | 154 | 150 | 156 | 162 | 154 |
| PR #121108 (hardware) | 24 MHz: 236 | 98.304 MHz: 181 | 100 MHz: 144 | 196.608 MHz: 152 |
