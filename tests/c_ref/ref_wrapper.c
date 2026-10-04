/* SPDX-License-Identifier: Apache-2.0
 *
 * Thin C wrapper exposing the vendored Zephyr firmware arithmetic to the Python
 * test-suite through ctypes.  The vendored files are verbatim copies of
 * Zephyr zmagnifico-integration-2 @ 553d973f1b78 (see README in this directory).
 */
#include <stdint.h>
#include <zephyr/precision_timing/precision_pi.h>
#include <zephyr/precision_timing/precision_clock.h>
#include "ptp_clock_nxp_enet_rate_math.h"

double ref_pi_run(double kp, double ki, const double *err, double *out, int n, double *integral)
{
	struct precision_pi pi;

	precision_pi_init(&pi, kp, ki);
	for (int i = 0; i < n; i++) {
		out[i] = precision_pi_update(&pi, err[i]);
	}
	*integral = pi.integral;
	return pi.integral;
}

int ref_ppb_to_scaled_ppm(double ppb, int64_t *scaled_ppm)
{
	return precision_clock_ppb_to_scaled_ppm(ppb, scaled_ppm);
}

int ref_find_correction(int inc, double target_ns, int inc_corr_max, uint32_t cor_max,
			int *inc_corr, uint32_t *cor)
{
	return ptp_clock_nxp_enet_find_correction(inc, target_ns, inc_corr_max, cor_max, inc_corr,
						  cor);
}
