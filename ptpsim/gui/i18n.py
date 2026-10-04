# SPDX-License-Identifier: Apache-2.0
"""GUI strings in English (default) and Italian.  ``T(key, **fmt)`` uses the current language."""
from __future__ import annotations

LANGS = {"en": "English", "it": "Italiano"}
_lang = "en"


def set_lang(lang: str) -> None:
    global _lang
    _lang = lang if lang in LANGS else "en"


def get_lang() -> str:
    return _lang


def T(key: str, **fmt) -> str:
    entry = STRINGS.get(key)
    if entry is None:
        return key
    text = entry.get(_lang) or entry["en"]
    return text.format(**fmt) if fmt else text


def _s(en: str, it: str) -> dict:
    return {"en": en, "it": it}


STRINGS: dict[str, dict] = {
    # --- window / tabs
    "title": _s("PTP_sim — closed-loop PTPv2 simulator (Zephyr time receiver)",
                "PTP_sim — simulatore PTPv2 ad anello chiuso (time receiver Zephyr)"),
    "tab_run": _s("Run", "Esecuzione"),
    "tab_ctrl": _s("Controller", "Controllore"),
    "tab_ptp": _s("PTP intervals", "Intervalli PTP"),
    "tab_scen": _s("Scenario", "Scenario"),
    "tab_net": _s("Network and noise", "Rete e disturbi"),
    "tab_act": _s("Actuator", "Attuatore"),
    # --- run tab
    "g_mode": _s("Mode and control", "Modalità e controllo"),
    "mode_explore": _s("Exploration (recompute the whole trajectory, same seed)",
                       "Esplorazione (ricalcola tutta la traiettoria, stesso seed)"),
    "mode_live": _s("Live simulation (new parameters apply from now on)",
                    "Simulazione live (i parametri valgono da ora in poi)"),
    "btn_start": _s("▶ Start", "▶ Start"),
    "btn_pause": _s("⏸ Pause", "⏸ Pausa"),
    "btn_reset": _s("⟲ Reset", "⟲ Reset"),
    "speed": _s("Playback speed", "Velocità riproduzione"),
    "speed_tip": _s("Live only: simulated seconds per real second. It never changes the maths.",
                    "Solo live: secondi simulati per secondo reale. Non modifica la matematica."),
    "policy_lbl": _s("Integrator policy when gains change", "Politica integratore al cambio guadagni"),
    "policy_keep": _s("keep (keep the integrator)", "keep (mantieni integratore)"),
    "policy_reset": _s("reset (zero the integrator)", "reset (azzera integratore)"),
    "policy_bumpless": _s("bumpless (continuous output)", "bumpless (uscita continua)"),
    "follow": _s("Follow (sliding window)", "Segui (finestra scorrevole)"),
    "follow_win": _s("Live window", "Finestra live"),
    "g_metrics": _s("Metrics settings", "Impostazioni metriche"),
    "m_band": _s("Settling band (±)", "Banda di assestamento (±)"),
    "m_band_tip": _s("The transient ends when |offset| stays inside this band.",
                     "Il transitorio finisce quando |offset| resta dentro questa banda."),
    "m_dwell": _s("Dwell time", "Permanenza"),
    "m_dwell_tip": _s("The offset must stay in the band at least this long to count as settled.",
                      "L'offset deve restare in banda almeno questo tempo per contare come assestato."),
    "m_win": _s("Steady-state window (last)", "Finestra di regime (ultimi)"),
    "m_from_settle": _s("Steady state = after the transient end", "Regime = dopo la fine del transitorio"),
    "g_files": _s("Files", "File"),
    "btn_save": _s("Save config/seed…", "Salva config/seed…"),
    "btn_load": _s("Load config…", "Carica config…"),
    "btn_export": _s("Export results CSV…", "Esporta CSV risultati…"),
    "dlg_save": _s("Save configuration", "Salva configurazione"),
    "dlg_load": _s("Load configuration", "Carica configurazione"),
    "dlg_export": _s("Export folder", "Cartella di esportazione"),
    # --- controller
    "g_ctrl": _s("Controller", "Controllore"),
    "ctrl_lbl": _s("Controller", "Controllore"),
    "p_kp": _s("kp", "kp"), "p_ki": _s("ki", "ki"),
    "pd_kp": _s("proportional gain [ppb/ns]", "guadagno proporzionale [ppb/ns]"),
    "pd_ki": _s("integral gain [ppb/ns per update]", "guadagno integrale [ppb/ns per aggiornamento]"),
    "pd_wn": _s("natural frequency [rad/s]", "frequenza naturale [rad/s]"),
    "pd_zeta": _s("damping ratio []", "smorzamento []"),
    "pd_sat_ppb": _s("command limit [ppb] (must stay below the actuator limit)",
                     "limite del comando [ppb] (deve restare sotto il limite dell'attuatore)"),
    "pd_wn_ts_max": _s("max wn·dt [rad] (stability guard, kp·dt < 2)", "max wn·dt [rad] (guardia di stabilità, kp·dt < 2)"),
    "pd_dt_clamp": _s("measured dt is clamped to dt_clamp × nominal interval",
                      "il dt misurato è limitato a dt_clamp × intervallo nominale"),
    # --- intervals
    "g_int": _s("PTP intervals (1 s × 2ⁿ)", "Intervalli PTP (1 s × 2ⁿ)"),
    "sync_n": _s("Sync: exponent n", "Sync: esponente n"),
    "sync_tip": _s("logMessageInterval of Sync/Follow_Up (default -2 = 0.25 s)",
                   "logMessageInterval di Sync/Follow_Up (default -2 = 0.25 s)"),
    "sync_txt": _s("Sync interval: {t}", "Intervallo Sync: {t}"),
    "dly_lbl": _s("Delay_Req", "Delay_Req"),
    "dly_ind": _s("Independent interval (as the firmware)", "Intervallo indipendente (come il firmware)"),
    "dly_every": _s("Every N Sync (NOT in the firmware)", "Ogni N Sync (NON nel firmware)"),
    "dly_n": _s("Delay_Req: exponent n", "Delay_Req: esponente n"),
    "dly_tip": _s("Independent interval: the client's k_timer (monotonic, not disciplined)",
                  "Intervallo indipendente: k_timer del client (monotono, non disciplinato)"),
    "dly_txt": _s("Delay_Req interval: {t}", "Intervallo Delay_Req: {t}"),
    "dly_txt_n": _s("   (active: every {n} Sync = {s:g} s)", "   (attivo: ogni {n} Sync = {s:g} s)"),
    "every_n": _s("Every N Sync: N", "Ogni N Sync: N"),
    "rearm": _s("Delay_Req timer re-armed at handling (cumulative drift, firmware)",
                "Timer Delay_Req riarmato alla gestione (deriva cumulativa, firmware)"),
    # --- scenario
    "g_init": _s("Initial conditions / duration", "Condizioni iniziali / durata"),
    "off0": _s("Initial offset", "Offset iniziale"),
    "off0_tip": _s("slave − GM at t = 0, up to ±2e9 s. Beyond 1 s the firmware steps the clock (forced alignment); "
                   "above ≈71 ms the baseline PI asks > 50000 ppm and the driver rejects it (servo reset).",
                   "slave − GM a t = 0, fino a ±2e9 s. Oltre 1 s il firmware fa uno step del clock (riallineamento "
                   "forzato); oltre ≈71 ms il PI baseline chiede > 50000 ppm e il driver rifiuta (reset del servo)."),
    "btn_phc0": _s("Slave PHC starts at 0 (offset = −epoch)", "PHC slave parte da 0 (offset = −epoch)"),
    "freq0": _s("Frequency error", "Errore di frequenza"),
    "duration": _s("Duration", "Durata"),
    "seed": _s("Seed", "Seed"),
    "drift": _s("Oscillator drift", "Deriva oscillatore"),
    "timer_err": _s("k_timer error", "Errore timer k_timer"),
    "timer_tip": _s("The Delay_Req timer is a monotonic clock independent of the disciplined PHC",
                    "Il timer dei Delay_Req è un clock monotono indipendente dal PHC disciplinato"),
    # --- network
    "g_net": _s("Network and disturbances", "Rete e disturbi"),
    "d_mean": _s("Mean delay", "Ritardo medio"),
    "d_asym": _s("Asymmetry (GM→S − S→GM)", "Asimmetria (GM→S − S→GM)"),
    "d_asym_tip": _s("Bias of the estimated offset = asymmetry / 2", "Bias dell'offset stimato = asimmetria / 2"),
    "net_jit": _s("Path jitter (exp., each direction)", "Jitter di rete (esp., ogni direzione)"),
    "tx_jit": _s("Send jitter (σ, all messages)", "Jitter di invio (σ, tutti i messaggi)"),
    "tx_jit_tip": _s("Around the nominal instant (not cumulative)", "Attorno all'istante nominale (non cumulativo)"),
    "lat_fup": _s("Follow_Up latency", "Latenza Follow_Up"),
    "lat_dresp": _s("Delay_Resp latency", "Latenza Delay_Resp"),
    "lat_cmd": _s("Command application latency", "Latenza applicazione comando"),
    "lat_step": _s("Clock step latency (read→set)", "Latenza step clock (read→set)"),
    "lat_step_tip": _s("Time between reading the PHC and setting it inside clock_step: the residual error after a "
                       "forced alignment", "Tempo fra lettura e scrittura del PHC dentro clock_step: errore residuo "
                       "dopo un riallineamento forzato"),
    "ts_noise": _s("Timestamp noise (σ, GM and slave)", "Rumore timestamp (σ, GM e slave)"),
    "gm_q": _s("GM timestamp quantisation", "Quantizzazione timestamp GM"),
    "loss": _s("Message loss (each type)", "Perdita messaggi (ogni tipo)"),
    "btn_noisy": _s("Noisy preset (20 µs jitter, 200 ns network, 3 ns ts)",
                    "Preset rumoroso (jitter 20 µs, rete 200 ns, ts 3 ns)"),
    # --- actuator
    "g_act": _s("Actuator", "Attuatore"),
    "act_type": _s("Type", "Tipo"),
    "act_ideal": _s("ideal", "ideale"),
    "act_nxp": _s("NXP ENET (PR #121108)", "NXP ENET (PR #121108)"),
    "act_root": _s("1588 timer clock root", "Clock root timer 1588"),
    "act_info_ideal": _s("ideal actuator — rate ratio {r:.12f}", "attuatore ideale — rate ratio {r:.12f}"),
    "act_info_nxp": _s("NXP: INC={inc}  INC_CORR={ic}  ATCOR={cor}  (effective ratio {r:.9f})",
                       "NXP: INC={inc}  INC_CORR={ic}  ATCOR={cor}  (ratio effettivo {r:.9f})"),
    # --- toolbar
    "units": _s("Units:", "Unità:"),
    "overlay": _s("Overlay baseline (firmware PI)", "Sovrapponi baseline (PI firmware)"),
    "show_est": _s("Estimated offset", "Offset stimato"),
    "show_diag": _s("Rate diagnostics", "Diagnostica rate"),
    "view": _s("View:", "Vista:"),
    "view_full": _s("Full", "Completa"),
    "view_trans": _s("Transient", "Transitorio"),
    "view_steady": _s("Steady state", "Regime"),
    "show_trans": _s("Transient end line", "Riga fine transitorio"),
    "btn_fit": _s("Fit view", "Adatta vista"),
    "lang": _s("Language:", "Lingua:"),
    # --- status
    "st_start": _s("starting worker…", "avvio worker…"),
    "st_calc": _s("computing…", "calcolo in corso…"),
    "st_done": _s("updated in {lat:.0f} ms (compute {c:.0f} ms, sim {s:.0f} ms)",
                  "aggiornato in {lat:.0f} ms (calcolo {c:.0f} ms, sim {s:.0f} ms)"),
    "st_err": _s("worker ERROR (see console)", "ERRORE nel worker (vedi console)"),
    "st_export": _s("exporting…", "esportazione…"),
    "st_exported": _s("exported to {p}", "esportato in {p}"),
    "st_live_ready": _s("live ready (t = 0)", "live pronto (t = 0)"),
    "st_live": _s("live  t = {t:.2f} s", "live  t = {t:.2f} s"),
    # --- plots
    "pl_delay": _s("Mean delay (firmware estimate vs physical reference)", "Delay medio (stima del firmware vs riferimento fisico)"),
    "pl_off": _s("Offset slave − GM", "Offset slave − GM"),
    "pl_diag": _s("Rate: commanded by the servo and effective of the clock", "Rate: comandato dal servo e effettivo del clock"),
    "ax_time": _s("physical (GM) time [s]", "tempo fisico (GM) [s]"),
    "ax_off": _s("offset [{u}]", "offset [{u}]"),
    "ax_delay": _s("delay [{u}]", "delay [{u}]"),
    "lg_delay_step": _s("estimated delay (held until the next sample)", "delay stimato (tenuto fino al campione successivo)"),
    "lg_delay_pts": _s("samples (Delay_Resp processed)", "campioni (Delay_Resp elaborate)"),
    "lg_delay_ref": _s("physical delay (network)", "delay fisico (rete)"),
    "lg_true": _s("true offset", "offset reale"),
    "lg_est": _s("estimated offset (samples)", "offset stimato (campioni)"),
    "lg_base_true": _s("baseline: true offset", "baseline: offset reale"),
    "lg_base_est": _s("baseline: estimated offset", "baseline: offset stimato"),
    "lg_cmd": _s("commanded (ppb)", "comandato (ppb)"),
    "lg_eff": _s("effective clock rate vs GM (ppb)", "rate effettivo clock vs GM (ppb)"),
    "tr_line": _s("transient ends {t:.2f} s", "fine transitorio {t:.2f} s"),
    "tr_line_base": _s("baseline: {t:.2f} s", "baseline: {t:.2f} s"),
    # --- table
    "th_metric": _s("Metric", "Metrica"),
    "th_true": _s("True offset", "Offset reale"),
    "th_est": _s("Estimated offset", "Offset stimato"),
    "th_btrue": _s("Baseline true", "Baseline reale"),
    "th_best": _s("Baseline estimated", "Baseline stimato"),
    "m_transient": _s("transient end (settling)", "fine transitorio (assestamento)"),
    "m_overshoot": _s("overshoot (vs servo start)", "sovraelongazione (vs avvio servo)"),
    "m_peak": _s("peak |error| (from servo start)", "picco |errore| (dall'avvio servo)"),
    "m_ss_median": _s("steady state: median", "regime: mediana"),
    "m_ss_min": _s("steady state: minimum", "regime: minimo"),
    "m_ss_max": _s("steady state: maximum", "regime: massimo"),
    "m_ss_medabs": _s("steady state: median |x|", "regime: mediana |x|"),
    "m_rms": _s("steady state: RMS", "regime: RMS"),
    "m_bias": _s("steady state: mean (bias)", "regime: media (bias)"),
    "m_diverged": _s("diverged", "divergente"),
    "m_delay": _s("delay estimate (steady): median / min / max", "stima delay (regime): mediana / min / max"),
    "m_sat": _s("saturation / resets", "saturazione / reset"),
    "m_time": _s("compute time", "tempo di calcolo"),
    "m_gui": _s("GUI update", "aggiornamento GUI"),
    "v_yes": _s("DIVERGED", "DIVERGE"), "v_no": _s("no", "no"),
    "v_ov_undef": _s("– (reference offset is zero)", "– (offset di riferimento nullo)"),
    "rs_in_band": _s("within band from the start", "in banda fin dall'inizio"),
    "rs_outside_end": _s("still outside the band at the end", "ancora fuori banda alla fine"),
    "rs_dwell": _s("in band for less than the dwell time", "in banda per meno della permanenza"),
    "rs_nodata": _s("no data", "nessun dato"), "rs_nonfinite": _s("non-finite values", "valori non finiti"),
    "never": _s("– ({why})", "– ({why})"),
    "sat_txt": _s("clamp {a}, range-reset {b}, reset {c}, step {d}, outlier {e}",
                  "clamp {a}, range-reset {b}, reset {c}, step {d}, outlier {e}"),
    "time_txt": _s("sim {a:.1f} ms + metrics {b:.1f} ms; {n} events", "sim {a:.1f} ms + metriche {b:.1f} ms; {n} eventi"),
    "gui_txt": _s("{a:.0f} ms from the change (total compute {b:.0f} ms)", "{a:.0f} ms dalla modifica (calcolo totale {b:.0f} ms)"),
    "live_hdr": _s("Live counters (settling/RMS metrics: Exploration mode)",
                   "Contatori live (le metriche di assestamento/RMS: modalità Esplorazione)"),
    "live_val": _s("value", "valore"),
    "c_pairs": _s("Sync/Follow_Up pairs", "Sync/Follow_Up accoppiati"),
    "c_delay": _s("delay samples", "campioni di delay"),
    "c_steps": _s("clock steps", "step di clock"),
    "c_resets": _s("servo resets", "reset del servo"),
    "c_range": _s("out-of-range resets", "reset per comando fuori range"),
    "c_outl": _s("outliers rejected", "outlier rifiutati"),
    "c_lost": _s("lost messages", "messaggi persi"),
    "cn_baseline_pi": _s(
        "Faithful port of the firmware PI (precision_pi_update): integral += ki·e; u = kp·e + integral; "
        "e = −offset [ns]; absolute output in ppb; no dt, no saturation.",
        "Porting fedele del PI del firmware (precision_pi_update): integrale += ki·e; u = kp·e + integrale; "
        "e = −offset [ns]; uscita assoluta in ppb; nessun dt, nessuna saturazione."),
    "cn_pi_time_aware": _s(
        "Experimental PI with explicit sampling time: kp = 2ζ·wn, ki = wn² per second, measured dt, "
        "bandwidth guard wn·dt ≤ wn_ts_max, output clamp and anti-windup.",
        "PI sperimentale con tempo di campionamento esplicito: kp = 2ζ·wn, ki = wn² al secondo, dt misurato, "
        "guardia wn·dt ≤ wn_ts_max, limite sull'uscita e anti-windup."),
}
