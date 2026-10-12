# SPDX-License-Identifier: Apache-2.0
"""GUI configuration round-trip (offscreen Qt, no worker process)."""
import os
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
from PySide6 import QtWidgets  # noqa: E402

from ptpsim.config import SimConfig  # noqa: E402
from ptpsim.gui.app import MainWindow  # noqa: E402

CONFIGS = sorted((Path(__file__).parent.parent / "configs").glob("*.json"))


@pytest.fixture(scope="module")
def win():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = MainWindow(start_worker=False, lang="en")
    yield w
    w.close()


def test_configs_exist():
    assert len(CONFIGS) >= 10


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.stem)
def test_loading_a_config_reproduces_it_exactly(win, path):
    cfg = SimConfig.load(path)
    win._apply_cfg_to_widgets(cfg)
    assert win.build_config().to_dict() == cfg.to_dict()


def test_editing_one_widget_changes_only_its_fields(win):
    cfg = SimConfig.load(CONFIGS[0])
    cfg.tx_jitter.sync.kind, cfg.tx_jitter.sync.scale_ns = "uniform", 5000.0   # not representable by the widget
    cfg.loss.delay_req = 0.07
    win._apply_cfg_to_widgets(cfg)
    row = win.rows["lat_cmd"]
    row.spin.setValue(25.0)                       # 25 us
    out = win.build_config().to_dict()
    exp = cfg.to_dict()
    exp["latency"]["command_ns"] = 25_000.0
    assert out == exp                             # per-type jitter/loss untouched
    win.rows["loss"].spin.setValue(1.0)           # editing the collapsed control rewrites all four loss values
    assert {win.build_config().loss.sync, win.build_config().loss.delay_req} == {0.01}


def test_large_offset_and_delay_asymmetry_are_representable(win):
    cfg = SimConfig.load(CONFIGS[0])
    cfg.oscillator.initial_offset_ns = 71e6
    win._apply_cfg_to_widgets(cfg)
    assert win.rows["offset0"].config_value() == pytest.approx(71e6)
    win.rows["offset0"].spin.setValue(1_500_000.0)   # 1.5 s, step region
    assert win.build_config().oscillator.initial_offset_ns == pytest.approx(1.5e9)
    win.rows["d_asym"].spin.setValue(600.0)
    c = win.build_config()
    assert c.network.delay_ms_ns - c.network.delay_sm_ns == pytest.approx(600.0)
    assert c.network.delay_asymmetry_ns == 0.0


def test_controller_switch_builds_the_new_parameter_set(win):
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    win.ctrl_combo.setCurrentText("pi_time_aware")
    c = win.build_config()
    assert c.controller.name == "pi_time_aware" and set(c.controller.params) == {
        "wn", "zeta", "sat_ppb", "i_max_ppm", "wn_ts_max", "dt_max_s"}
    win.ctrl_combo.setCurrentText("baseline_pi")
    assert win.build_config().controller.params == {"kp": 0.7, "ki": 0.3}


def test_gain_scheduled_controller_is_selectable_with_log_sliders(win):
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    win.ctrl_combo.setCurrentText("pi_gain_scheduled")
    c = win.build_config()
    assert c.controller.name == "pi_gain_scheduled" and c.controller.params == {
        "wn_min": 0.2, "wn_max": 1.2, "e_lo_ns": 1000.0, "e_hi_ns": 100_000.0, "tau_release_s": 4.0, "zeta": 1.0,
        "sat_ppb": 400_000.0, "i_max_ppm": 0.0, "wn_ts_max": 0.35, "dt_max_s": 10.0}
    assert win.ctrl_rows["e_lo_ns"].log and win.ctrl_rows["e_hi_ns"].log and not win.ctrl_rows["wn_min"].log
    win.ctrl_rows["e_hi_ns"].spin.setValue(50_000.0)
    assert win.build_config().controller.params["e_hi_ns"] == 50_000.0
    win.ctrl_combo.setCurrentText("baseline_pi")


def test_default_gui_scenario_is_the_100us_20ppm_one():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = MainWindow(start_worker=False, lang="en")
    c = w.build_config()
    assert c.oscillator.initial_offset_ns == 100_000.0 and c.oscillator.freq_error_ppb == 20_000.0
    w.close()


def test_live_changes_before_start_trigger_a_reset(win):
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    win.mode = "live"
    win.live_state = "stopped"
    calls = []
    win._live_reset = lambda: calls.append("reset")
    win.rows["offset0"].spin.setValue(50.0)
    win._debounced()
    assert calls == ["reset"]
    win.mode = "explore"


def test_i18n_has_english_and_italian_for_every_string():
    from ptpsim.gui.i18n import STRINGS, T, set_lang
    for k, v in STRINGS.items():
        assert v.get("en") and v.get("it"), k
    set_lang("it")
    assert T("tab_run") == "Esecuzione"
    set_lang("en")
    assert T("tab_run") == "Run"


def test_language_switch_rebuilds_ui_and_keeps_config(win):
    cfg = SimConfig.load(CONFIGS[0])
    win._apply_cfg_to_widgets(cfg)
    win.rows["lat_cmd"].spin.setValue(12.0)
    before = win.build_config().to_dict()
    win.lang_combo.setCurrentIndex(win.lang_combo.findData("it"))
    assert win.tabs.tabText(0) == "Esecuzione" and win.build_config().to_dict() == before
    win.lang_combo.setCurrentIndex(win.lang_combo.findData("en"))
    assert win.tabs.tabText(0) == "Run" and win.build_config().to_dict() == before


def test_firmware_servo_widgets_round_trip_and_write_the_config(win):
    cfg = SimConfig.load(CONFIGS[0])
    cfg.firmware.cmd_clamp_ppm = 1234.5
    cfg.firmware.step_threshold_ns = 250_000_000
    win._apply_cfg_to_widgets(cfg)
    assert win.build_config().to_dict() == cfg.to_dict()
    win.rows["fw_step"].spin.setValue(0.5)            # seconds in the widget
    win.rows["fw_clamp"].spin.setValue(2000.0)
    c = win.build_config()
    assert c.firmware.step_threshold_ns == 500_000_000 and isinstance(c.firmware.step_threshold_ns, int)
    assert c.firmware.cmd_clamp_ppm == 2000.0


def test_per_second_controller_is_selectable(win):
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    win.ctrl_combo.setCurrentText("pi_per_second")
    assert win.build_config().controller.name == "pi_per_second"
    assert win.build_config().controller.params == {"kp": 0.7, "ki": 0.3, "i_max_ppm": 0.0, "t_ref_s": 1.0,
                                                    "dt_max_s": 10.0, "kp_dt_max": 1.0}
    win.ctrl_combo.setCurrentText("baseline_pi")


def test_pi_terms_plot_draws_the_terms_and_the_active_limits(win):
    from ptpsim.engine import simulate
    from ptpsim.live import pack_result
    cfg = SimConfig.load(CONFIGS[0]).with_overrides(**{
        "duration_s": 20.0, "firmware.cmd_clamp_ppm": 1000.0, "controller.name": "pi_per_second",
        "controller.params": {"kp": 0.7, "ki": 0.3, "i_max_ppm": 30.0, "t_ref_s": 0.25, "kp_dt_max": 0.0}})
    win.main_res = pack_result(simulate(cfg))
    win.chk_pi.setChecked(True)                       # toggling redraws
    x, y = win.cv["pi_i"].getData()
    assert x.size == win.main_res["pi_t"].size > 0 and np.max(np.abs(y)) <= 30.0
    assert len(win.lim_lines) == 6                    # +- actuator, +- clamp, +- integrator limit
    # the three labels are spread evenly along the lines, so they cannot overprint
    pos = sorted(ln.label.orthoPos for ln in win.lim_lines if getattr(ln, "label", None) is not None)
    assert pos == pytest.approx([1 / 6, 3 / 6, 5 / 6])
    win.chk_pi.setChecked(False)
    assert win.lim_lines == []


@pytest.mark.parametrize("lang", ["en", "it"])
def test_window_fits_a_1920_px_screen(lang):
    """The toolbar once needed ~1450 px alone and pushed the window minimum beyond 2000 px."""
    from ptpsim.gui.i18n import T
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = MainWindow(start_worker=False, lang=lang)
    w.show()
    app.processEvents()
    w.status.setText(T("st_exported", p="/a/rather/long/path/to/an/export/folder/of/results/run_0001"))
    app.processEvents()
    assert w.minimumSizeHint().width() <= 1920 - 200     # margin for a larger desktop font
    w.close()


def _wheel(pw, delta=120):
    from PySide6 import QtCore, QtGui
    vp = pw.viewport()
    pos = QtCore.QPointF(vp.width() / 2, vp.height() / 2)
    ev = QtGui.QWheelEvent(pos, vp.mapToGlobal(pos), QtCore.QPoint(0, 0), QtCore.QPoint(0, delta), QtCore.Qt.NoButton,
                           QtCore.Qt.NoModifier, QtCore.Qt.NoScrollPhase, False)
    QtWidgets.QApplication.sendEvent(vp, ev)


def test_zoom_axes_can_be_chosen_x_shared_y_per_plot(win):
    win.show()
    app = QtWidgets.QApplication.instance()
    for p in (win.p_delay, win.p_off):                # fixed ranges: no autorange follows the visible x
        p.getViewBox().disableAutoRange()
        p.getViewBox().setAutoVisible(y=False)
        p.setXRange(0, 100, padding=0)
        p.setYRange(-50, 50, padding=0)
    app.processEvents()

    def spans():
        r = [p.getViewBox().viewRange() for p in (win.p_delay, win.p_off)]
        return [(round(v[0][1] - v[0][0], 6), round(v[1][1] - v[1][0], 6)) for v in r]

    def zoom(x, yd, yo):
        win.chk_zoom_x.setChecked(x)
        win.chk_zoom_y["delay"].setChecked(yd)
        win.chk_zoom_y["off"].setChecked(yo)
        for p in (win.p_delay, win.p_off):
            p.setXRange(0, 100, padding=0)
            p.setYRange(-50, 50, padding=0)
        app.processEvents()
        _wheel(win.pw_delay)
        app.processEvents()
        return spans()

    (xd, yd), (xo, yo) = zoom(True, True, True)
    assert xd < 100 and xo == xd and yd < 100 and yo == 100          # x is linked; y only on the plot under the mouse
    (xd, yd), (xo, yo) = zoom(True, False, True)
    assert xd < 100 and xo == xd and yd == 100                        # y of this plot not zoomed
    (xd, yd), (xo, yo) = zoom(False, True, True)
    assert xd == 100 and xo == 100 and yd < 100                       # x not zoomed in any plot
    assert zoom(False, False, False) == [(100, 100), (100, 100)]
    win.chk_zoom_x.setChecked(True)
    win.chk_zoom_y["delay"].setChecked(True)
    win.chk_zoom_y["off"].setChecked(True)
    win.hide()


def test_zoom_checkboxes_follow_the_visibility_of_the_optional_plots(win):
    win.show()
    assert win.plot_boxes["diag"].isHidden() and win.plot_boxes["pi"].isHidden()
    win.chk_diag.setChecked(True)
    win.chk_pi.setChecked(True)
    assert not win.plot_boxes["diag"].isHidden() and not win.plot_boxes["pi"].isHidden()
    win.chk_zoom_y["pi"].setChecked(False)
    assert win.pw_pi.getViewBox().zoom_enabled == [True, False]
    win.chk_zoom_y["pi"].setChecked(True)
    win.chk_diag.setChecked(False)
    win.chk_pi.setChecked(False)
    assert win.plot_boxes["diag"].isHidden() and win.plot_boxes["pi"].isHidden()
    win.hide()


def test_baseline_columns_are_sized_to_their_contents_when_the_overlay_appears(win):
    """The overlay columns are hidden without overlay; a hidden column was not sized, so its text was cut."""
    from ptpsim.engine import simulate
    from ptpsim.metrics import compute_metrics
    win.show()
    cfg = SimConfig.load(CONFIGS[0]).with_overrides(duration_s=60.0, **{"oscillator.initial_offset_ns": 100e3})
    win.metrics = compute_metrics(simulate(cfg))
    win.base_metrics = None
    win._fill_table()
    assert win.table.isColumnHidden(3) and win.table.isColumnHidden(4)
    win.base_metrics = compute_metrics(simulate(cfg))
    win._fill_table()
    for j in (3, 4):
        assert not win.table.isColumnHidden(j)
        assert win.table.columnWidth(j) >= win.table.sizeHintForColumn(j)
    win.metrics = win.base_metrics = None
    win.hide()


def test_live_change_notes_are_localised_and_have_no_dict_repr():
    from ptpsim.engine import Simulation
    from ptpsim.gui import i18n
    cfg = SimConfig.load(CONFIGS[0])
    sim = Simulation(cfg)
    sim.run_until(5.0)
    sim.update_config({"controller.params.kp": 1.4}, "keep")
    note = sim.changes[-1][1]
    assert "{" not in note and note == "gains kp=1.4 (keep)"
    try:
        i18n.set_lang("it")
        assert i18n.localize_change(note) == "guadagni kp=1.4 (keep)"
        assert i18n.localize_change("something else") == "something else"   # unknown notes pass through
    finally:
        i18n.set_lang("en")


def test_delay_rate_comp_checkbox_round_trips_and_defaults_to_off(win):
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    assert not win.chk_dcomp.isChecked() and not win.build_config().firmware.delay_rate_comp
    win.chk_dcomp.setChecked(True)
    assert win.build_config().firmware.delay_rate_comp
    on = win.build_config()
    win._apply_cfg_to_widgets(on)                                          # a loaded config is shown as it is
    assert win.chk_dcomp.isChecked() and win.build_config().to_dict() == on.to_dict()
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    assert not win.chk_dcomp.isChecked()


# --------------------------------------------------------------------------- detachable plots

def _fresh_window(lang="en"):
    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = MainWindow(start_worker=False, lang=lang)
    w.show()
    return w


def test_plots_detach_into_their_own_window_and_come_back():
    w = _fresh_window()
    cfg = w.build_config().to_dict()
    assert w.plot_panel.window() is w and not w.detached_box.isVisible()
    w.btn_detach.click()
    assert w._detached and w.plot_panel.window() is w.plot_window and w.plot_window.isVisible()
    assert w.plot_panel.isVisible() and w.detached_box.isVisible()
    # the plot controls went with the plots; status and language stayed in the main window
    for ctl in (w.units, w.view_combo, w.chk_overlay, w.chk_diag, w.chk_zoom_x, w.btn_detach, w.pw_off, w.pw_delay):
        assert ctl.window() is w.plot_window
    for ctl in (w.status, w.lang_combo, w.table, w.tabs, w.btn_reattach):
        assert ctl.window() is w
    assert w.btn_detach.text() == "Reattach plots"
    assert w.build_config().to_dict() == cfg
    w.btn_reattach.click()                                   # the button left in the main window
    assert not w._detached and w.plot_panel.window() is w and not w.plot_window.isVisible()
    assert not w.detached_box.isVisible() and w.btn_detach.text() == "Detach plots"
    assert w.status.window() is w and w.lang_combo.window() is w
    w.btn_detach.click()
    w.btn_detach.click()                                     # the toolbar button toggles
    assert not w._detached and w.plot_panel.window() is w
    w.close()


def test_closing_the_plot_window_reattaches_the_plots():
    w = _fresh_window()
    w.detach_plots()
    w.plot_window.close()                                    # the X of the window
    assert not w._detached and w.plot_panel.window() is w and w.plot_panel.isVisible()
    w.close()


def test_closing_the_main_window_closes_the_plot_window():
    w = _fresh_window()
    w.detach_plots()
    pw = w.plot_window
    w.close()
    assert not pw.isVisible()


def test_language_change_keeps_the_plots_detached_and_leaves_no_orphans():
    from ptpsim.gui import i18n
    w = _fresh_window("en")
    try:
        w.detach_plots()
        old_panel = w.plot_panel
        w.lang_combo.setCurrentIndex(w.lang_combo.findData("it"))
        assert w._detached and w.plot_panel is not old_panel
        assert w.plot_panel.window() is w.plot_window and w.plot_window.isVisible()
        assert w.plot_window.layout().count() == 1            # only the new panel
        assert w.plot_window.windowTitle() == i18n.T("title_plots") and "grafici" in w.plot_window.windowTitle()
        assert w.btn_detach.text() == "Riporta i grafici qui"
        w.attach_plots()
        assert w.plot_panel.window() is w and w.btn_detach.text() == "Stacca grafici"
    finally:
        w.lang_combo.setCurrentIndex(w.lang_combo.findData("en"))   # the language is stored in QSettings: put it back
        i18n.set_lang("en")
        w.close()


# ---------------------------------------------------------------------------------------- oscilloscope window
def _scope_result(duration=30.0):
    from ptpsim.engine import Simulation
    from ptpsim.live import pack_result
    cfg = SimConfig.load(Path(__file__).parent.parent / "configs" / "default_deterministic.json")
    cfg = cfg.with_overrides(duration_s=duration)
    sim = Simulation(cfg)
    sim.run_until(duration)
    return pack_result(sim.result())


def test_scope_window_needs_an_explore_result_and_plays_it():
    from ptpsim.gui.scope_window import ScopeWindow
    _ = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    sc = ScopeWindow()
    assert not sc.slider.isEnabled() and sc.model is None                # nothing to show yet
    res = _scope_result(30.0)
    sc.set_data(res["scope_bp"], res["t_end"])
    assert sc.slider.isEnabled() and sc.n_max == 30_000 and sc.slider.maximum() == 30_000
    sc.set_position(0)
    assert "#0" in sc.info.text() and sc.slider.value() == 0
    sc.go_end()
    assert sc.pos == 30_000 and sc.slider.value() == 30_000
    sc.step(-1)
    assert sc.pos == 29_999 and sc.dir == 0
    sc.slider.setValue(1234)                                              # the user drags the slider
    assert sc.pos == 1234 and "#1234" in sc.info.text()
    # play / pause / rewind buttons
    sc.btn_play.click()
    assert sc.dir == 1 and sc.timer.isActive()
    sc.btn_play.click()
    assert sc.dir == 0 and not sc.timer.isActive()
    sc.set_position(20_000)
    sc.btn_rev.click()
    assert sc.dir == -1 and sc.btn_rev.isChecked()
    sc._last_wall -= 0.5                                                  # half a second of wall time at 10x
    sc._tick()
    assert sc.pos == pytest.approx(20_000 - 0.5 * 10 * 1000, abs=300) and sc.dir == -1
    sc.btn_start.click()
    assert sc.pos == 0 and sc.dir == 0
    # at the end: stops, or wraps with "loop"
    sc.set_position(29_999)
    sc.btn_play.click()
    sc._last_wall -= 1.0
    sc._tick()
    assert sc.dir == 0 and sc.pos == 30_000
    sc.chk_loop.setChecked(True)
    sc.set_position(29_999)
    sc.btn_play.click()
    sc._last_wall -= 1.0
    sc._tick()
    assert sc.dir == 1 and sc.pos < 30_000
    sc.set_direction(0)
    # no result (live mode): disabled and stopped, position kept for the next result
    sc.set_data(None)
    assert not sc.slider.isEnabled() and sc.dir == 0
    sc.force_close()


def test_scope_timebase_and_persistence_and_ambiguity_flag():
    from ptpsim.gui.scope_window import ScopeWindow
    _ = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    sc = ScopeWindow()
    res = _scope_result(60.0)
    sc.set_data(res["scope_bp"], res["t_end"])
    sc.set_position(1000)                                                 # t = 1 s: the 100 us initial offset is still there
    assert sc.tb_ns >= 20_000                                             # auto timebase needs to show a ~100 us edge
    sc.set_position(59_000)
    assert sc.tb_ns <= 100                                                # locked: the auto timebase has zoomed in
    fixed = sc.tb.findData(2000.0)
    sc.tb.setCurrentIndex(fixed)
    assert sc.tb_ns == 2000.0
    sc.set_position(10_000)
    sc.persist.setCurrentIndex(sc.persist.findData(1.0))                         # changing it starts from now ...
    assert sc._p_origin == 10_000 and sc.img.isVisible()
    assert sc.img.image.max() < 0.55                                             # ... the past is not there: one trace
    for k in range(10_001, 10_301, 10):
        sc.set_position(k)
    assert sc.img.image.max() > 0.9 and sc.img.image.shape[0] >= 300             # 300 sweeps overlap: saturated levels
    sc.set_position(10_300 + 2_000)                                              # 1 s later the trail has faded away
    sc.reset_persistence()                                                       # the button: forget the past again
    assert sc._p_origin == 12_300 and sc.img.image.max() < 0.55
    sc.set_position(11_000)                                                      # back before the reset point: restart there
    assert sc._p_origin == 11_000
    sc.persist.setCurrentIndex(0)
    assert not sc.img.isVisible()                                                # persistence off: just the trace
    # infinite persistence accumulates what the playback passes over since the reset
    sc.tb.setCurrentIndex(sc.tb.findData(50_000.0))
    sc.set_position(0)
    sc.persist.setCurrentIndex(sc.persist.findData(float("inf")))
    assert sc._acc is not None and sc._acc["k"] == 0 and sc._p_origin == 0
    for k in (500, 1000, 2000, 5000):
        sc.set_position(k)
    assert sc._acc["k"] == 5000 and sc._acc["total"] > 100
    total = sc._acc["total"]
    sc.tb.setCurrentIndex(sc.tb.findData(20_000.0))                              # new timebase: rebuilt from the origin
    assert sc._acc["k"] == 5000 and sc._acc["total"] > 100 and sc._p_origin == 0
    sc.set_position(4000)                                                        # backwards: restart there
    assert sc._acc["k"] == 4000 and sc._p_origin == 4000 and sc._acc["total"] <= 1.0
    sc.persist.setCurrentIndex(0)
    assert sc._acc is None
    sc.force_close()


def test_main_window_feeds_the_scope_and_follows_the_mode():
    w = _fresh_window()
    try:
        assert w.scope_win is None
        w.main_res = _scope_result(20.0)
        w.open_scope()
        assert w.scope_win.isVisible() and w.scope_win.slider.isEnabled() and w.scope_win.n_max == 20_000
        w.mode = "live"                                                   # no breakpoints for a live result
        w._push_scope()
        assert not w.scope_win.slider.isEnabled()
        w.mode = "explore"
        w._push_scope()
        assert w.scope_win.slider.isEnabled()
    finally:
        w.close()
    assert not w.scope_win.isVisible()                                    # closing the main window closes it


def test_language_change_rebuilds_the_open_scope_window():
    from ptpsim.gui import i18n
    w = _fresh_window("en")
    try:
        w.main_res = _scope_result(10.0)
        w.open_scope()
        w.lang_combo.setCurrentIndex(w.lang_combo.findData("it"))
        assert w.scope_win is not None and w.scope_win.isVisible()
        assert "oscilloscopio" in w.scope_win.windowTitle()
    finally:
        w.lang_combo.setCurrentIndex(w.lang_combo.findData("en"))
        i18n.set_lang("en")
        if w.scope_win is not None:
            w.scope_win.force_close()
        w.close()


def test_scope_auto_timebase_does_not_jump_while_playing():
    from ptpsim.gui.scope_window import ScopeWindow
    _ = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    sc = ScopeWindow()
    res = _scope_result(30.0)
    sc.set_data(res["scope_bp"], res["t_end"])
    sc.set_position(3000)
    sc.set_direction(1)
    sc.timer.stop()                                   # frames are driven by hand below
    seen = []
    for k in range(3000, 12_000, 333):                # the transient at 10x: 333 sweeps per frame, zero crossings
        sc.pos = float(k)
        sc._draw()
        seen.append(sc.tb_ns)
    assert seen == sorted(seen)                       # never zooms in within the hold time (frames take microseconds here)
    # not playing (slider): zooms in at once
    sc.set_direction(0)
    sc.set_position(12_000)
    assert sc.tb_ns < seen[-1] or seen[-1] <= 1000
    sc.force_close()


def test_language_change_keeps_the_scope_state():
    from ptpsim.gui import i18n
    w = _fresh_window("en")
    try:
        w.main_res = _scope_result(20.0)
        w.open_scope()
        sc = w.scope_win
        sc.set_position(4321)
        sc.speed.setCurrentIndex(2)
        sc.persist.setCurrentIndex(3)
        sc.chk_loop.setChecked(True)
        w.lang_combo.setCurrentIndex(w.lang_combo.findData("it"))
        sc2 = w.scope_win
        assert sc2 is not sc and sc2.pos == 4321 and sc2.speed.currentIndex() == 2
        assert sc2.persist.currentIndex() == 3 and sc2.chk_loop.isChecked()
    finally:
        w.lang_combo.setCurrentIndex(w.lang_combo.findData("en"))
        i18n.set_lang("en")
        if w.scope_win is not None:
            w.scope_win.force_close()
        w.close()
