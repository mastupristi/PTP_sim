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
        "wn", "zeta", "sat_ppb", "wn_ts_max", "dt_max_s"}
    win.ctrl_combo.setCurrentText("baseline_pi")
    assert win.build_config().controller.params == {"kp": 0.7, "ki": 0.3}


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


def test_anti_windup_controller_is_selectable(win):
    win._apply_cfg_to_widgets(SimConfig.load(CONFIGS[0]))
    win.ctrl_combo.setCurrentText("pi_anti_windup")
    c = win.build_config()
    assert c.controller.name == "pi_anti_windup"
    assert c.controller.params == {"kp": 0.7, "ki": 0.3, "i_max_ppm": 0.0}
    win.ctrl_combo.setCurrentText("pi_per_second")
    assert win.build_config().controller.params == {"kp": 0.7, "ki": 0.3, "i_max_ppm": 0.0, "t_ref_s": 1.0,
                                                    "dt_max_s": 10.0}
    win.ctrl_combo.setCurrentText("baseline_pi")


def test_pi_terms_plot_draws_the_terms_and_the_active_limits(win):
    from ptpsim.engine import simulate
    from ptpsim.live import pack_result
    cfg = SimConfig.load(CONFIGS[0]).with_overrides(**{
        "duration_s": 20.0, "firmware.cmd_clamp_ppm": 1000.0, "controller.name": "pi_anti_windup",
        "controller.params": {"kp": 0.7, "ki": 0.3, "i_max_ppm": 30.0}})
    win.main_res = pack_result(simulate(cfg))
    win.chk_pi.setChecked(True)                       # toggling redraws
    x, y = win.cv["pi_i"].getData()
    assert x.size == win.main_res["pi_t"].size > 0 and np.max(np.abs(y)) <= 30.0
    assert len(win.lim_lines) == 6                    # +- actuator, +- clamp, +- integrator limit
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
    assert win.chk_zoom_y["diag"].isHidden() and win.chk_zoom_y["pi"].isHidden()
    win.chk_diag.setChecked(True)
    win.chk_pi.setChecked(True)
    assert not win.chk_zoom_y["diag"].isHidden() and not win.chk_zoom_y["pi"].isHidden()
    win.chk_zoom_y["pi"].setChecked(False)
    assert win.pw_pi.getViewBox().zoom_enabled == [True, False]
    win.chk_zoom_y["pi"].setChecked(True)
    win.chk_diag.setChecked(False)
    win.chk_pi.setChecked(False)
    assert win.chk_zoom_y["diag"].isHidden() and win.chk_zoom_y["pi"].isHidden()
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
