import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")
from PySide6.QtWidgets import QApplication

from host.foc_client.protocol import StatusReport
from host.gui.main_window import MainWindow


class ConnectedSerial:
    is_open = True


def status(lifecycle, operation="NONE", active=0, latched=0):
    return StatusReport(
        lifecycle=lifecycle,
        operation=operation,
        source="ABZ",
        diagnostic="NONE",
        mode=17,
        fault_active=active,
        fault_latched=latched,
    )


def test_gui_controls_follow_runtime_state_and_defer_position_target():
    application = QApplication.instance() or QApplication([])
    window = MainWindow()
    sent = []
    window.device.serial = ConnectedSerial()
    window._command = sent.append

    window._status(status("READY"))
    assert window.start_speed_button.isEnabled()
    assert window.apply_config_button.isEnabled()
    assert not window.set_position_button.isEnabled()

    window.position_target.setValue(30.0)
    window._start_position()
    assert sent == ["START POSITION"]
    assert window.pending_target == ("POSITION_SPEED", "POSITION 30")

    window._status(status("STARTING"))
    assert sent == ["START POSITION"]
    assert window.stop_button.isEnabled()

    window._status(status("RUNNING", "POSITION_SPEED"))
    assert sent == ["START POSITION", "POSITION 30"]
    assert window.pending_target is None
    assert window.set_position_button.isEnabled()
    assert not window.start_speed_button.isEnabled()

    window._status(status("FAULT_LATCHED", active=0, latched=4))
    assert window.fault_clear_button.isEnabled()
    assert not window.stop_button.isEnabled()
    assert "BUS_UNDERVOLTAGE" in window.fault_bits_label.text()

    window.device.serial = None
    window.close()
    application.processEvents()
