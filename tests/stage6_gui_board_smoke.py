from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication

from host.gui.main_window import MainWindow


def wait_until(application, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        application.processEvents()
        if predicate():
            return
        time.sleep(0.01)
    raise RuntimeError(f"timeout waiting for {description}")


def speed_mean(window: MainWindow, count: int = 50) -> float:
    values = list(window.history_speed)[-count:]
    return statistics.fmean(values) if values else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Real-board smoke test through the minimal Qt GUI path"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument(
        "--output", type=Path,
        default=Path("tests/results/stage6_gui_board_smoke_d081.json"),
    )
    args = parser.parse_args()

    application = QApplication.instance() or QApplication([])
    window = MainWindow()
    result = {
        "status": "running",
        "port": args.port,
        "speed_targets_rpm": [300.0, -300.0],
    }

    try:
        index = window.port_box.findText(args.port)
        if index < 0:
            window.port_box.addItem(args.port)
            index = window.port_box.findText(args.port)
        window.port_box.setCurrentIndex(index)
        window._toggle_connection()
        wait_until(
            application,
            lambda: window.last_status is not None
            and window.last_status.lifecycle == "READY",
            20.0, "GUI READY",
        )
        result["ready_controls"] = {
            "start_enabled": window.start_speed_button.isEnabled(),
            "configuration_enabled": window.apply_config_button.isEnabled(),
        }

        window._apply_configuration()
        wait_until(
            application,
            lambda: window.log.toPlainText().count("COMMAND ") >= 5,
            5.0, "configuration responses",
        )

        window.speed_source.setCurrentIndex(0)
        window.speed_target.setValue(300.0)
        window._start_speed()
        wait_until(
            application,
            lambda: window.last_status is not None
            and window.last_status.lifecycle == "RUNNING"
            and window.last_status.operation == "SPEED"
            and window.pending_target is None,
            20.0, "speed RUNNING",
        )
        wait_until(
            application,
            lambda: len(window.history_speed) >= 50
            and abs(speed_mean(window) - 300.0) <= 15.0,
            8.0, "+300 rpm",
        )
        result["positive_speed_mean_rpm"] = speed_mean(window)

        window._command("START SPEED")
        wait_until(
            application,
            lambda: "COMMAND START_SPEED: REJECT (NOT_READY)"
            in window.log.toPlainText(),
            3.0, "duplicate START rejection",
        )
        wait_until(
            application,
            lambda: abs(speed_mean(window) - 300.0) <= 15.0,
            2.0, "target preserved after rejected START",
        )
        result["duplicate_start"] = {
            "reason": "NOT_READY",
            "speed_mean_rpm": speed_mean(window),
        }

        window._command("FF MODEL LEGACY")
        wait_until(
            application,
            lambda: "COMMAND FF_MODEL: REJECT (NOT_READY)"
            in window.log.toPlainText(),
            3.0, "structured command rejection",
        )
        result["running_reject_reason"] = "NOT_READY"

        window.speed_target.setValue(-300.0)
        window.set_speed_button.click()
        wait_until(
            application,
            lambda: len(window.history_speed) >= 50
            and abs(speed_mean(window) + 300.0) <= 15.0,
            8.0, "-300 rpm",
        )
        result["negative_speed_mean_rpm"] = speed_mean(window)

        window.scope_divider.setCurrentText("10")
        window._arm_scope()
        wait_until(application, lambda: "COMMAND SCOPE_ARM: OK" in window.log.toPlainText(),
                   3.0, "scope arm acknowledgement")
        time.sleep(1.1)
        window._command("SCOPE DUMP")
        wait_until(application, lambda: window.last_scope is not None,
                   8.0, "scope download")
        result["scope"] = {
            "channels": window.last_scope.channel_count,
            "samples": window.last_scope.sample_count,
            "divider": window.last_scope.divider,
        }

        window.stop_button.click()
        wait_until(
            application,
            lambda: window.last_status is not None
            and window.last_status.lifecycle == "READY",
            5.0, "final READY",
        )
        window._commands("STATUS", "FAULT")
        time.sleep(0.1)
        application.processEvents()
        result["final"] = {
            "lifecycle": window.last_status.lifecycle,
            "operation": window.last_status.operation,
            "active_faults": window.last_status.fault_active,
            "latched_faults": window.last_status.fault_latched,
        }
        result["status"] = "passed"
    except Exception as error:
        result["status"] = "failed"
        result["error"] = str(error)
        raise
    finally:
        if window.device.connected:
            try:
                window._command("STOP")
                time.sleep(0.1)
                application.processEvents()
            finally:
                window.device.close(send_stop=False)
        window.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
