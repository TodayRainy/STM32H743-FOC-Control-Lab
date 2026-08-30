from __future__ import annotations

import math
import time
from collections import deque

import pyqtgraph as pg
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QPlainTextEdit,
)

from host.foc_client import (
    CommandResult,
    FaultReport,
    FocDevice,
    LineEvent,
    ScopeCapture,
    StatusReport,
    TelemetryFrame,
    available_ports,
)
from host.foc_client.scope import save_scope_csv


POLE_PAIRS = 10.0
RAD_S_TO_MECHANICAL_RPM = 60.0 / (2.0 * math.pi * POLE_PAIRS)
FAULT_NAMES = {
    0: "CURRENT_OFFSET",
    1: "SOFTWARE_OVERCURRENT",
    2: "BUS_UNDERVOLTAGE",
    3: "BUS_OVERVOLTAGE",
    4: "OVERSPEED",
    5: "SENSORLESS_STARTUP",
    6: "ROTOR_SOURCE_LOST",
    7: "CONTROL_DEADLINE",
    8: "SPEED_TRACKING",
    9: "POSITION_TRACKING",
}


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("STM32H743 FOC Control")
        self.resize(1180, 760)
        self.device = FocDevice()
        self.last_scope: ScopeCapture | None = None
        self.last_status: StatusReport | None = None
        self.pending_target: tuple[str, str] | None = None
        self.started = time.monotonic()
        self.history_time: deque[float] = deque(maxlen=1500)
        self.history_speed: deque[float] = deque(maxlen=1500)
        self.history_speed_ref: deque[float] = deque(maxlen=1500)
        self.history_iq: deque[float] = deque(maxlen=1500)
        self.history_id: deque[float] = deque(maxlen=1500)
        self.history_vbus: deque[float] = deque(maxlen=1500)
        self._build_ui()
        self._refresh_ports()
        self._apply_runtime_state()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll)
        self.poll_timer.start(20)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._request_status)
        self.status_timer.start(1000)

    def _build_ui(self) -> None:
        central = QWidget()
        layout = QVBoxLayout(central)
        connection = QHBoxLayout()
        self.port_box = QComboBox()
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self._refresh_ports)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle_connection)
        self.connection_label = QLabel("Disconnected")
        connection.addWidget(QLabel("Port"))
        connection.addWidget(self.port_box)
        connection.addWidget(refresh)
        connection.addWidget(self.connect_button)
        connection.addWidget(self.connection_label)
        connection.addStretch(1)
        layout.addLayout(connection)

        tabs = QTabWidget()
        tabs.addTab(self._control_tab(), "Control")
        tabs.addTab(self._telemetry_tab(), "Telemetry")
        tabs.addTab(self._scope_tab(), "Scope")
        tabs.addTab(self._fault_tab(), "Faults / Log")
        layout.addWidget(tabs)
        self.setCentralWidget(central)

    def _control_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        state = QGroupBox("Runtime state")
        state_layout = QFormLayout(state)
        self.lifecycle_label = QLabel("-")
        self.operation_label = QLabel("-")
        self.source_label = QLabel("-")
        self.diagnostic_label = QLabel("-")
        self.fault_bits_label = QLabel("0x00000000 / 0x00000000")
        state_layout.addRow("Lifecycle", self.lifecycle_label)
        state_layout.addRow("Operation", self.operation_label)
        state_layout.addRow("Rotor source", self.source_label)
        state_layout.addRow("Diagnostic", self.diagnostic_label)
        state_layout.addRow("Active / latched", self.fault_bits_label)
        layout.addWidget(state)

        commands = QGroupBox("Commands")
        grid = QGridLayout(commands)
        self.speed_source = QComboBox()
        self.speed_source.addItem("ABZ encoder", "ABZ")
        self.speed_source.addItem("HFI / Active Flux", "HFI")
        self.speed_source.addItem("I/f / Active Flux", "SENSORLESS")
        self.speed_target = self._spin(-6500.0, 6500.0, " rpm", 50.0)
        self.current_iq = self._spin(-7.0, 7.0, " A", 0.1)
        self.current_id = self._spin(-7.0, 7.0, " A", 0.1)
        self.position_target = self._spin(-36000.0, 36000.0, " deg", 10.0)
        self.start_speed_button = QPushButton("Start speed")
        self.start_speed_button.clicked.connect(self._start_speed)
        self.set_speed_button = QPushButton("Set speed")
        self.set_speed_button.clicked.connect(
            lambda: self._command(f"SPEED {self.speed_target.value():g}")
        )
        self.start_current_button = QPushButton("Start current")
        self.start_current_button.clicked.connect(self._start_current)
        self.set_current_button = QPushButton("Set current")
        self.set_current_button.clicked.connect(
            lambda: self._command(
                f"CURRENT {self.current_iq.value():g} {self.current_id.value():g}"
            )
        )
        self.start_position_button = QPushButton("Start position")
        self.start_position_button.clicked.connect(self._start_position)
        self.set_position_button = QPushButton("Set position")
        self.set_position_button.clicked.connect(
            lambda: self._command(f"POSITION {self.position_target.value():g}")
        )
        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet(
            "font-weight: bold; color: white; background: #b3261e"
        )
        self.stop_button.clicked.connect(lambda: self._command("STOP"))
        refresh_status = QPushButton("Refresh status")
        refresh_status.clicked.connect(self._request_status)
        grid.addWidget(QLabel("Speed source"), 0, 0)
        grid.addWidget(self.speed_source, 0, 1)
        grid.addWidget(QLabel("Speed"), 1, 0)
        grid.addWidget(self.speed_target, 1, 1)
        grid.addWidget(self.start_speed_button, 1, 2)
        grid.addWidget(self.set_speed_button, 1, 3)
        grid.addWidget(QLabel("Iq / Id"), 2, 0)
        grid.addWidget(self.current_iq, 2, 1)
        grid.addWidget(self.current_id, 2, 2)
        grid.addWidget(self.start_current_button, 2, 3)
        grid.addWidget(self.set_current_button, 2, 4)
        grid.addWidget(QLabel("Position"), 3, 0)
        grid.addWidget(self.position_target, 3, 1)
        grid.addWidget(self.start_position_button, 3, 2)
        grid.addWidget(self.set_position_button, 3, 3)
        grid.addWidget(self.stop_button, 4, 2)
        grid.addWidget(refresh_status, 4, 3)
        layout.addWidget(commands)

        configuration = QGroupBox("Control configuration (READY only)")
        config_grid = QGridLayout(configuration)
        self.ff_model = QComboBox()
        self.ff_model.addItems(("IDENTIFIED", "LEGACY"))
        self.ff_mode = QComboBox()
        self.ff_mode.addItems(("BOTH", "CROSS", "BEMF", "OFF"))
        self.fw_mode = QComboBox()
        self.fw_mode.addItems(("HYBRID", "FEEDBACK", "OFF"))
        self.speed_limit = self._spin(0.02, 7.0, " A", 0.1)
        self.speed_limit.setValue(4.0)
        self.voltage_limit = self._spin(0.05, 1.0, " x", 0.05)
        self.voltage_limit.setDecimals(2)
        self.voltage_limit.setValue(1.0)
        self.apply_config_button = QPushButton("Apply configuration")
        self.apply_config_button.clicked.connect(self._apply_configuration)
        config_grid.addWidget(QLabel("FF model"), 0, 0)
        config_grid.addWidget(self.ff_model, 0, 1)
        config_grid.addWidget(QLabel("FF mode"), 0, 2)
        config_grid.addWidget(self.ff_mode, 0, 3)
        config_grid.addWidget(QLabel("FW mode"), 1, 0)
        config_grid.addWidget(self.fw_mode, 1, 1)
        config_grid.addWidget(QLabel("Speed current limit"), 1, 2)
        config_grid.addWidget(self.speed_limit, 1, 3)
        config_grid.addWidget(QLabel("Voltage limit"), 2, 0)
        config_grid.addWidget(self.voltage_limit, 2, 1)
        config_grid.addWidget(self.apply_config_button, 2, 3)
        layout.addWidget(configuration)
        layout.addStretch(1)
        return page

    def _telemetry_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.speed_plot = pg.PlotWidget(title="Mechanical speed")
        self.speed_plot.addLegend()
        self.speed_curve = self.speed_plot.plot(pen="#1f77b4", name="speed rpm")
        self.speed_ref_curve = self.speed_plot.plot(pen="#ff7f0e", name="reference rpm")
        self.current_plot = pg.PlotWidget(title="dq current")
        self.current_plot.addLegend()
        self.iq_curve = self.current_plot.plot(pen="#d62728", name="Iq A")
        self.id_curve = self.current_plot.plot(pen="#2ca02c", name="Id A")
        self.vbus_plot = pg.PlotWidget(title="DC bus")
        self.vbus_curve = self.vbus_plot.plot(pen="#9467bd", name="Vbus V")
        layout.addWidget(self.speed_plot)
        layout.addWidget(self.current_plot)
        layout.addWidget(self.vbus_plot)
        return page

    def _scope_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        controls = QHBoxLayout()
        self.scope_divider = QComboBox()
        self.scope_divider.addItems(("1", "10", "100"))
        self.scope_arm_button = QPushButton("Arm")
        self.scope_arm_button.clicked.connect(self._arm_scope)
        self.scope_dump_button = QPushButton("Download")
        self.scope_dump_button.clicked.connect(lambda: self._command("SCOPE DUMP"))
        save = QPushButton("Save CSV")
        save.clicked.connect(self._save_scope)
        self.scope_label = QLabel("No capture")
        controls.addWidget(QLabel("Divider"))
        controls.addWidget(self.scope_divider)
        controls.addWidget(self.scope_arm_button)
        controls.addWidget(self.scope_dump_button)
        controls.addWidget(save)
        controls.addWidget(self.scope_label)
        controls.addStretch(1)
        self.scope_plot = pg.PlotWidget(title="Scope: Iq, Id, Iq ref, Id ref")
        self.scope_plot.addLegend()
        layout.addLayout(controls)
        layout.addWidget(self.scope_plot)
        return page

    def _fault_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        controls = QHBoxLayout()
        self.fault_read_button = QPushButton("Read fault")
        self.fault_read_button.clicked.connect(lambda: self._command("FAULT"))
        self.fault_clear_button = QPushButton("FAULT CLEAR")
        self.fault_clear_button.clicked.connect(lambda: self._command("FAULT CLEAR"))
        controls.addWidget(self.fault_read_button)
        controls.addWidget(self.fault_clear_button)
        controls.addStretch(1)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        layout.addLayout(controls)
        layout.addWidget(self.log)
        return page

    @staticmethod
    def _spin(low: float, high: float, suffix: str, step: float) -> QDoubleSpinBox:
        control = QDoubleSpinBox()
        control.setRange(low, high)
        control.setDecimals(3)
        control.setSingleStep(step)
        control.setSuffix(suffix)
        return control

    def _start_speed(self) -> None:
        target = self.speed_target.value()
        source = self.speed_source.currentData()
        if source == "SENSORLESS":
            if target == 0.0:
                self._append_log("REJECT START_SENSORLESS: target must be non-zero")
                return
            self.pending_target = None
            command = f"START SENSORLESS {target:g}"
        else:
            self.pending_target = ("SPEED", f"SPEED {target:g}")
            command = "START HFI" if source == "HFI" else "START SPEED"
        self._set_start_enabled(False)
        self._command(command)

    def _start_current(self) -> None:
        self.pending_target = (
            "CURRENT",
            f"CURRENT {self.current_iq.value():g} {self.current_id.value():g}",
        )
        self._set_start_enabled(False)
        self._command("START CURRENT")

    def _start_position(self) -> None:
        self.pending_target = (
            "POSITION_SPEED", f"POSITION {self.position_target.value():g}"
        )
        self._set_start_enabled(False)
        self._command("START POSITION")

    def _apply_configuration(self) -> None:
        self._commands(
            f"FF MODEL {self.ff_model.currentText()}",
            f"FF MODE {self.ff_mode.currentText()}",
            f"FW MODE {self.fw_mode.currentText()}",
            f"SPEED LIMIT {self.speed_limit.value():g}",
            f"VOLT LIMIT {self.voltage_limit.value():g}",
        )

    def _set_start_enabled(self, enabled: bool) -> None:
        self.start_speed_button.setEnabled(enabled)
        self.start_current_button.setEnabled(enabled)
        self.start_position_button.setEnabled(enabled)

    @staticmethod
    def _fault_names(bits: int) -> str:
        names = [name for bit, name in FAULT_NAMES.items() if bits & (1 << bit)]
        return "|".join(names) if names else "NONE"

    def _apply_runtime_state(self) -> None:
        connected = self.device.connected
        status = self.last_status
        lifecycle = status.lifecycle if status is not None else "UNKNOWN"
        operation = status.operation if status is not None else "NONE"
        ready = connected and lifecycle == "READY"
        running = connected and lifecycle == "RUNNING"
        starting = connected and lifecycle == "STARTING"

        self._set_start_enabled(ready)
        self.speed_source.setEnabled(ready)
        self.apply_config_button.setEnabled(ready)
        for control in (
            self.ff_model, self.ff_mode, self.fw_mode,
            self.speed_limit, self.voltage_limit,
        ):
            control.setEnabled(ready)

        self.set_speed_button.setEnabled(running and operation == "SPEED")
        self.set_current_button.setEnabled(running and operation == "CURRENT")
        self.set_position_button.setEnabled(
            running and operation == "POSITION_SPEED"
        )
        self.stop_button.setEnabled(ready or starting or running)
        self.scope_arm_button.setEnabled(connected)
        self.scope_dump_button.setEnabled(connected)
        self.fault_read_button.setEnabled(connected)
        self.fault_clear_button.setEnabled(
            connected and status is not None
            and status.fault_active == 0 and status.fault_latched != 0
        )

    def _send_pending_target(self, status: StatusReport) -> None:
        if self.pending_target is None or status.lifecycle != "RUNNING":
            return
        operation, command = self.pending_target
        if status.operation == operation:
            self.pending_target = None
            self._command(command)

    def _refresh_ports(self) -> None:
        selected = self.port_box.currentText()
        self.port_box.clear()
        self.port_box.addItems(available_ports())
        index = self.port_box.findText(selected)
        if index >= 0:
            self.port_box.setCurrentIndex(index)

    def _toggle_connection(self) -> None:
        try:
            if self.device.connected:
                self.device.close()
                self.last_status = None
                self.pending_target = None
                self.connect_button.setText("Connect")
                self.connection_label.setText("Disconnected")
                self._apply_runtime_state()
                return
            port = self.port_box.currentText()
            if not port:
                raise RuntimeError("no serial port selected")
            self.device.open(port)
            self.connect_button.setText("Disconnect")
            self.connection_label.setText(f"Connected: {port}")
            self.last_status = None
            self._apply_runtime_state()
            self.device.commands(("STATUS", "FAULT"))
        except Exception as error:
            QMessageBox.critical(self, "Connection", str(error))

    def _command(self, command: str) -> None:
        try:
            self.device.command(command)
            self._append_log(f"> {command}")
        except Exception as error:
            self._append_log(f"ERROR: {error}")

    def _commands(self, *commands: str) -> None:
        for index, command in enumerate(commands):
            self._command(command)
            if index + 1 < len(commands):
                time.sleep(0.02)

    def _request_status(self) -> None:
        if self.device.connected:
            self.device.request_status()

    def _arm_scope(self) -> None:
        try:
            self.device.arm_scope(int(self.scope_divider.currentText()))
            self.scope_label.setText("Armed")
        except Exception as error:
            self._append_log(f"ERROR: {error}")

    def _save_scope(self) -> None:
        if self.last_scope is None:
            self._append_log("No scope capture to save")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save scope", "scope.csv", "CSV files (*.csv)"
        )
        if path:
            save_scope_csv(path, self.last_scope)
            self._append_log(f"Saved {path}")

    def _poll(self) -> None:
        try:
            events = self.device.poll()
        except Exception as error:
            self._append_log(f"SERIAL ERROR: {error}")
            self.device.close(send_stop=False)
            self.last_status = None
            self.pending_target = None
            self.connect_button.setText("Connect")
            self.connection_label.setText("Disconnected")
            self._apply_runtime_state()
            return
        telemetry_updated = False
        for event in events:
            if isinstance(event, TelemetryFrame):
                self._telemetry(event)
                telemetry_updated = True
            elif isinstance(event, StatusReport):
                self._status(event)
            elif isinstance(event, FaultReport):
                self._append_log(
                    f"FAULT first={event.first} active=0x{event.active:08X} "
                    f"latched=0x{event.latched:08X} Vbus={event.vbus:.2f} V"
                )
            elif isinstance(event, CommandResult):
                result = "OK" if event.accepted else "REJECT"
                self._append_log(
                    f"COMMAND {event.name}: {result} ({event.reason})"
                )
                if not event.accepted and event.name.startswith("START_"):
                    self.pending_target = None
                    self._request_status()
            elif isinstance(event, ScopeCapture):
                self._scope(event)
            elif isinstance(event, LineEvent):
                self._append_log(event.line)
        if telemetry_updated:
            self._refresh_telemetry_plots()

    def _status(self, status: StatusReport) -> None:
        self.last_status = status
        self.lifecycle_label.setText(status.lifecycle)
        self.operation_label.setText(status.operation)
        self.source_label.setText(status.source)
        self.diagnostic_label.setText(status.diagnostic)
        self.fault_bits_label.setText(
            f"0x{status.fault_active:08X} {self._fault_names(status.fault_active)} / "
            f"0x{status.fault_latched:08X} {self._fault_names(status.fault_latched)}"
        )
        self._apply_runtime_state()
        self._send_pending_target(status)

    def _telemetry(self, frame: TelemetryFrame) -> None:
        timestamp = time.monotonic() - self.started
        self.history_time.append(timestamp)
        self.history_speed.append(frame["elec_speed"] * RAD_S_TO_MECHANICAL_RPM)
        self.history_speed_ref.append(frame["speed_ref"] * RAD_S_TO_MECHANICAL_RPM)
        self.history_iq.append(frame["iq"])
        self.history_id.append(frame["id"])
        self.history_vbus.append(frame["vbus"])

    def _refresh_telemetry_plots(self) -> None:
        x = list(self.history_time)
        self.speed_curve.setData(x, list(self.history_speed))
        self.speed_ref_curve.setData(x, list(self.history_speed_ref))
        self.iq_curve.setData(x, list(self.history_iq))
        self.id_curve.setData(x, list(self.history_id))
        self.vbus_curve.setData(x, list(self.history_vbus))

    def _scope(self, capture: ScopeCapture) -> None:
        self.last_scope = capture
        self.scope_label.setText(
            f"{capture.sample_count} samples @ "
            f"{capture.base_sample_hz / capture.divider:g} Hz"
        )
        self.scope_plot.clear()
        self.scope_plot.addLegend()
        x = [index * capture.sample_period for index in range(capture.sample_count)]
        for index, name, color in (
            (2, "Iq", "#d62728"), (3, "Id", "#2ca02c"),
            (4, "Iq ref", "#ff7f0e"), (5, "Id ref", "#1f77b4"),
        ):
            if index < capture.channel_count:
                self.scope_plot.plot(
                    x, [sample[index] for sample in capture.samples],
                    pen=color, name=name,
                )

    def _append_log(self, line: str) -> None:
        self.log.appendPlainText(line)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        try:
            self.device.close(send_stop=True)
        except Exception as error:
            self._append_log(f"STOP/close error: {error}")
        finally:
            event.accept()
