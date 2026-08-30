from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Union


TELEMETRY_CHANNELS = (
    "ua", "ub", "uc",
    "ia_shifted", "ib_shifted", "ic_shifted",
    "iq", "id",
    "vf_angle", "hfi_angle", "smo_angle", "flux_angle",
    "abz_angle", "abs_angle",
    "vf_speed", "hfi_speed", "smo_speed", "flux_speed",
    "abz_speed", "abs_speed",
    "uq", "ud", "iq_ref", "id_ref",
    "target_elec_speed", "elec_speed", "voltage_scale", "motor_mode",
    "speed_ref", "vbus", "target_position", "position_ref",
    "position_feedback", "uq_feedforward", "ud_feedforward",
    "feedforward_mode",
)
TELEMETRY_TAIL = b"\x00\x00\x80\x7f"
TELEMETRY_PAYLOAD_SIZE = 4 * len(TELEMETRY_CHANNELS)

SCOPE_MAGIC = b"SCP1"
SCOPE_HEADER_FORMAT = "<IHH8I"
SCOPE_HEADER_SIZE = struct.calcsize(SCOPE_HEADER_FORMAT)
SCOPE_CHANNELS = (
    "ia", "ib", "iq", "id", "iq_ref", "id_ref",
    "uq", "ud", "elec_angle", "elec_speed", "speed_ref",
    "voltage_scale", "hfi_phase_error", "hfi_d_response",
    "hfi_q_response",
)

ASCII_PREFIXES = (
    b"STATUS,", b"FAULT,", b"FAULT_CLEAR,", b"FAULT_EVENT,", b"COMMAND,",
    b"FAULT_OVERCURRENT,", b"IDENT_", b"Writing ", b"Calibration ",
    b"Encoder ", b"ABZ ",
)


def _fields(line: str) -> tuple[str, dict[str, str], tuple[str, ...]]:
    parts = tuple(part.strip() for part in line.strip().split(","))
    values: dict[str, str] = {}
    flags = []
    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            values[key] = value
        elif part:
            flags.append(part)
    return parts[0], values, tuple(flags)


def _integer(value: str, default: int = 0) -> int:
    try:
        return int(value, 0)
    except (TypeError, ValueError):
        return default


def _number(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class TelemetryFrame:
    values: tuple[float, ...]

    def __getitem__(self, item: int | str) -> float:
        if isinstance(item, str):
            item = TELEMETRY_CHANNELS.index(item)
        return self.values[item]

    @property
    def channels(self) -> dict[str, float]:
        return dict(zip(TELEMETRY_CHANNELS, self.values))


@dataclass(frozen=True)
class StatusReport:
    lifecycle: str
    operation: str
    source: str
    diagnostic: str
    mode: int
    fault_active: int
    fault_latched: int

    @classmethod
    def parse(cls, line: str) -> "StatusReport":
        kind, values, _ = _fields(line)
        if kind != "STATUS":
            raise ValueError("not a STATUS response")
        return cls(
            lifecycle=values.get("Lifecycle", "UNKNOWN"),
            operation=values.get("Operation", "UNKNOWN"),
            source=values.get("Source", "UNKNOWN"),
            diagnostic=values.get("Diagnostic", "UNKNOWN"),
            mode=_integer(values.get("Mode", "0")),
            fault_active=_integer(values.get("FaultActive", "0")),
            fault_latched=_integer(values.get("FaultLatched", "0")),
        )


@dataclass(frozen=True)
class FaultReport:
    active: int
    latched: int
    first: str
    timestamp_ms: int
    lifecycle: int
    operation: int
    source: int
    iq: float
    id: float
    speed: float
    target: float
    vbus: float
    voltage_scale: float

    @classmethod
    def parse(cls, line: str) -> "FaultReport":
        kind, values, _ = _fields(line)
        if kind != "FAULT":
            raise ValueError("not a FAULT response")
        return cls(
            active=_integer(values.get("Active", "0")),
            latched=_integer(values.get("Latched", "0")),
            first=values.get("First", "NONE"),
            timestamp_ms=_integer(values.get("TimeMs", "0")),
            lifecycle=_integer(values.get("Lifecycle", "0")),
            operation=_integer(values.get("Operation", "0")),
            source=_integer(values.get("Source", "0")),
            iq=_number(values.get("Iq", "0")),
            id=_number(values.get("Id", "0")),
            speed=_number(values.get("Speed", "0")),
            target=_number(values.get("Target", "0")),
            vbus=_number(values.get("Vbus", "0")),
            voltage_scale=_number(values.get("Scale", "0")),
        )


@dataclass(frozen=True)
class CommandResult:
    name: str
    accepted: bool
    reason: str

    @classmethod
    def parse(cls, line: str) -> "CommandResult":
        kind, values, _ = _fields(line)
        if kind != "COMMAND":
            raise ValueError("not a COMMAND response")
        return cls(
            name=values.get("Name", "UNKNOWN"),
            accepted=values.get("Result", "REJECT") == "OK",
            reason=values.get("Reason", "UNKNOWN"),
        )


@dataclass(frozen=True)
class LineEvent:
    line: str


@dataclass(frozen=True)
class ScopeCapture:
    version: int
    channel_count: int
    sample_count: int
    divider: int
    base_sample_hz: int
    max_capture_cycles: int
    cpu_hz: int
    samples: tuple[tuple[float, ...], ...]

    @property
    def sample_period(self) -> float:
        return self.divider / float(self.base_sample_hz)


ProtocolEvent = Union[TelemetryFrame, StatusReport, FaultReport, CommandResult,
                      LineEvent, ScopeCapture]


class StreamParser:
    """Incrementally parses the mixed telemetry, ASCII and Scope stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.pending: list[ProtocolEvent] = []

    def feed(self, data: bytes) -> list[ProtocolEvent]:
        self.buffer.extend(data)
        events: list[ProtocolEvent] = []
        while self.buffer:
            event = self._next_event()
            if event is None:
                break
            if event is not False:
                events.append(event)
        return events

    def _next_event(self) -> ProtocolEvent | bool | None:
        if self.pending:
            return self.pending.pop(0)
        if self.buffer.startswith(SCOPE_MAGIC):
            if len(self.buffer) < SCOPE_HEADER_SIZE:
                return None
            fields = struct.unpack_from(SCOPE_HEADER_FORMAT, self.buffer)
            (magic, version, channel_count, sample_count, divider,
             base_sample_hz, sample_size, data_bytes, max_cycles,
             cpu_hz, _) = fields
            if magic != int.from_bytes(SCOPE_MAGIC, "little") or version != 1:
                del self.buffer[0]
                return False
            if sample_size != channel_count * 4 or data_bytes != sample_count * sample_size:
                del self.buffer[:SCOPE_HEADER_SIZE]
                return LineEvent("SCOPE_ERROR,InvalidSize")
            total = SCOPE_HEADER_SIZE + data_bytes
            if len(self.buffer) < total:
                return None
            payload = self.buffer[SCOPE_HEADER_SIZE:total]
            samples = tuple(struct.iter_unpack(f"<{channel_count}f", payload))
            del self.buffer[:total]
            return ScopeCapture(
                version, channel_count, sample_count, divider,
                base_sample_hz, max_cycles, cpu_hz, samples,
            )

        if self.buffer.startswith(ASCII_PREFIXES):
            end = self.buffer.find(b"\n")
            if end < 0:
                return None
            raw = bytes(self.buffer[:end + 1])
            del self.buffer[:end + 1]
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("STATUS,"):
                return StatusReport.parse(line)
            if line.startswith("FAULT,"):
                return FaultReport.parse(line)
            if line.startswith("COMMAND,"):
                return CommandResult.parse(line)
            return LineEvent(line)

        tail = self.buffer.find(TELEMETRY_TAIL)
        if tail >= 0:
            end = tail + len(TELEMETRY_TAIL)
            if tail < TELEMETRY_PAYLOAD_SIZE:
                del self.buffer[:end]
                return False
            start = tail - TELEMETRY_PAYLOAD_SIZE
            if start > 0:
                prefix = bytes(self.buffer[:start])
                del self.buffer[:start]
                for raw_line in prefix.splitlines():
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("STATUS,"):
                        self.pending.append(StatusReport.parse(line))
                    elif line.startswith("FAULT,"):
                        self.pending.append(FaultReport.parse(line))
                    elif line.startswith("COMMAND,"):
                        self.pending.append(CommandResult.parse(line))
                    else:
                        self.pending.append(LineEvent(line))
                return self.pending.pop(0) if self.pending else False
            values = struct.unpack_from(
                f"<{len(TELEMETRY_CHANNELS)}f", self.buffer
            )
            del self.buffer[:end]
            if all(math.isfinite(value) for value in values):
                return TelemetryFrame(values)
            return False

        if len(self.buffer) > TELEMETRY_PAYLOAD_SIZE + 4096:
            del self.buffer[:-(TELEMETRY_PAYLOAD_SIZE + len(TELEMETRY_TAIL))]
            return False
        return None
