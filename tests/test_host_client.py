import math
import struct

from host.foc_client import device as device_module
from host.foc_client.device import FocDevice
from host.foc_client.protocol import (
    CommandResult,
    FaultReport,
    LineEvent,
    SCOPE_CHANNELS,
    SCOPE_HEADER_FORMAT,
    SCOPE_MAGIC,
    ScopeCapture,
    StatusReport,
    StreamParser,
    TELEMETRY_CHANNELS,
    TELEMETRY_TAIL,
    TelemetryFrame,
)


class FakeSerial:
    def __init__(self, port, baudrate, timeout, write_timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.is_open = True
        self.writes = []
        self.in_waiting = 0

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def read(self, _size):
        return b""


def telemetry_bytes(offset: float = 0.0) -> bytes:
    values = tuple(offset + index * 0.25 for index in range(len(TELEMETRY_CHANNELS)))
    return struct.pack(f"<{len(values)}f", *values) + TELEMETRY_TAIL


def test_stream_parser_handles_fragmented_telemetry_and_ascii():
    parser = StreamParser()
    stream = (
        telemetry_bytes()
        + b"STATUS,Lifecycle=READY,Operation=NONE,Source=ABZ,Diagnostic=NONE,"
          b"Mode=17,FaultActive=0x00000000,FaultLatched=0x00000002\r\n"
        + telemetry_bytes(10.0)
    )
    events = []
    for start in range(0, len(stream), 17):
        events.extend(parser.feed(stream[start:start + 17]))

    assert isinstance(events[0], TelemetryFrame)
    assert isinstance(events[1], StatusReport)
    assert isinstance(events[2], TelemetryFrame)
    assert events[1].lifecycle == "READY"
    assert events[1].mode == 17
    assert events[1].fault_latched == 2
    assert math.isclose(events[2]["iq"], 11.5)


def test_fault_response_and_event_are_distinct():
    parser = StreamParser()
    data = (
        b"FAULT_EVENT,New=0x00000004,First=BUS_UNDERVOLTAGE,TimeMs=99\r\n"
        b"FAULT,Active=0x00000004,Latched=0x00000004,"
        b"First=BUS_UNDERVOLTAGE,TimeMs=99,Lifecycle=4,Operation=5,Source=1,"
        b"Iq=0.1,Id=-0.2,Speed=20,Target=30,Vbus=17.5,Scale=1\r\n"
    )
    events = parser.feed(data)
    assert isinstance(events[0], LineEvent)
    assert isinstance(events[1], FaultReport)
    assert events[1].active == 4
    assert events[1].first == "BUS_UNDERVOLTAGE"
    assert math.isclose(events[1].vbus, 17.5)


def test_command_results_preserve_acceptance_and_reject_reason():
    parser = StreamParser()
    events = parser.feed(
        b"COMMAND,Name=START_SPEED,Result=OK,Reason=NONE\r\n"
        b"COMMAND,Name=FW_MODE,Result=REJECT,Reason=NOT_READY\r\n"
    )
    assert events == [
        CommandResult("START_SPEED", True, "NONE"),
        CommandResult("FW_MODE", False, "NOT_READY"),
    ]


def test_scope_capture_parser_waits_for_complete_payload():
    channel_count = len(SCOPE_CHANNELS)
    samples = tuple(
        tuple(sample * 10.0 + channel for channel in range(channel_count))
        for sample in range(3)
    )
    payload = b"".join(
        struct.pack(f"<{channel_count}f", *sample) for sample in samples
    )
    header = struct.pack(
        SCOPE_HEADER_FORMAT,
        int.from_bytes(SCOPE_MAGIC, "little"), 1, channel_count,
        len(samples), 10, 40000, channel_count * 4, len(payload),
        120, 480_000_000, 0,
    )

    parser = StreamParser()
    assert parser.feed(header + payload[:11]) == []
    events = parser.feed(payload[11:])
    assert len(events) == 1 and isinstance(events[0], ScopeCapture)
    assert events[0].sample_count == 3
    assert events[0].sample_period == 10 / 40000
    assert events[0].samples[2][4] == 24.0


def test_corrupt_telemetry_is_discarded_and_parser_recovers():
    parser = StreamParser()
    corrupt = bytearray(telemetry_bytes())
    corrupt[0:4] = struct.pack("<f", float("nan"))
    events = parser.feed(bytes(corrupt) + telemetry_bytes(20.0))
    assert len(events) == 1
    assert isinstance(events[0], TelemetryFrame)
    assert events[0]["vbus"] == 20.0 + 29 * 0.25


def test_boot_text_does_not_hide_a_status_before_telemetry():
    parser = StreamParser()
    events = parser.feed(
        b"\r\nDefault Uq:0.50V\r\n"
        b"STATUS,Lifecycle=READY,Operation=NONE,Source=ABZ,Diagnostic=NONE,"
        b"Mode=17,FaultActive=0x0,FaultLatched=0x0\r\n"
        + telemetry_bytes()
    )
    assert any(isinstance(event, StatusReport) for event in events)
    assert any(isinstance(event, TelemetryFrame) for event in events)


def test_reopen_stops_and_closes_the_previous_connection(monkeypatch):
    connections = []

    def open_fake(*args, **kwargs):
        connection = FakeSerial(*args, **kwargs)
        connections.append(connection)
        return connection

    monkeypatch.setattr(device_module.serial, "Serial", open_fake)
    device = FocDevice()
    device.open("COM1")
    device.open("COM2")

    assert connections[0].writes == [b"STOP\r\n"]
    assert not connections[0].is_open
    assert device.connected and device.serial is connections[1]
