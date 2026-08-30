from __future__ import annotations

from collections.abc import Iterable
import time

import serial
from serial.tools import list_ports

from .protocol import ProtocolEvent, StreamParser


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


class FocDevice:
    """Small non-blocking serial client shared by scripts and the GUI."""

    def __init__(self, baudrate: int = 115200) -> None:
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self.parser = StreamParser()

    @property
    def connected(self) -> bool:
        return self.serial is not None and self.serial.is_open

    def open(self, port: str) -> None:
        self.close(send_stop=True)
        self.serial = serial.Serial(
            port, self.baudrate, timeout=0, write_timeout=0.2
        )
        self.serial.reset_input_buffer()
        self.parser = StreamParser()

    def close(self, send_stop: bool = True) -> None:
        if self.connected:
            assert self.serial is not None
            connection = self.serial
            try:
                if send_stop:
                    self.command("STOP")
            finally:
                connection.close()
                self.serial = None
        else:
            self.serial = None

    def command(self, command: str) -> None:
        if not self.connected:
            raise RuntimeError("device is not connected")
        assert self.serial is not None
        self.serial.write((command.rstrip() + "\r\n").encode("ascii"))
        self.serial.flush()

    def commands(self, commands: Iterable[str]) -> None:
        sequence = tuple(commands)
        for index, command in enumerate(sequence):
            self.command(command)
            if index + 1 < len(sequence):
                time.sleep(0.02)

    def poll(self) -> list[ProtocolEvent]:
        if not self.connected:
            return []
        assert self.serial is not None
        available = self.serial.in_waiting
        return self.parser.feed(self.serial.read(available or 1))

    def request_status(self) -> None:
        self.command("STATUS")

    def request_fault(self) -> None:
        self.command("FAULT")

    def clear_fault(self) -> None:
        self.command("FAULT CLEAR")

    def arm_scope(self, divider: int) -> None:
        if divider not in (1, 10, 100):
            raise ValueError("scope divider must be 1, 10 or 100")
        self.command(f"SCOPE ARM {divider}")

    def dump_scope(self) -> None:
        self.command("SCOPE DUMP")
