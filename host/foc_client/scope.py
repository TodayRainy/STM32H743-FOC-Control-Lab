from __future__ import annotations

import csv
from pathlib import Path

from .protocol import SCOPE_CHANNELS, ScopeCapture


def save_scope_csv(path: str | Path, capture: ScopeCapture) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    names = SCOPE_CHANNELS
    if capture.channel_count != len(names):
        names = tuple(f"channel_{index}" for index in range(capture.channel_count))
    with target.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", *names))
        for index, sample in enumerate(capture.samples):
            writer.writerow((index * capture.sample_period, *sample))
    return target
