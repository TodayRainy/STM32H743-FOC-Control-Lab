from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path


@dataclass(frozen=True)
class SessionMetadata:
    port: str
    baudrate: int = 115200
    firmware: str = "unknown"
    protocol: str = "debug-ascii+justfloat-v1"
    started_at: str = ""

    @classmethod
    def create(cls, port: str, baudrate: int = 115200,
               firmware: str = "unknown") -> "SessionMetadata":
        return cls(
            port=port,
            baudrate=baudrate,
            firmware=firmware,
            started_at=datetime.now(timezone.utc).isoformat(),
        )

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")
        return target
