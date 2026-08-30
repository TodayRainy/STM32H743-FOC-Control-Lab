from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="STM32H743 FOC host interface")
    parser.add_argument(
        "--check", action="store_true",
        help="create and close an off-screen window for software verification",
    )
    args = parser.parse_args()
    if args.check:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication
    from .main_window import MainWindow

    application = QApplication.instance() or QApplication(sys.argv)
    window = MainWindow()
    if args.check:
        window.close()
        return 0
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
