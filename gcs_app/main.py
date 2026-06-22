"""Entry point for the Ground Control Station application."""

from __future__ import annotations

import os
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from gcs_app.qt_compat import QApplication
from gcs_app.ui.pages.main_window import MainWindow


def main():
    """Entry point for GCS application."""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
