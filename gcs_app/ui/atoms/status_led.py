"""Simple textual status indicator used for connection and safety state."""

from __future__ import annotations

from typing import Any

from gcs_app.qt_compat import QHBoxLayout, QLabel, QWidget


class StatusLED(QWidget):
    """Atom: Status LED indicator widget."""

    def __init__(self, label: str = "OFF", state: str = "idle", parent: Any = None):
        super().__init__(parent)
        self.state_label = QLabel(label)
        self.state_value = QLabel(state.upper())

        layout = QHBoxLayout()
        layout.addWidget(self.state_label)
        layout.addStretch(1)
        layout.addWidget(self.state_value)
        self.setLayout(layout)

    def set_state(self, state: str, label: str | None = None):
        self.state_value.setText(state.upper())
        if label is not None:
            self.state_label.setText(label)

    def state(self) -> str:
        return self.state_value.text()

