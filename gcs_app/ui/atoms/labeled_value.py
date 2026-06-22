"""Reusable label/value row used across the telemetry dashboard."""

from __future__ import annotations

from typing import Any

from gcs_app.qt_compat import QHBoxLayout, QLabel, QWidget


class LabeledValue(QWidget):
    """Atom: Labeled value display widget."""

    def __init__(self, label: str = "", value: str = "--", parent: Any = None):
        super().__init__(parent)
        self.label_widget = QLabel(label)
        self.value_widget = QLabel(value)

        layout = QHBoxLayout()
        layout.addWidget(self.label_widget)
        layout.addStretch(1)
        layout.addWidget(self.value_widget)
        self.setLayout(layout)

    def set_label(self, label: str):
        self.label_widget.setText(label)

    def set_value(self, value: Any):
        self.value_widget.setText(str(value))

    def value(self) -> str:
        return self.value_widget.text()

