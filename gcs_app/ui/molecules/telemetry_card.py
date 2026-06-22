"""Card widget used to group related telemetry rows."""

from __future__ import annotations

from typing import Any, Mapping

from gcs_app.qt_compat import QGridLayout, QLabel, QVBoxLayout, QWidget

from gcs_app.ui.atoms.labeled_value import LabeledValue


class TelemetryCard(QWidget):
    """Molecule: Card combining labeled values and title for telemetry display."""

    def __init__(self, title: str = "", fields: list[str] | None = None, parent: Any = None):
        super().__init__(parent)
        self.title_label = QLabel(title)
        self._fields: dict[str, LabeledValue] = {}

        self._grid = QGridLayout()
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setSpacing(6)

        layout = QVBoxLayout()
        layout.addWidget(self.title_label)
        layout.addLayout(self._grid)
        self.setLayout(layout)

        if fields:
            for field_name in fields:
                self.set_value(field_name, "--")

    def set_title(self, title: str):
        self.title_label.setText(title)

    def set_value(self, field_name: str, value: Any):
        field_widget = self._fields.get(field_name)
        if field_widget is None:
            display_label = field_name.replace("_", " ").title()
            field_widget = LabeledValue(display_label, str(value))
            row_index = len(self._fields)
            column = 0 if row_index % 2 == 0 else 1
            row = row_index // 2
            self._fields[field_name] = field_widget
            self._grid.addWidget(field_widget, row, column)
        else:
            field_widget.set_value(value)

    def set_values(self, values: Mapping[str, Any]):
        for field_name, value in values.items():
            self.set_value(field_name, value)

    def values(self) -> dict[str, str]:
        return {field_name: widget.value() for field_name, widget in self._fields.items()}

