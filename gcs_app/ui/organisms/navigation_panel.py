"""Navigation dashboard section."""

from __future__ import annotations

from typing import Any

from gcs_app.ui.molecules.telemetry_card import TelemetryCard


class NavigationPanel(TelemetryCard):
    """Organism: Navigation panel combining molecules into a functional section."""

    def __init__(self, parent: Any = None):
        super().__init__(
            title="Navigation",
            fields=[
                "speed_kmh",
                "heading_deg",
                "pos_lat",
                "pos_lon",
                "dist_traveled_m",
                "wp_current",
                "wp_error_m",
                "wp_status",
            ],
            parent=parent,
        )

