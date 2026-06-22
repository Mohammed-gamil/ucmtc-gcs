"""Safety dashboard section."""

from __future__ import annotations

from typing import Any

from gcs_app.ui.molecules.telemetry_card import TelemetryCard


class SafetyPanel(TelemetryCard):
    """Organism: Safety panel combining molecules into a functional section."""

    def __init__(self, parent: Any = None):
        super().__init__(
            title="Safety",
            fields=[
                "mode",
                "light_state",
                "estop_mech_armed",
                "estop_wire_armed",
                "estop_triggered",
                "is_blocked",
                "collision_detected",
                "border_crossed",
                "border_partial",
                "obstacle_touched",
            ],
            parent=parent,
        )

