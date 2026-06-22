"""Vision and system dashboard section."""

from __future__ import annotations

from typing import Any

from gcs_app.ui.molecules.telemetry_card import TelemetryCard


class VideoDashboard(TelemetryCard):
    """Organism: Video dashboard combining molecules into a functional section."""

    def __init__(self, parent: Any = None):
        super().__init__(
            title="Vision",
            fields=[
                "img_confidence",
                "img_detected",
                "laser_active",
                "img_elapsed_sec",
                "img_task_status",
                "lane_detected",
                "obstacles_count",
                "fps_vision",
            ],
            parent=parent,
        )

