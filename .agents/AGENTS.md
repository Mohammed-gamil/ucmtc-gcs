# UCMTC GS — Project Agent Rules

This file defines workspace-scoped rules for the UCMTC Ground Station project.

## Project Context

- **Project**: UCMTC Tactical Ground Control Station (GCS)
- **Stack**: Python 3, ROS 2 Humble, Flask/HTTP SSE, HTML/CSS/JS (vanilla)
- **ROS Workspace**: `/home/medochi/GS/rover_ws/`
- **Virtual Env**: `/home/medochi/GS/.venv/` — always use `.venv/bin/python` for Python execution
- **Web UI**: `/home/medochi/GS/web_gcs/` — served from `web_gcs_server.py` on port 8090

## Coding Standards

- Always source ROS 2 Humble before running ROS commands: `. /opt/ros/humble/setup.bash`
- Use `colcon build --symlink-install` when building the rover workspace
- Python files must be compatible with the `.venv` environment
- Never import `rclpy` at module level in the web server — use lazy imports inside try/except
- Use `qos_profile_sensor_data` for sensor topic subscriptions (BEST_EFFORT/VOLATILE)

## UI Design Rules

- The GCS UI uses a **war-room / military tactical** aesthetic — dark (#010306 background), neon accents
- CSS variables are defined in `styles.css` `:root` — always use them, never hardcode colors
- Font stack: Rajdhani (UI), Share Tech Mono (labels/data), Outfit (headers)
- All panel classes follow the `.panel .panel-{color}` pattern with `--panel-color` CSS variable
- Data value elements use `.dv`, `.dv.ok`, `.dv.err`, `.dv.warn` classes (not `val`, `ok-val`, `err-val`)
- Status indicator dots use `.chip-dot .d-online/.d-offline/.d-idle/.d-mesh` (not `indicator-dot`)

## File Layout

```
GS/
├── web_gcs/
│   ├── index.html          # Main UI — 3-column war-room layout
│   ├── styles.css          # Premium dark tactical CSS
│   ├── app.js              # Frontend logic + SSE + canvas HUD
│   └── web_gcs_server.py   # Python HTTP server with /events SSE endpoint
├── rover_ws/
│   └── src/rover_core/     # ROS 2 package (motor, nav, vision, safety nodes)
├── mock_rover_telemetry.py # Local telemetry simulator
├── tests/                  # Python unittest suite
└── .agents/
    ├── AGENTS.md           # This file
    └── skills/             # Project-scoped agent skills
        └── ros2-engineering-skills/
```
