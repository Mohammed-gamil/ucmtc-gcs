#!/usr/bin/env python3
"""
UCMTC Tactical Ground Control Station (GCS) Runner & Controller
An interactive Terminal User Interface (TUI) for managing GCS services,
ROS 2 nodes, workspace building, telemetry simulation, and testing.

Includes a fully automated Zero-Touch bringup sequence on startup.
Supports SSH Port Forwarding Tunnels for remote rover mesh connections.
"""

import os
import sys
import time
import signal
import socket
import curses
import threading
import subprocess
import webbrowser
from typing import Dict, List, Any, Optional

# Constants
WORKSPACE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(WORKSPACE_DIR, "build", "logs")
VENV_PYTHON = os.path.join(WORKSPACE_DIR, ".venv", "bin", "python")
ROS_SETUP_PATH = "/opt/ros/humble/setup.bash"
WORKSPACE_SETUP_PATH = os.path.join(WORKSPACE_DIR, "rover_ws", "install", "setup.bash")

# Ensure log directory exists
os.makedirs(LOG_DIR, exist_ok=True)

# Global variables for terminal sizing and control
screen_min_y = 23
screen_min_x = 90


class ProcessManager:
    """Manages spawning, tracking, and cleanly terminating subprocesses using process groups."""
    
    def __init__(self, ros_env: Dict[str, str]):
        self.ros_env = ros_env
        self.processes: Dict[str, Any] = {
            "GCS Web Server (Standby)": {
                "cmd": [VENV_PYTHON, "web_gcs/web_gcs_server.py", "--no-sim"],
                "env_type": "sim",
                "popen": None,
                "status": "STOPPED",
                "log_name": "gcs_server_standby.log",
                "desc": "Web GCS server in quiet standby mode (no mock data).",
                "ports": [8082, 8090],
                "start_time": None,
                "is_backend": True
            },
            "GCS Web Server (Sim)": {
                "cmd": [VENV_PYTHON, "web_gcs/web_gcs_server.py"],
                "env_type": "sim",
                "popen": None,
                "status": "STOPPED",
                "log_name": "gcs_server_sim.log",
                "desc": "Web GCS server with auto-generated simulation data.",
                "ports": [8082, 8090],
                "start_time": None,
                "is_backend": True
            },
            "GCS Web Server (ROS 2)": {
                "cmd": [VENV_PYTHON, "web_gcs/web_gcs_server.py"],
                "env_type": "ros2",
                "popen": None,
                "status": "STOPPED",
                "log_name": "gcs_server_ros2.log",
                "desc": "Web GCS server bridging to ROS 2 topics.",
                "ports": [8082, 8090],
                "start_time": None,
                "is_backend": True
            },
            "SSH Tunnel to Rover": {
                "cmd": [
                    "ssh", "-N", "-o", "BatchMode=yes",
                    "-L", f"{os.environ.get('GCS_SSH_LOCAL_PORT', '8091')}:localhost:{os.environ.get('GCS_SSH_REMOTE_PORT', '8090')}",
                    f"{os.environ.get('GCS_SSH_USER', 'ubuntu')}@{os.environ.get('GCS_SSH_HOST', '192.168.1.100')}"
                ],
                "env_type": "sim",
                "popen": None,
                "status": "STOPPED",
                "log_name": "ssh_tunnel.log",
                "desc": f"SSH Tunnel: Forward remote 8090 to local {os.environ.get('GCS_SSH_LOCAL_PORT', '8091')}.",
                "ports": [int(os.environ.get('GCS_SSH_LOCAL_PORT', '8091'))],
                "start_time": None
            },
            "ROS 2 Rover Bringup": {
                "cmd": ["ros2", "launch", "rover_core", "rover_bringup.launch.py", "cmd_vel_topic:=/cmd_vel_int"],
                "env_type": "ros2",
                "popen": None,
                "status": "STOPPED",
                "log_name": "rover_bringup.log",
                "desc": "Launches all 5 ROS 2 core nodes: motor, navigation, safety, vision, telemetry.",
                "ports": [],
                "start_time": None
            },
            "Mock Telemetry Pub": {
                "cmd": [VENV_PYTHON, "mock_rover_telemetry.py"],
                "env_type": "ros2",
                "popen": None,
                "status": "STOPPED",
                "log_name": "mock_telemetry.log",
                "desc": "Publishes simulated telemetry over ROS 2 topics.",
                "ports": [],
                "start_time": None
            },
            "Build Workspace": {
                "cmd": ["colcon", "build", "--symlink-install"],
                "env_type": "ros2",
                "popen": None,
                "status": "IDLE",
                "log_name": "colcon_build.log",
                "desc": "Compiles rover_ws packages using colcon.",
                "ports": [],
                "start_time": None,
                "one_shot": True
            },
            "Run Unit Tests": {
                "cmd": [VENV_PYTHON, "-m", "unittest", "tests/test_gcs_rover.py", "-v"],
                "env_type": "sim",
                "popen": None,
                "status": "IDLE",
                "log_name": "unit_tests.log",
                "desc": "Runs the unittest suite covering server, safety, and mesh logic.",
                "ports": [],
                "start_time": None,
                "one_shot": True
            }
        }
        self._lock = threading.Lock()

    def is_port_in_use(self, port: int) -> bool:
        """Checks if a TCP port is currently open/bound on localhost."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.1)
            return s.connect_ex(("127.0.0.1", port)) == 0

    def get_process_env(self, env_type: str) -> Dict[str, str]:
        """Prepares environment. Strips ROS environment variables if 'sim' mode is requested."""
        if env_type == "ros2":
            return self.ros_env
        else:
            # Simulation environment: copy system variables but remove ROS 2 variables to trigger fallback
            env = os.environ.copy()
            for key in list(env.keys()):
                if key.startswith("ROS_") or key.startswith("RMW_") or key == "AMENT_PREFIX_PATH":
                    env.pop(key, None)
            return env

    def start_process(self, name: str) -> str:
        """Spawns the requested subprocess in a separate process group to allow complete clean tear-downs."""
        with self._lock:
            p_info = self.processes.get(name)
            if not p_info:
                return "Unknown process."

            if p_info["popen"] is not None:
                return "Process is already running."

            # Check if ports are in use
            for port in p_info.get("ports", []):
                if self.is_port_in_use(port):
                    return f"Port {port} is already in use."

            # Prepare log file
            log_path = os.path.join(LOG_DIR, p_info["log_name"])
            try:
                log_file = open(log_path, "w")
                log_file.write(f"=== Starting {name} at {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n\n")
                log_file.flush()
            except Exception as e:
                return f"Failed to open log: {e}"

            # Prepare working directory
            cwd = WORKSPACE_DIR
            if name == "Build Workspace":
                cwd = os.path.join(WORKSPACE_DIR, "rover_ws")

            env = self.get_process_env(p_info["env_type"])

            try:
                p = subprocess.Popen(
                    p_info["cmd"],
                    stdout=log_file,
                    stderr=log_file,
                    cwd=cwd,
                    env=env,
                    preexec_fn=os.setsid
                )
                p_info["popen"] = p
                p_info["status"] = "RUNNING"
                p_info["start_time"] = time.time()
                return "SUCCESS"
            except Exception as e:
                p_info["status"] = "ERROR"
                log_file.write(f"CRITICAL: Failed to launch process: {e}\n")
                log_file.close()
                return f"Failed to launch: {e}"

    def stop_process(self, name: str) -> None:
        """Terminates the subprocess and all children in its process group."""
        with self._lock:
            p_info = self.processes.get(name)
            if not p_info or p_info["popen"] is None:
                return

            p = p_info["popen"]
            try:
                # Kill the entire process group (negative PID sends signal to PGID)
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                
                # Give it a brief moment to terminate cleanly
                for _ in range(10):
                    if p.poll() is not None:
                        break
                    time.sleep(0.05)
                
                # Escalate to SIGKILL if still active
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    p.wait()
            except ProcessLookupError:
                pass
            except Exception:
                pass
            finally:
                p_info["popen"] = None
                p_info["status"] = "STOPPED" if not p_info.get("one_shot") else "IDLE"
                p_info["start_time"] = None

    def update_statuses(self) -> None:
        """Checks process handles and transitions statuses if they have exited."""
        with self._lock:
            for name, p_info in self.processes.items():
                p = p_info["popen"]
                if p is not None:
                    ret = p.poll()
                    if ret is not None:
                        p_info["popen"] = None
                        p_info["start_time"] = None
                        if p_info.get("one_shot"):
                            p_info["status"] = "SUCCESS" if ret == 0 else "FAILED"
                        else:
                            p_info["status"] = "ERROR" if ret != 0 else "STOPPED"

    def stop_all(self) -> None:
        """Stops all running processes."""
        for name in list(self.processes.keys()):
            self.stop_process(name)


def load_ros2_env() -> Dict[str, str]:
    """Runs a bash subshell to source ROS 2 and local workspaces, returning the fully loaded env."""
    env = os.environ.copy()
    commands = []
    
    if os.path.exists(ROS_SETUP_PATH):
        commands.append(f"source {ROS_SETUP_PATH}")
    if os.path.exists(WORKSPACE_SETUP_PATH):
        commands.append(f"source {WORKSPACE_SETUP_PATH}")
        
    if not commands:
        return env
        
    commands.append("env")
    cmd = ["bash", "-c", " && ".join(commands)]
    
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        for line in res.stdout.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                env[k] = v
    except Exception:
        pass
        
    return env


def draw_panel_border(win, title: str, color_pair: int):
    """Draws a beautiful box around the window with a styled title."""
    win.attron(curses.color_pair(color_pair))
    win.box()
    win.attroff(curses.color_pair(color_pair))
    
    if title:
        win.addstr(0, 2, f" {title} ", curses.color_pair(color_pair) | curses.A_BOLD)


def get_last_n_log_lines(log_name: str, n: int) -> List[str]:
    """Reads the last N lines from the specified process log file."""
    log_path = os.path.join(LOG_DIR, log_name)
    if not os.path.exists(log_path):
        return ["No logs available yet. Toggle the process to begin."]
        
    try:
        with open(log_path, "r", errors="ignore") as f:
            lines = f.readlines()
            return [line.rstrip("\n") for line in lines[-n:]]
    except Exception as e:
        return [f"Failed to read logs: {e}"]


def main_tui(stdscr):
    # Setup Curses settings
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    
    # Initialize color pairs
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)    # Primary Borders & UI Theme
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)   # Active/Success Status
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)     # Stopped/Alert/Failed
    curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # Warning/Loading/Pending
    curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_BLACK)   # Base Labels & Text
    curses.init_pair(6, curses.COLOR_BLACK, curses.COLOR_CYAN)    # Selection highlight

    # Sourcing ROS 2
    stdscr.clear()
    stdscr.addstr(2, 2, "🛰️  UCMTC GROUND STATION INITIALIZATION MODULE", curses.color_pair(1) | curses.A_BOLD)
    stdscr.addstr(4, 2, "🔍 Loading local configurations & environment...")
    stdscr.refresh()
    
    ros_env = load_ros2_env()
    ros_found = "ROS_DISTRO" in ros_env
    pm = ProcessManager(ros_env)
    
    # ── Automated Bringup Sequence ──
    stdscr.addstr(6, 2, "🚀 Starting Zero-Touch bringup sequence...")
    stdscr.refresh()
    time.sleep(0.3)
    
    if ros_found:
        # Check if workspace needs to be built
        workspace_setup = os.path.join(WORKSPACE_DIR, "rover_ws", "install", "setup.bash")
        if not os.path.exists(workspace_setup):
            stdscr.addstr(8, 2, "⚠️  Workspace installation folder not found. Building workspace first...", curses.color_pair(4))
            stdscr.refresh()
            pm.start_process("Build Workspace")
            
            # Spin-wait loop showing build progress in the startup log
            while True:
                pm.update_statuses()
                p_build = pm.processes["Build Workspace"]
                if p_build["status"] in ("SUCCESS", "FAILED", "ERROR"):
                    break
                time.sleep(0.1)
                
            # Reload ROS environment after build
            ros_env = load_ros2_env()
            pm.ros_env = ros_env
            
        stdscr.addstr(10, 2, "✓ Workspace compiled/sourced.", curses.color_pair(2))
        stdscr.addstr(11, 2, "✓ Sourced ROS 2 Humble environment.", curses.color_pair(2))
        stdscr.addstr(12, 2, "⚡ Spawning Web GCS Server Bridge (ROS 2)...", curses.color_pair(1))
        stdscr.addstr(13, 2, "⚡ Spawning ROS 2 Rover Bringup...", curses.color_pair(1))
        stdscr.refresh()
        
        # Start server and rover nodes in ROS 2 mode automatically
        pm.start_process("GCS Web Server (ROS 2)")
        pm.start_process("ROS 2 Rover Bringup")
    else:
        stdscr.addstr(8, 2, "ℹ ROS 2 not detected. Loading Standby fallback mode...", curses.color_pair(4))
        stdscr.addstr(9, 2, "⚡ Spawning Web GCS Server Bridge (Standby)...", curses.color_pair(1))
        stdscr.refresh()
        # Start server in quiet standby mode
        pm.start_process("GCS Web Server (Standby)")

        
    stdscr.addstr(15, 2, "✓ Zero-touch startup complete. Launching interactive monitor...", curses.color_pair(2))
    stdscr.refresh()
    time.sleep(0.8) # Short delay to let sockets bind

    menu_items = list(pm.processes.keys())
    selected_idx = 0
    
    # Message Banner
    banner_message = "All GCS services started automatically. Press [O] to open browser dashboard."
    banner_time = time.time()
    
    def set_banner(msg: str):
        nonlocal banner_message, banner_time
        banner_message = msg
        banner_time = time.time()

    # Time tracking
    start_run_time = time.time()
    log_scroll_offset = 0

    while True:
        pm.update_statuses()
        stdscr.clear()
        
        # Get terminal size
        height, width = stdscr.getmaxyx()
        
        # Check size constraints
        if height < screen_min_y or width < screen_min_x:
            stdscr.addstr(height // 2, max(0, (width - 60) // 2), 
                         f"⚠️  TERMINAL TOO SMALL ({width}x{height}). REQUIRE >= {screen_min_x}x{screen_min_y}.", 
                         curses.color_pair(3) | curses.A_BOLD)
            stdscr.addstr(height // 2 + 1, max(0, (width - 40) // 2), 
                         "Please resize your window to proceed.", 
                         curses.color_pair(5))
            stdscr.refresh()
            time.sleep(0.2)
            ch = stdscr.getch()
            if ch in (ord('q'), ord('Q')):
                break
            continue
            
        # Draw header bar
        stdscr.attron(curses.color_pair(1) | curses.A_REVERSE)
        header_text = f" UCMTC TACTICAL GCS RUNNER & INTEGRATION CONSOLE v2.5 "
        stdscr.addstr(0, 0, header_text.ljust(width))
        stdscr.attroff(curses.color_pair(1) | curses.A_REVERSE)
        
        # Sourcing status info on the header
        status_info = f"ROS2: {'HUMBLE (OK)' if ros_found else 'NOT FOUND (FALLBACK SIM)'} | IP: {socket.gethostbyname(socket.gethostname())} "
        stdscr.addstr(0, max(0, width - len(status_info) - 2), status_info, curses.color_pair(1) | curses.A_REVERSE | curses.A_BOLD)
        
        # Define layouts
        col1_w = 42
        col2_w = width - col1_w - 2
        panels_h = height - 6
        
        # Left Panel (Processes and Actions)
        win_left = curses.newwin(panels_h, col1_w, 2, 1)
        draw_panel_border(win_left, "SERVICE CONTROL", 1)
        
        # Render Process List (Density optimized at idx * 2)
        for idx, name in enumerate(menu_items):
            p_info = pm.processes[name]
            status = p_info["status"]
            
            is_selected = (idx == selected_idx)
            bullet = "⚡" if is_selected else "  "
            
            if status in ("RUNNING", "SUCCESS"):
                color = curses.color_pair(2)
                badge = f"[{status}]"
            elif status in ("STOPPED", "FAILED", "ERROR"):
                color = curses.color_pair(3)
                badge = f"[{status}]"
            else:
                color = curses.color_pair(4)
                badge = f"[{status}]"
                
            win_left.addstr(1 + idx * 2, 2, bullet, curses.color_pair(1))
            
            name_style = curses.color_pair(6) if is_selected else curses.color_pair(5)
            if is_selected:
                win_left.addstr(1 + idx * 2, 5, name.ljust(22), name_style | curses.A_BOLD)
            else:
                win_left.addstr(1 + idx * 2, 5, name.ljust(22), name_style)
                
            win_left.addstr(1 + idx * 2, 28, badge.rjust(10), color | curses.A_BOLD)
            
            # Sub-info under process
            if p_info["popen"] is not None and p_info["start_time"] is not None:
                elapsed = int(time.time() - p_info["start_time"])
                mins, secs = divmod(elapsed, 60)
                uptime_str = f"PID: {p_info['popen'].pid} | Up: {mins:02d}m {secs:02d}s"
                win_left.addstr(2 + idx * 2, 5, uptime_str.ljust(33), curses.color_pair(5) | curses.A_DIM)
            else:
                win_left.addstr(2 + idx * 2, 5, p_info["desc"][:33].ljust(33), curses.color_pair(5) | curses.A_DIM)
                
        win_left.refresh()
        
        # Right Panel (Console Logs & Status Details)
        win_right = curses.newwin(panels_h, col2_w, 2, col1_w + 1)
        selected_log_name = menu_items[selected_idx]
        p_log_info = pm.processes[selected_log_name]
        
        draw_panel_border(win_right, f"CONSOL LOG: {selected_log_name.upper()}", 1)
        
        # Get and display log lines
        max_log_lines = panels_h - 4
        log_lines = get_last_n_log_lines(p_log_info["log_name"], max_log_lines + 50)
        
        if len(log_lines) > max_log_lines:
            display_lines = log_lines[-(max_log_lines + log_scroll_offset):len(log_lines) - log_scroll_offset]
            if log_scroll_offset > 0:
                win_right.addstr(1, col2_w - 18, " ▲ SCROLL ACTIVE ", curses.color_pair(4) | curses.A_REVERSE)
        else:
            display_lines = log_lines
            log_scroll_offset = 0
            
        for l_idx, line in enumerate(display_lines[:max_log_lines]):
            truncated = line[:col2_w - 4]
            line_color = curses.color_pair(5)
            if "error" in line.lower() or "critical" in line.lower() or "fail" in line.lower() or "❌" in line:
                line_color = curses.color_pair(3)
            elif "warning" in line.lower() or "warn" in line.lower() or "⚠️" in line:
                line_color = curses.color_pair(4)
            elif "success" in line.lower() or "✓" in line or "ok" in line.lower():
                line_color = curses.color_pair(2)
                
            win_right.addstr(2 + l_idx, 2, truncated, line_color)
            
        win_right.refresh()
        
        # Status/Banner Display
        banner_y = height - 3
        if time.time() - banner_time < 4.0 and banner_message:
            banner_bg = curses.color_pair(4) if "warning" in banner_message.lower() or "port" in banner_message.lower() or "error" in banner_message.lower() else curses.color_pair(2)
            stdscr.attron(banner_bg | curses.A_BOLD)
            stdscr.addstr(banner_y, 2, f" ▶ STATUS: {banner_message.upper()} ".ljust(width - 4))
            stdscr.attroff(banner_bg | curses.A_BOLD)
        else:
            uptime = int(time.time() - start_run_time)
            up_m, up_s = divmod(uptime, 60)
            status_line = f"Mission Elapsed Time (MET): {up_m:02d}:{up_s:02d} | Log Directory: {LOG_DIR}"
            stdscr.addstr(banner_y, 2, status_line, curses.color_pair(1) | curses.A_DIM)
            
        # Keyboard command guide
        help_y = height - 2
        help_text = "[Space/Enter] Toggle | [O] Open GUI Browser | [K] Kill All | [Q] Quit"
        stdscr.addstr(help_y, 2, help_text, curses.color_pair(1) | curses.A_BOLD)
        stdscr.refresh()
        
        # Fetch inputs
        try:
            ch = stdscr.getch()
        except Exception:
            ch = -1
            
        if ch == -1:
            time.sleep(0.05)
            continue
            
        if ch == curses.KEY_DOWN:
            selected_idx = (selected_idx + 1) % len(menu_items)
            log_scroll_offset = 0
        elif ch == curses.KEY_UP:
            selected_idx = (selected_idx - 1) % len(menu_items)
            log_scroll_offset = 0
        elif ch == curses.KEY_PPAGE:
            log_scroll_offset = min(log_scroll_offset + 5, max(0, len(log_lines) - max_log_lines))
        elif ch == curses.KEY_NPAGE:
            log_scroll_offset = max(log_scroll_offset - 5, 0)
            
        # Select and Trigger
        elif ch in (ord("\n"), ord("\r"), ord(" ")):
            selected_name = menu_items[selected_idx]
            p_info = pm.processes[selected_name]
            
            if p_info["popen"] is None:
                if p_info.get("one_shot"):
                    p_info["status"] = "PENDING"
                set_banner(f"Launching {selected_name}...")
                
                if p_info["env_type"] == "ros2" and not ros_found:
                    set_banner("Error: ROS 2 Humble not found. Cannot launch node!")
                    p_info["status"] = "ERROR"
                    continue
                    
                res = pm.start_process(selected_name)
                if res != "SUCCESS":
                    set_banner(f"Error: {res}")
                else:
                    if p_info.get("is_backend"):
                        set_banner(f"Started {selected_name}! Press [O] to open browser.")
                    else:
                        set_banner(f"Successfully started {selected_name}")
            else:
                set_banner(f"Stopping {selected_name}...")
                pm.stop_process(selected_name)
                set_banner(f"Stopped {selected_name}")
                
        # Manually open GUI Browser hotkey
        elif ch in (ord("o"), ord("O")):
            is_standby_running = pm.processes["GCS Web Server (Standby)"]["popen"] is not None
            is_sim_running = pm.processes["GCS Web Server (Sim)"]["popen"] is not None
            is_ros_running = pm.processes["GCS Web Server (ROS 2)"]["popen"] is not None
            if is_standby_running or is_sim_running or is_ros_running:
                set_banner("Opening Web GUI Browser...")
                webbrowser.open("http://localhost:8082")
            else:
                set_banner("Warning: Start a GCS Web Server first!")
                
        # Force terminate all
        elif ch in (ord("k"), ord("K")):
            set_banner("Terminating all active processes...")
            pm.stop_all()
            set_banner("All processes terminated successfully.")
            
        # Quit TUI
        elif ch in (ord("q"), ord("Q"), 27):
            set_banner("Tearing down GCS services...")
            pm.stop_all()
            break
            
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        curses.wrapper(main_tui)
        print("\n[INFO] UCMTC GCS Runner exited cleanly. All background tasks stopped.")
    except Exception as e:
        print(f"\n[CRITICAL] TUI Crashed: {e}")
        try:
            subprocess.run(["pkill", "-f", "web_gcs_server.py"])
            subprocess.run(["pkill", "-f", "rover_bringup.launch.py"])
            subprocess.run(["pkill", "-f", "mock_rover_telemetry.py"])
        except Exception:
            pass
        sys.exit(1)
