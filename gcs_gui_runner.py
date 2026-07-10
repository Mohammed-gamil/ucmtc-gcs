#!/usr/bin/env python3
"""
UCMTC Tactical Ground Control Station (GCS) - GUI Only Runner
Runs the Web GCS Server in standby/listen mode without launching
the local simulation or ROS 2 bringup nodes.
"""

import os
import sys
import time
import subprocess
import webbrowser

WORKSPACE_DIR = os.path.dirname(os.path.abspath(__file__))
VENV_PYTHON = os.path.join(WORKSPACE_DIR, ".venv", "bin", "python")
SERVER_SCRIPT = os.path.join(WORKSPACE_DIR, "web_gcs", "web_gcs_server.py")

def main():
    print("╔════════════════════════════════════════════════════════════════╗")
    print("║     UCMTC GUI Standalone Runner (Standby Mode)                 ║")
    print("╚════════════════════════════════════════════════════════════════╝\n")

    if not os.path.exists(VENV_PYTHON):
        print(f"❌ Error: Virtual environment python not found at {VENV_PYTHON}")
        sys.exit(1)

    if not os.path.exists(SERVER_SCRIPT):
        print(f"❌ Error: Web GCS server script not found at {SERVER_SCRIPT}")
        sys.exit(1)

    print("🚀 Starting GCS Web Server in standby mode (--no-sim)...")
    print("🌐 GUI will be available at: http://localhost:8082")
    print("🛑 Press Ctrl+C to stop.\n")

    cmd = [VENV_PYTHON, SERVER_SCRIPT, "--no-sim"]
    
    try:
        server_process = subprocess.Popen(cmd)
        
        # Wait briefly for server to bind port
        time.sleep(2.0)
        
        # Try to open the browser automatically
        try:
            webbrowser.open("http://localhost:8082")
        except Exception:
            pass
            
        # Keep the script running until the server exits or user interrupts
        server_process.wait()
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down GCS Web Server...")
        try:
            server_process.terminate()
            server_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            server_process.kill()
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
