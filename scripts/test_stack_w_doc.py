"""
Brings up the ROS2 stack and launches teleop for the PFR trike.

This script changes to the workspace directory, sources the ROS2 setup script,
launches the bring-up in the background, prompts the user to reset the microcontroller,
then runs the teleop node in the foreground.

Author: Paras + Vouk  •  Date: 2025-8-6
Project: Motor-Controller Station  •  Language: Python 3.12

Usage-host must have ROS2 and bash installed:
   $ python3 test_stack_w_doc.py

** IMPORTANT*******************************************************
The code follows documentation guidelines (§SWE-061 / PEP 257
Google format) so every public function and module begins with a clear
72-char summary line, then a blank line, then extended detail.
*******************************************************************
"""
# ──────────────────────────────────────────────────────────────────────────
# Code Structure Diagram
# -------------------------------------------------------------------------
#
#   ┌───────────────┐
#   │  Constants &  │
#   │ Configuration │
#   └───────┬───────┘
#           │
#           ▼
#   ┌─────────────────────────────┐
#   │      Helper Functions       │
#   │ ──────────────────────────  │
#   │ die(msg)                    │
#   └───────┬─────────────────────┘
#           │
#           ▼
#   ┌─────────────────────────────┐
#   │     Main Program Flow       │
#   │ ──────────────────────────  │
#   │ main()                      │
#   └───────┬─────────────────────┘
#           │
#           ▼
#   ┌─────────────────────────────┐
#   │   Script Entry Point        │
#   │ ──────────────────────────  │
#   │ if __name__ == "__main__":  │
#   └─────────────────────────────┘
# -------------------------------------------------------------------------

import os
import sys
import subprocess

# ──────────────────────────────────────────────────────────────────────────
# Constants & configuration
# -------------------------------------------------------------------------

WORK_DIR     = '/r8/pfr-software'
SETUP_SCRIPT = os.path.join(WORK_DIR, 'install/setup.bash')
ZENOHBRINGUP = 'ros2 run rmw_zenoh_cpp rmw_zenohd'
BRINGUP_CMD  = 'ros2 launch pfr_launch trike-without-teleop-bringup.launch.yaml'
TELEOP_CMD   = 'ros2 run pfr_teleop pfr_teleop'

# ──────────────────────────────────────────────────────────────────────────
# Helper functions
# -------------------------------------------------------------------------

def die(msg: str):
    """Prints an error message and exits the script."""
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)

# ──────────────────────────────────────────────────────────────────────────
# Main program flow
# -------------------------------------------------------------------------

def main():
    """
    Brings up the ROS2 stack and launches teleop.

    Steps:
      1. Change to workspace directory.
      2. Verify setup script exists.
      3. Launch bring-up in background.
      4. Prompt user to reset microcontroller.
      5. Run teleop in foreground.
    """
    # 1) cd into workspace
    if not os.path.isdir(WORK_DIR):
        die(f"working directory not found: {WORK_DIR}")
    os.chdir(WORK_DIR)

    # 2) verify setup script
    if not os.path.isfile(SETUP_SCRIPT):
        die(f"setup script not found: {SETUP_SCRIPT}")

    # 3) launch bring-up in background (no output)
    print("Running ROS2 bring-up in background...")
    subprocess.Popen([
        'bash', '-i', '-c',
        f'source "{SETUP_SCRIPT}" && exec {BRINGUP_CMD}'
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 4) prompt user
    input("Bring-up launched in the background.  Please click the reset button on the microcontroller then press Enter to continue.")

    # 5) run teleop in foreground
    print("Running pfr_teleop in this terminal…")
    subprocess.run([
        'bash', '-i', '-c',
        f'source "{SETUP_SCRIPT}" && exec {TELEOP_CMD}'
    ], check=True)

# ──────────────────────────────────────────────────────────────────────────
# Script entry point
# -------------------------------------------------------------------------

if __name__ == '__main__':
    main()
