#!/usr/bin/env python3
"""
cd into /r8/pfr-software, source setup, launch the ROS2 bring-up in the background,
then prompt for Enter and run pfr_teleop in the foreground.
"""

import os
import sys
import subprocess

WORK_DIR     = '/r8/pfr-software'
SETUP_SCRIPT = os.path.join(WORK_DIR, 'install/setup.bash')
ZENOHBRINGUP = 'ros2 run rmw_zenoh_cpp rmw_zenohd'
BRINGUP_CMD  = 'ros2 launch pfr_launch trike-without-teleop-bringup.launch.yaml'
TELEOP_CMD   = 'ros2 run pfr_teleop pfr_teleop'

def die(msg: str):
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)

def main():
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

if __name__ == '__main__':
    main()
