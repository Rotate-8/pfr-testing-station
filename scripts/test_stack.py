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
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from start_menu import CLI_AUTOMATION_SCRIPT

# ──────────────────────────────────────────────────────────────────────────
# Constants & configuration
# -------------------------------------------------------------------------

SOFTWARE_REPO = '/r8/pfr-software'
BRINGUP_CMD  = 'ros2 launch pfr_launch trike-without-teleop-bringup.launch.yaml'
TELEOP_CMD   = 'ros2 run pfr_teleop pfr_teleop'
COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

# ──────────────────────────────────────────────────────────────────────────
# Main program flow
# -------------------------------------------------------------------------

def main():
    """
    Brings up the ROS2 stack and launches teleop.

    Steps:
      1. Source pfr-software
      2. Turn of zenoh endpoint on pi.
      3. Launch bring-up in background.
      4. Prompt user to reset microcontroller.
      5. Run teleop in foreground.
      6. Prompt user as to whether they want to reset motor controller settings.
    """
    try:
        instant_exit = False

        if not os.path.exists(SOFTWARE_REPO):
            raise Exception(f"Software repository not found at {SOFTWARE_REPO}. Please clone the repository first.")
        
        # 1) source setup script
        subprocess.run(
            'source install/setup.bash', # The command as a single string
            check=True,
            capture_output=False,
            text=True,
            shell=True,
            executable='/bin/bash',
            cwd=SOFTWARE_REPO
        )

        # 2) launch bring-up in background (no output)
        print("Running ROS2 bring-up in background...")
        subprocess.Popen([
            'bash', '-i', '-c',
            f'exec {BRINGUP_CMD}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 3) prompt user
        input("\nBring-up launched in the background.  Reset the motor controller if not already connected to correct zenoh:")

        # 4) run teleop in foreground
        print("Running pfr_teleop in this terminal…")
        try:
            proc = subprocess.Popen([
                'bash', '-i', '-c',
                f'exec {TELEOP_CMD}'
            ])
            proc.wait()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received. Terminating teleop process...")
            proc.terminate()
            proc.wait()

        # 5) prompt user as to whether they want to reset motor controller settings
        if input("\nWould you like to change motor controller settings back to robot default? (y/n): ").lower().strip() == "y":
            subprocess.run(
            ['python3', CLI_AUTOMATION_SCRIPT, '--mode=zenoh', '--reset_settings'],
            check=True,
            capture_output=False,
            )
            instant_exit = True # user will already be prompted to return

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR: {e}{COLOR_RESET}")
    finally:
        if not instant_exit:
            input("\nPress enter to return to main menu: ")
        sys.exit()

# ──────────────────────────────────────────────────────────────────────────
# Script entry point
# -------------------------------------------------------------------------

if __name__ == '__main__':
    main()