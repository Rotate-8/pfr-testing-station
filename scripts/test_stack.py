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

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from start_menu import ZENOH_AUTOMATION_SCRIPT

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
            ['source', 'setup/install.bash'],
            check=True,
            capture_output=True,
            text=True,
            cwd=SOFTWARE_REPO
        )

        # 2) turn off zenoh so it can be launched by setup script
        print("Turning of zenoh endpoint")
        subprocess.run(
        ['sudo', '/usr/bin/systemctl', 'stop', 'zenohd.service'],
        check=True,
        capture_output=True,
        text=True)

        # 3) launch bring-up in background (no output)
        print("Running ROS2 bring-up in background...")
        subprocess.Popen([
            'bash', '-i', '-c',
            f'exec {BRINGUP_CMD}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 4) prompt user
        input("\nBring-up launched in the background.  Please click the reset button on the microcontroller then press Enter to continue.")

        # 5) run teleop in foreground
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

        # 6) prompt user as to whether they want to reset motor controller settings
        if input("\nWould you like to change motor controller settings back to standard? (y/n): ").lower().strip() == "y":
            subprocess.run(
            ['python3', ZENOH_AUTOMATION_SCRIPT, '--mode=zenoh', '--reset_settings'],
            check=True,
            capture_output=False,
            )
            instant_exit = True # user will already be prompted to return

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR: {e}{COLOR_RESET}")
    finally:
        subprocess.run(
        ['sudo', '/usr/bin/systemctl', 'start', 'zenohd.service'],
        check=True,
        capture_output=True,
        text=True)
        if not instant_exit:
            input("\nPress enter to return to main menu: ")
        sys.exit()

if __name__ == '__main__':
    main()