"""
Wrapper script for launching the motor controller CLI automation script.

This script prompts the user to select a connection mode (Zenoh or BLE) and
whether to automatically set, reset, or manually adjust motor controller settings.
It then launches the CLI automation script with the appropriate flags.

Author: Rotate-8 Team • Date: 2025-08-20
Project: PFR Testing Station • Language: Python 3.12

Usage:
    python3 open_CLI_connection_wrapper.py

** IMPORTANT *******************************************************
All public functions and modules begin with a clear summary line,
then a blank line, then extended detail (PEP 257 / Google format).
********************************************************************
"""
# Code Structure Diagram
# ────────────────────────────────────────────────────────────────────
# ┌─────────────────────────────┐
# │   Constants & Imports       │
# └─────────────┬───────────────┘
#               │
#               ▼
# ┌─────────────────────────────┐
# │   Main Program Flow         │
# │ ──────────────────────────  │
# │ main                        │
# └─────────────┬───────────────┘
#               │
#               ▼
# ┌─────────────────────────────┐
# │   Script Entry Point        │
# │ ──────────────────────────  │
# │ if __name__ == "__main__"   │
# └─────────────────────────────┘
# ────────────────────────────────────────────────────────────────────

import subprocess
import sys
from open_serial_monitor import CLI_AUTOMATION_SCRIPT

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'


def main() -> None:
    """
    Main entry point for the CLI connection wrapper.

    Prompts user for connection mode and adjustment option, then launches
    the CLI automation script with the selected flags.
    """
    try:
        instant_exit = False
        mode = input("Would you like to use [Z]enoh or [B]luetooth to open CLI: ").strip().lower()
        while mode[0] not in ("z", "b"):
            mode = input("Would you like to use [Z]enoh or [B]luetooth to open CLI: ").strip().lower()
        
        if mode == 'b':
            print("Consider resetting motor controller now in order to save its mac address for more reliable connection.")
            mode_flag = "--mode=BLE"
        else:
            mode_flag = "--mode=ZENOH"

        adjustment_option = input("\nWould you like to automatically [S]et settings to testing mode, [R]eset motor controller settings to robot standard, or [M]anually adjust: ").strip().lower()
        while adjustment_option not in ('s', 'r', 'm'):
            adjustment_option = input("Would you like to automatically [S]et settings to testing mode, [R]eset motor controller settings to robot standard, or [M]anually adjust: ").strip().lower()
        
        if adjustment_option == 's':
            adjustment_flag = "--noreset_settings"
        elif adjustment_option == 'r':
            adjustment_flag = "--reset_settings"
        else:
            adjustment_flag = "--noautomate_commands"

        subprocess.run(['python3', CLI_AUTOMATION_SCRIPT, mode_flag, adjustment_flag], capture_output=False)
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR: {e}{COLOR_RESET}")
        if not instant_exit:
            input("Press enter to return: ")
        sys.exit()

if __name__ == '__main__':
    main()