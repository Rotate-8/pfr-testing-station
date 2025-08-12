"""
Opens the Bluetooth CLI for the connected motor controller.

This script launches the Bluetooth CLI automation script in a new terminal window
using tmux, allowing the user to interact with the motor controller over BLE.

Author: Paras + Vouk  •  Date: 2025-8-12
Project: Motor-Controller Station  •  Language: Python 3.12

Usage:
    $ python3 open_ble.py
"""

import subprocess
import os

# Path to the Bluetooth CLI automation script
CLI_AUTOMATION_SCRIPT = "open_CLI_and_adjust_settings.py"
SESSION_NAME = "ble_cli_session"

def main():
    # Check if the automation script exists
    script_path = os.path.join(os.path.dirname(__file__), CLI_AUTOMATION_SCRIPT)
    if not os.path.isfile(script_path):
        print(f"Error: {CLI_AUTOMATION_SCRIPT} not found in scripts directory.")
        return

    # Launch the Bluetooth CLI in a new tmux session
    tmux_cmd = [
        "tmux", "new-session", "-s", SESSION_NAME, "python3", script_path
    ]
    subprocess.run(tmux_cmd)
    print(f"Bluetooth CLI launched in tmux session '{SESSION_NAME}'.")

if __name__ == "__main__":
    main()