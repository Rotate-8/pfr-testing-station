"""
Opens a serial monitor for the ESP32-brain board.

This script finds the ESP32-brain board's serial port and opens a serial monitor
using PlatformIO CLI. It is intended for debugging and viewing output from the board.

Author: Paras + Vouk  •  Date: 2025-8-6
Project: Motor-Controller Station  •  Language: Python 3.12

Usage-host must have PlatformIO CLI and pyserial installed:
    $ python3 open_serial_monitor.py

** IMPORTANT*******************************************************
The code follows documentation guidelines (§SWE-061 / PEP 257
Google format) so every public function and module begins with a clear
72-char summary line, then a blank line, then extended detail.
*******************************************************************"""
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
#   │ run_command                 │
#   │ find_brain_board_port       │
#   │ write_bluetooth_status      │
#   │ is_tmux_pane_idle           │
#   │ read_bluetooth_status       │
#   │ monitor_serial_output_and_prompt_operations │
#   └───────┬─────────────────────┘
#           │
#           ▼
#   ┌─────────────────────────────┐
#   │     Main Program Flow       │
#   │ ──────────────────────────  │
#   │ __main__                    │
#   └─────────────────────────────┘
# -------------------------------------------------------------------------


import serial
import time
import json
import subprocess
from serial.tools import list_ports
import sys
import os
import socket
import signal
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from start_menu import CLI_AUTOMATION_SCRIPT

# ──────────────────────────────────────────────────────────────────────────
# Constants
# -------------------------------------------------------------------------

BRAIN_BOARD_IDENTIFIER: str = "FT232R"

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

BLUETOOTH_FILE_PATH: str = "bluetooth_connected.txt"
CONNECTION_TEST_COMMAND: str = "pong"
WIFI_SETUP_MSG: str = "enter \'wifi-connect\'"
MAC_ADDR_MSG: str = "mac addr:"
READY_MESSAGE: str = "state: ready"
TEST_STACK_LOCK_FILE: str = "test_active.txt"

ZENOH_LOCATOR: str = 'tcp'
ZENOH_PORT: str = '7447'
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
ipv4: str = s.getsockname()[0]


def run_command(command, cwd=None, capture_output=False, text=False):
    """
    Runs a shell command and handles errors.

    If capture_output is True, output is returned; otherwise, it goes to console.
    Args:
        command (list): Command and arguments to run.
        cwd (str, optional): Working directory for the command.
        capture_output (bool): Whether to capture output.
        text (bool): Whether to treat output as text.
    Returns:
        CompletedProcess: The result of subprocess.run.
    Raises:
        Exception: If the command fails or is not found.
    """
    print(f"\nExecuting command: {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=True,
            capture_output=capture_output,
            text=text
        )
        return result
    except subprocess.CalledProcessError as e:
        raise Exception(f"Command '{' '.join(e.cmd)}' failed with exit code {e.returncode}")
    except FileNotFoundError:
        raise Exception(f"Command '{command[0]}' not found. Make sure PlatformIO (pio) is installed and in your PATH.")
    except Exception as e:
        raise Exception(f"unexpected error: {e}")

def find_brain_board_port(identifier):
    """
    Finds the serial port for the ESP32-brain board using 'pio device list --json-output'.

    Args:
        identifier (str): Unique identifier for the board (e.g., 'FT232R').
    Returns:
        str: Serial port name.
    Raises:
        Exception: If no matching device is found.
    """
    try:
        result = run_command(["pio", "device", "list", "--json-output"], capture_output=True, text=True)
        devices = json.loads(result.stdout)
    except json.JSONDecodeError:
        raise Exception(f"Could not parse JSON output from 'pio device list'.")

    brain_board_port = None
    found_devices = []

    for dev in devices:
        port = dev.get("port")
        description = dev.get("description", "").lower()
        hwid = dev.get("hwid", "").lower()

        # Check if the identifier is in the description or hardware ID
        if identifier.lower() in description or identifier.lower() in hwid:
            found_devices.append(dev)

    if not found_devices:
        raise Exception(f"No ESP32-brain board found matching identifier '{identifier}'.")
    elif len(found_devices) > 1:
        print(f"{COLOR_YELLOW}Warning: Multiple devices found matching identifier '{identifier}'. Using the first one found: {found_devices[0]['port']}{COLOR_RESET}", file=sys.stderr)
        print("Consider using a more specific BRAIN_BOARD_IDENTIFIER if this is not intended.\n", file=sys.stderr)
        brain_board_port = found_devices[0]["port"]
    else:
        brain_board_port = found_devices[0]["port"]
        print(f"Successfully identified ESP32-brain board on port: {brain_board_port}\n")

    return brain_board_port


def write_bluetooth_status(bluetooth_connected=None, mac_addr=None):
    """
    Writes the Bluetooth connection status, MAC address, and test settings status to a file.

    Args:
        bluetooth_connected (int, optional): Connection status (0/1).
        mac_addr (str, optional): MAC address string.
        test_settings_applied (int, optional): Test settings status (0/1).
    """
    if bluetooth_connected is None and mac_addr is None:
        raise ValueError("At least one of bluetooth_connected or mac_addr must be provided when writing bluetooth status.")
    with open(BLUETOOTH_FILE_PATH, "a+") as f:
        f.seek(0)
        content = f.read().strip()
        if None in [bluetooth_connected, mac_addr]:
            if bluetooth_connected is None:
                bluetooth_connected = content.split("\n")[0].split("=")[-1]
                try:
                    int(bluetooth_connected)
                except ValueError:
                    bluetooth_connected = 0
            if mac_addr is None:
                split_file = content.split("\n")
                if len(split_file) >= 2:
                    mac_addr = split_file[1].split("=")[-1]
                elif len(split_file) == 1:
                    mac_addr = ''
                else:
                    raise ValueError("Invalid content in Bluetooth file. Expected format: 'connection=<value>' and 'mac_addr=<value>'")
        f.truncate(0)
        f.write(f"connection={bluetooth_connected}\nmac_addr={mac_addr}".replace("\x1b[0m", ""))


def read_bluetooth_status():
    """
    Reads the Bluetooth connection status, MAC address, and test settings status from a file.

    Returns:
        tuple: (bluetooth_connected, mac_addr)
    """
    if not os.path.isfile(BLUETOOTH_FILE_PATH):
        return 0, ""

    with open(BLUETOOTH_FILE_PATH, "r") as f:
        content = f.read().replace(' ', '').split('\n')
        if len(content) != 2:
            raise ValueError(f"Invalid content in Bluetooth file: {'\n'.join(content)}. Expected format: 'connection=<value>\nmac_addr=<value>'")
        bluetooth_connected = int(content[0].split("=")[-1].strip())
        mac_addr = content[1].split("=")[-1].strip() if len(content) >= 1 else ""

    return bluetooth_connected, mac_addr


def _is_port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False

def _launch_rmw_zenohd_background(host: str, port: int, extra_args=None):
    """
    Start `ros2 run rmw_zenoh_cpp rmw_zenohd` in the background **only if**
    nothing is already listening on host:port. Returns the Popen handle if we
    started it, or None if something is already up (or we couldn't start it).
    """
    if _is_port_open(host, port):
        print(f"Zenoh already running...")
        return None

    cmd = ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
    if extra_args:
        cmd += list(extra_args)

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # new process group (Unix)
        )
        print("Started rmw_zenohd in the background.")
        # Give the router a moment to bind its sockets
        time.sleep(2.0)
        return proc
    except FileNotFoundError:
        print("ERROR: 'ros2' not found. Ensure ROS 2 is installed and on your PATH.")
    except Exception as e:
        print(f"ERROR: failed to start rmw_zenohd: {e}")
    return None


def monitor_serial_output_and_prompt_operations(port_name):
    """
    Continuously reads and prints serial output from the specified port.

    Also listens for specific messages to trigger automation scripts and test stack.

    Args:
        port_name (str): The serial port name (e.g., '/dev/ttyUSB0' or 'COMx').
    """
    try:
        _launch_rmw_zenohd_background(ZENOH_PORT, ipv4)
        bluetooth_connected = 0
        baud_rate = 115200
        # Open the serial port
        # timeout=1 makes read_until non-blocking for 1 second,
        # which allows the script to be interrupted gracefully.
        ser = serial.Serial(port_name, baudrate=baud_rate, timeout=1)
        print(f"Successfully connected to {port_name} serial monitor at {baud_rate} baud.")

        # Clear any initial garbage data that might be in the buffer
        time.sleep(0.1)
        ser.flushInput()
       
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:  # Only print if the line is not empty
                print(line)
                line_lower = line.lower()
                # if a new line comes in say bluetooth connected is false
                if bluetooth_connected == 1:
                    bluetooth_connected = 0
                    write_bluetooth_status(bluetooth_connected=bluetooth_connected)

                # Check for specific messages and take actions
                if WIFI_SETUP_MSG in line_lower:
                    if input("\nWould you like to automatically set up ESP32 settings and cerdentials in a new pane? (y/n): ").strip().lower() == 'y':
                        if "TMUX" not in os.environ:
                            session_name = "bluetooth cli session"
                            tmux_cmd = (
                                f'tmux new-session -d -s {session_name} "python3 {CLI_AUTOMATION_SCRIPT} --board_awaiting_wifi; tmux kill-session -t {session_name}" && '
                                f'tmux attach-session -t {session_name}'
                            )
                            os.system(tmux_cmd)
                        else:
                            tmux_cmd = f'tmux split-window -v "python3 {CLI_AUTOMATION_SCRIPT} --board_awaiting_wifi; exit"'
                            subprocess.run(tmux_cmd, shell=True)

                # save in a file whether or not the serial monitor confirms that a bluetooth device was connected
                elif CONNECTION_TEST_COMMAND in line_lower:
                    bluetooth_connected = 1
                    write_time = time.time()
                    write_bluetooth_status(bluetooth_connected=bluetooth_connected)
                    write_time = time.time() - write_time

                elif MAC_ADDR_MSG in line_lower:
                    mac_addr = line_lower[line_lower.index(MAC_ADDR_MSG) + len(MAC_ADDR_MSG):].strip()
                    write_bluetooth_status(mac_addr=mac_addr)

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port_name}. {e}")
        print("Common reasons: Port is in use, incorrect port name, or no device connected.")
        print("Available ports:")
        for port in list_ports.comports():
            print(f"  {port.device} - {port.description}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    try:
        instant_exit = False

        # This file doubles as a lock file that shows this script is running
        write_bluetooth_status(bluetooth_connected=0)

        default_port = find_brain_board_port(BRAIN_BOARD_IDENTIFIER)
        
        if default_port is None:
            # If auto-detection fails, prompt the user
            port_input = input("Please enter the serial port (e.g., COM3, /dev/ttyUSB0): ").strip()
            if not port_input:
                print("No port entered. Exiting.")
                exit()
            port_to_use = port_input
        else:
            port_to_use = default_port

        monitor_serial_output_and_prompt_operations(port_to_use)
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR:{e}{COLOR_RESET}")
    finally:
        os.remove(BLUETOOTH_FILE_PATH)
        if not instant_exit:
            input("\nPress enter to return to main menu: ")
        sys.exit()
