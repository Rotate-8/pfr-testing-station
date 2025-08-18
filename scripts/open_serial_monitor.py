import serial
import time
import json
import subprocess
from serial.tools import list_ports
import sys
import os
from absl import flags

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from start_menu import CLI_AUTOMATION_SCRIPT, TEST_STACK_SCRIPT


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
#   │ get_tmux_info               │
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

flags.DEFINE_bool(
    "currently_testing",
    default=False,
    help="Whether the user is currently testing the motor controller. Default is False."
)   

BRAIN_BOARD_IDENTIFIER = "FT232R"

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

BLUETOOTH_FILE_PATH = f"bluetooth_connected.txt"
CONNECTION_TEST_COMMAND = "pong"
WIFI_SETUP_MSG = "enter \'wifi-connect\'"
MAC_ADDR_MSG = "mac addr:"


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


def get_tmux_info():
    """
    Gets the number of panes and the current pane index from tmux.

    Returns:
        tuple: (pane_count, current_pane_index)
    """
    # Get the total number of panes in the current window
    pane_count_cmd = ['tmux', 'display-message', '-p', '#{window_panes}']
    pane_count_result = subprocess.run(pane_count_cmd, capture_output=True, text=True, check=True)
    pane_count = int(pane_count_result.stdout.strip())
    
    # Get the index of the currently active pane
    current_pane_cmd = ['tmux', 'display-message', '-p', '#{pane_index}']
    current_pane_result = subprocess.run(current_pane_cmd, capture_output=True, text=True, check=True)
    current_pane_index = int(current_pane_result.stdout.strip())

    return pane_count, current_pane_index


def write_bluetooth_status(bluetooth_connected=None, mac_addr=None, test_settings_applied=None):
    """
    Writes the Bluetooth connection status, MAC address, and test settings status to a file.

    Args:
        bluetooth_connected (int, optional): Connection status (0/1).
        mac_addr (str, optional): MAC address string.
        test_settings_applied (int, optional): Test settings status (0/1).
    """
    if bluetooth_connected is None and mac_addr is None and test_settings_applied is None:
        raise ValueError("At least one of bluetooth_connected or mac_addr must be provided.")
    with open(BLUETOOTH_FILE_PATH, "a+") as f:
        f.seek(0)
        content = f.read().strip()
        if None in [bluetooth_connected, mac_addr, test_settings_applied]:
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
            if test_settings_applied is None:
                try:
                    test_settings_applied = content.split("\n")[2].split("=")[-1]
                    test_settings_applied = int(test_settings_applied)
                except ValueError:
                    test_settings_applied = 0
                except IndexError:
                    test_settings_applied = 0
        if test_settings_applied != 0:
            print(f"Writing test settings applied status to file as {test_settings_applied}")
        f.truncate(0)
        f.write(f"connection={bluetooth_connected}\nmac_addr={mac_addr}\ntest_settings_applied={test_settings_applied}".replace("\x1b[0m", ""))


def is_tmux_pane_idle(pane_id):
    """
    Checks if the current tmux pane is running an idle shell.

    Args:
        pane_id (int): The pane index to check.
    Returns:
        bool: True if idle, False otherwise.
    """
    try:
        pane_id = int(pane_id)  # Ensure pane_id is an integer
        # Get the name of the current process in the active pane
        result = subprocess.run(
            ['tmux', 'list-panes', '-F', '#{pane_current_command}'],
            capture_output=True,
            text=True,
            check=True
        )
        current_command = result.stdout.split('\n')[pane_id].strip()
        # Check if the command is a common shell
        if current_command == 'bash':
            return True
        else:
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"Error running tmux command: {e}")
        return False


def read_bluetooth_status():
    """
    Reads the Bluetooth connection status, MAC address, and test settings status from a file.

    Returns:
        tuple: (bluetooth_connected, mac_addr, test_settings_applied)
    """
    if not os.path.isfile(BLUETOOTH_FILE_PATH):
        return 0, "", 0 

    with open(BLUETOOTH_FILE_PATH, "r") as f:
        content = f.read().replace(' ', '').split('\n')
        if len(content) != 3:
            raise ValueError(f"Invalid content in Bluetooth file: {'\n'.join(content)}. Expected format: 'connection=<value>\nmac_addr=<value>\ntest_settings_applied=<value>'")
        bluetooth_connected = int(content[0].split("=")[-1].strip())
        mac_addr = content[1].split("=")[-1].strip() if len(content) >= 1 else ""
        test_settings_applied = content[2].split("=")[-1].strip() if len(content) >= 2 else 0

    return bluetooth_connected, mac_addr, test_settings_applied

def monitor_serial_output_and_prompt_operations(port_name):
    """
    Continuously reads and prints serial output from the specified port.

    Also listens for specific messages to trigger automation scripts and test stack.

    Args:
        port_name (str): The serial port name (e.g., '/dev/ttyUSB0' or 'COMx').
    """
    try:
        bluetooth_connected = 0
        baud_rate = 115200
        # Open the serial port
        # timeout=1 makes read_until non-blocking for 1 second,
        # which allows the script to be interrupted gracefully.
        ser = serial.Serial(port_name, baudrate=baud_rate, timeout=1)
        print(f"Successfully connected to {port_name} serial monitor at {baud_rate} baud.")

        signal_listener = None
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
                    print("Writing bluetooth connected status to file as 0")
                    write_bluetooth_status(bluetooth_connected=bluetooth_connected)

                # Check for specific messages and take actions
                if WIFI_SETUP_MSG in line_lower:
                    if input("\nWould you like to automatically set up ESP32 settings and cerdentials in a new pane? (y/n): ").strip().lower() == 'y':
                        signal_name = 'BLE adjusted'
                        if "TMUX" not in os.environ:
                            session_name = "bluetooth cli session"
                            tmux_cmd = (
                                f'tmux new-session -d -s {session_name} "python3 {CLI_AUTOMATION_SCRIPT}; tmux kill-session -t {session_name}" && '
                                f'tmux attach-session -t {session_name}'
                            )
                            os.system(tmux_cmd)
                        else:
                            if not os.path.isfile(CLI_AUTOMATION_SCRIPT):
                                raise Exception(f"Bluetooth automation script named {CLI_AUTOMATION_SCRIPT} not found.")
                            tmux_cmd = f'tmux split-window -v "python3 {CLI_AUTOMATION_SCRIPT}; exit"'
                            subprocess.run(tmux_cmd, shell=True)

                # save in a file whether or not the serial monitor confirms that a bluetooth device was connected
                elif CONNECTION_TEST_COMMAND in line_lower:
                    print(f"Pong has successfully been recieved from {CLI_AUTOMATION_SCRIPT}")
                    bluetooth_connected = 1
                    write_time = time.time()
                    write_bluetooth_status(bluetooth_connected=bluetooth_connected)
                    write_time = time.time() - write_time

                elif MAC_ADDR_MSG in line_lower:
                    mac_addr = line_lower[line_lower.index(MAC_ADDR_MSG) + len(MAC_ADDR_MSG):].strip()
                    write_bluetooth_status(mac_addr=mac_addr)

            if read_bluetooth_status()[-1] == '1':
                if input("\nWould you like to test motor control stack? (y/n): ").strip().lower() == 'y':
                    if "TMUX" not in os.environ:
                        session_name = "test_motor_session"
                        tmux_cmd = (
                            f'tmux new-session -d -s {session_name} "python3 {TEST_STACK_SCRIPT}; tmux kill-session -t {session_name}" && '
                            f'tmux attach-session -t {session_name}'
                        )
                        os.system(tmux_cmd)
                    else:
                        tmux_panes, pane_index = get_tmux_info()
                        if tmux_panes == 2 and is_tmux_pane_idle((pane_index + 1) % 2):
                            tmux_cmd = (
                                f'tmux select-pane -t {(pane_index + 1) % 2} && '
                                f'"python3 {TEST_STACK_SCRIPT}"'
                            )
                        else:
                            print(f"{COLOR_YELLOW}Warning: Opening new window for test stack script.{COLOR_RESET}")
                            tmux_cmd = (
                            f'tmux split-window -h'
                            f'"python3 {TEST_STACK_SCRIPT}; exit"'
                            )
                        subprocess.run(tmux_cmd, shell=True)

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

        # This file doubles as a lock file that shows this script is running
        write_bluetooth_status(bluetooth_connected=0, mac_addr="", test_settings_applied=0)

        monitor_serial_output_and_prompt_operations(port_to_use)
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR:{e}{COLOR_RESET}")
    finally:
        os.remove(BLUETOOTH_FILE_PATH) if os.path.isfile(BLUETOOTH_FILE_PATH) else None
        if not instant_exit:
            input("\nPress enter to return to main menu: ")
        sys.exit()
