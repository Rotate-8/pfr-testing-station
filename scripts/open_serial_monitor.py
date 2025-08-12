import serial
import time
import json
import subprocess
from serial.tools import list_ports
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from start_menu import CLI_AUTOMATION_SCRIPT, TEST_STACK_SCRIPT



# CHANGE BACK BEFORE MONDAY
BRAIN_BOARD_IDENTIFIER = "FT232R"
# BRAIN_BOARD_IDENTIFIER = "USB Single Serial"

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

BLUETOOTH_FILE_PATH = f"bluetooth_connected.txt"
CONNECTION_TEST_COMMAND = "pong"
WIFI_SETUP_MSG = "enter \'wifi-connect\'"


def run_command(command, cwd=None, capture_output=False, text=False):
    """
    Helper function to run a shell command and handle errors.
    If capture_output is True, output is returned; otherwise, it goes to console.
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
    """Gets the number of panes and the current pane index from tmux."""
    # Get the total number of panes in the current window
    pane_count_cmd = ['tmux', 'display-message', '-p', '#{window_panes}']
    pane_count_result = subprocess.run(pane_count_cmd, capture_output=True, text=True, check=True)
    pane_count = int(pane_count_result.stdout.strip())
    
    # Get the index of the currently active pane
    current_pane_cmd = ['tmux', 'display-message', '-p', '#{pane_index}']
    current_pane_result = subprocess.run(current_pane_cmd, capture_output=True, text=True, check=True)
    current_pane_index = int(current_pane_result.stdout.strip())

    return pane_count, current_pane_index


def monitor_serial_output_and_prompt_operations(port_name):
    """
    Continuously rseads and prints serial output from the specified port.

    Args:
        port_name (str): The serial port name (e.g., '/dev/ttyUSB0' or 'COMx').
        baud_rate (int): The baud rate for serial communication (e.g., 115200).
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
                if WIFI_SETUP_MSG in line.lower():
                    if input("\nWould you like to automatically set up ESP32 settings and cerdentials in a new pane? (y/n): ").strip().lower() == 'y':
                        signal_name = 'BLE adjusted'
                        if "TMUX" not in os.environ:
                            session_name = "bluetooth cli session"
                            tmux_cmd = (
                                f'tmux new-session -d -s {session_name} "python3 {CLI_AUTOMATION_SCRIPT} && tmux wait-for -S {signal_name}; tmux kill-session -t {session_name}" && '
                                f'tmux attach-session -t {session_name}'
                            )
                            os.system(tmux_cmd)
                        else:
                            if not os.path.isfile(CLI_AUTOMATION_SCRIPT):
                                raise Exception(f"Bluetooth automation script named {CLI_AUTOMATION_SCRIPT} not found.")
                            tmux_cmd = f'tmux split-window -v "python3 {CLI_AUTOMATION_SCRIPT} && tmux wait-for -S {signal_name}; exit"'
                            subprocess.run(tmux_cmd, shell=True)
                            signal_listener = subprocess.Popen(['tmux', 'wait-for', signal_name])

                # save in a file whether or not the serial monitor confirms that a bluetooth device was connected
                if CONNECTION_TEST_COMMAND in line.lower():
                    print(f"Pong has successfully been recieved from {CLI_AUTOMATION_SCRIPT}")
                    bluetooth_connected = 1
                    write_time = time.time()
                    with open(BLUETOOTH_FILE_PATH, "w") as f:
                        f.write(str(bluetooth_connected))
                        print("Wrote ble value.")
                    write_time = time.time() - write_time
                    with open(BLUETOOTH_FILE_PATH, "r") as f:
                        print(f"DEBUG: value was written at {os.path.join(os.getcwd(), BLUETOOTH_FILE_PATH)}")
                elif bluetooth_connected == 1 and line != "":
                    bluetooth_connected = 0
                    with open(BLUETOOTH_FILE_PATH, "w") as f:
                        f.write(str(bluetooth_connected))
                
                if signal_listener is not None and signal_listener.poll() is not None:
                    if input("\nWould you like to test motor control stack? (y/n): ").strip().lower() == 'y':
                        if "TMUX" not in os.environ:
                            session_name = "test_motor_session"
                            tmux_cmd = (
                                f'tmux new-session -d -s {session_name} "python3 {TEST_STACK_SCRIPT} && tmux wait-for -S {signal_name}; tmux kill-session -t {session_name}" && '
                                f'tmux attach-session -t {session_name}'
                            )
                            os.system(tmux_cmd)
                        else:
                            tmux_panes, pane_index = get_tmux_info()
                            if tmux_panes == 2 and pane_index == 1:
                                tmux_cmd = (
                                    f'tmux select-pane -t 0 '
                                    f'"python3 {TEST_STACK_SCRIPT}; exit"'
                                )
                            else:
                                print(f"{COLOR_YELLOW}Warning: Unexpected tmux window configuration, opening new window for test stack script.{COLOR_RESET}")
                                tmux_cmd = (
                                f'tmux split-window -h -t {session_name} '
                                f'"python3 {CLI_AUTOMATION_SCRIPT}; exit"'
                                )
                            subprocess.run(tmux_cmd)


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

        monitor_serial_output_and_prompt_operations(port_to_use)
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR:{e}{COLOR_RESET}")
    finally:
        if not instant_exit:
            input("\nPress enter to return to main menu: ")
        sys.exit()
