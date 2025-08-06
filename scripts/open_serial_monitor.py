import serial
import time
import json
import subprocess
from serial.tools import list_ports
import sys
import os

BRAIN_BOARD_IDENTIFIER = "FT232R"

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

BLUETOOTH_FILE_PATH = "bluetooth_connected.txt"
BLUETOOH_MSG = "[ble] device connected"
WIFI_SETUP_MSG = "enter \'wifi-connect\'"
BLE_AUTOMATION_SCRIPT = "connect_BLE.py"

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
        print(f"{COLOR_RED}Error: Command '{' '.join(e.cmd)}' failed with exit code {e.returncode}{COLOR_RESET}", file=sys.stderr)
        input("Press any key to return to menu: ")
        sys.exit(1) # Exit script on command failure
    except FileNotFoundError:
        print(f"{COLOR_RED}Error: Command '{command[0]}' not found. Make sure PlatformIO (pio) is installed and in your PATH.{COLOR_RESET}", file=sys.stderr)
        input("Press any key to return to menu: ")
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        input("Press any key to return to menu: ")
        sys.exit(1)

def find_brain_board_port(identifier):
    """
    Finds the serial port for the ESP32-brain board using 'pio device list --json-output'.
    """
    try:
        result = run_command(["pio", "device", "list", "--json-output"], capture_output=True, text=True)
        devices = json.loads(result.stdout)
    except json.JSONDecodeError:
        print(f"{COLOR_RED}Error: Could not parse JSON output from 'pio device list'.{COLOR_RESET}", file=sys.stderr)
        input("Press any key to return to menu: ")
        sys.exit(1)

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
        print(f"{COLOR_RED}Error: No ESP32-brain board found matching identifier '{identifier}'.{COLOR_RESET}", file=sys.stderr)
        print("Please ensure the board is connected and powered, and verify your BRAIN_BOARD_IDENTIFIER.", file=sys.stderr)
        print("\nAvailable devices (from 'pio device list --json-output'):")
        for dev in devices:
            print(f"Port: {dev.get('port', 'N/A')}, Description: {dev.get('description', 'N/A')}, HWID: {dev.get('hwid', 'N/A')}")
        input("Press any key to return to menu: ")
        sys.exit(1)
    elif len(found_devices) > 1:
        print(f"{COLOR_YELLOW}Warning: Multiple devices found matching identifier '{identifier}'. Using the first one found: {found_devices[0]['port']}{COLOR_RESET}", file=sys.stderr)
        print("Consider using a more specific BRAIN_BOARD_IDENTIFIER if this is not intended.\n", file=sys.stderr)
        brain_board_port = found_devices[0]["port"]
    else:
        brain_board_port = found_devices[0]["port"]
        print(f"Successfully identified ESP32-brain board on port: {brain_board_port}\n")

    return brain_board_port


def monitor_serial_output(port_name):
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

        # Clear any initial garbage data that might be in the buffer
        time.sleep(0.1)
        ser.flushInput()

        while True:
            # Read a line from the serial port.
            # .readline() reads until a newline character (\n) is found.
            # .decode('utf-8') converts bytes to a string.
            # .strip() removes leading/trailing whitespace, including the newline.
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:  # Only print if the line is not empty
                print(line)
                if WIFI_SETUP_MSG in line.lower():
                    if input("\nWould you like to automatically set up ESP32 settings and cerdentials in a new pane? (y/n): ").strip().lower() == 'y':
                        if "TMUX" not in os.environ:
                            session_name = "bluetooth cli session"

                            tmux_cmd = (
                                f'tmux new-session -d -s {session_name} "bash -c \\"clear; echo \'Click Ctrl+B followed by arrow keys to navigate windows. \nPress Ctrl+C in script window to return to menu.\'; exec bash\\"" && '
                                f'tmux split-window -h -t {session_name} "python3 {BLE_AUTOMATION_SCRIPT}; tmux kill-session -t {session_name}" && '
                                f'tmux select-pane -t {session_name}:0.0 && '
                                f'tmux attach-session -t {session_name}'
                            )
                            os.system(tmux_cmd)
                        else:
                            scripts_dir = os.path.join(os.path.expanduser('~'), 'scripts')
                            if not os.path.isfile(os.path.join(scripts_dir, BLE_AUTOMATION_SCRIPT)):
                                raise Exception(f"Bluetooth automation script not found in {scripts_dir}!")
                            tmux_cmd = f'tmux split-window -v "cd {scripts_dir} && python3 {BLE_AUTOMATION_SCRIPT}; exit"'
                            
                            subprocess.run(tmux_cmd, shell=True)
                # save in a file whether or not the serial monitor confirms that a bluetooth device was connected
                if BLUETOOH_MSG in line.lower():
                    bluetooth_connected = 1
                    with open(BLUETOOTH_FILE_PATH, "w") as f:
                        f.write(str(bluetooth_connected))
                elif bluetooth_connected == 1:
                    bluetooth_connected = 0
                    with open(BLUETOOTH_FILE_PATH, "w") as f:
                        f.write(str(bluetooth_connected))

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port_name}. {e}")
        print("Common reasons: Port is in use, incorrect port name, or no device connected.")
        print("Available ports:")
        for port in list_ports.comports():
            print(f"  {port.device} - {port.description}")
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")



if __name__ == "__main__":
    # Default serial port and baud rate
    # You might need to change 'COM3' to your ESP32's port (e.g., 'COMx' on Windows,
    # '/dev/ttyUSB0' or '/dev/ttyACM0' on Linux, '/dev/cu.usbserial-XXXX' on macOS)
    # and adjust the baud_rate if your ESP32 firmware uses a different one.
    
    # Attempt to auto-detect the port
    try:
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

        monitor_serial_output(port_to_use)
    except Exception as e:
        print(f"An unexpected error occurred: {COLOR_RED}{e}{COLOR_RESET}")
    finally:
        input("Press any key to return to main menu: ")
        sys.exit()
