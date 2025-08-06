import subprocess
import sys
import json
import os
import time
import serial


# --- Configuration ---
# IMPORTANT: Adjust these paths and identifiers for your setup!

# Path to your PlatformIO project directory (where platformio.ini is located)
# This is crucial for 'pio run' to find your project.
PIO_PROJECT_PATH = "/r8/pfr-motor-controllers" # <<< CHANGE THIS! Example: "/home/user/my_esp32_project"
PIO_BUILD_ENV_NAME = "motor-controller"

# Identifier for your ESP32-brain board.
# Based on your 'pio device list --json-output', the FT232R chip is unique to your brain board.
# The ESP32-S3 typically shows "USB Single Serial" or similar.
FLASH_MODE = "dio"  # Default flash mode
BRAIN_BOARD_IDENTIFIER = "USB Single Serial" # Updated to distinguish the brain board (FT232R chip)

# --- Flashing Specifics (Derived from your exact pio run output) ---
# These parameters are taken directly from the successful esptool.py command you provided.
FLASH_CHIP_TYPE = "esp32s3"
UPLOAD_BAUD_RATE = "921600"
FLASH_MODE = "dio"
FLASH_FREQ = "80m"
FLASH_SIZE = "8MB"

# Offsets for each binary file as specified in your pio run output
BOOTLOADER_OFFSET = "0x0000"
PARTITIONS_OFFSET = "0x8000"
BOOT_APP0_OFFSET = "0xe000"
FIRMWARE_OFFSET = "0x10000"

# Path to boot_app0.bin (this is a fixed path within PlatformIO's packages)


COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

# --- Helper Functions ---


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
            check=True, # Raise CalledProcessError for non-zero exit codes
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


# TODO: Debug and test
def enter_esp32_boot_mode(port: str, baud_rate: int = 115200) -> bool:
    """
    Attempts to put an ESP32 board into boot mode by manipulating DTR and RTS lines.
    This function is intended to be called just before flashing.
    """
    ser = None
    try:
        ser = serial.Serial(port, baud_rate, rtscts=False, write_timeout=0)
        print(f"Successfully opened serial port {port} at {baud_rate} baud for boot mode sequence.")
        ser.setDTR(True)
        ser.setRTS(True)
        time.sleep(0.1)
        print("Attempting to enter boot mode...")
        ser.setDTR(False)
        time.sleep(0.05)
        ser.setRTS(False)
        time.sleep(0.05)
        ser.setRTS(True)
        time.sleep(0.05)
        ser.setDTR(True)
        time.sleep(0.2)
        print("ESP32 boot mode sequence completed. The board should now be in bootloader mode.")
        return True
    except serial.SerialException as e:
        print(f"Serial port error during boot mode sequence: {e}", file=sys.stderr)
        print("Please ensure the correct port is selected and the board is connected.", file=sys.stderr)
        input("Press any key to return to menu: ")
        return False
    except Exception as e:
        print(f"An unexpected error occurred during boot mode sequence: {e}", file=sys.stderr)
        input("Press any key to return to menu: ")
        return False
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed after boot mode sequence.")


def upload_project(port, project_path, build_env_name):
    """
    Uploads the built project to the specified ESP32 port.
    """

    if not os.path.isfile(os.path.join(project_path, ".pio", "build", build_env_name, "firmware.bin")):
        print(f"\n--- No prebuilt binary file detected, building and flashing to {port} ---")
        # The --upload-port (-p) flag specifies the target port for upload
        run_command(["pio", "run", "--target", "upload", "--upload-port", port], cwd=project_path, capture_output=False)
    else:
        print(f"\n--- Flashing prebuilt binary to {port} ---")
        upload_prebuilt_binary(port, PIO_PROJECT_PATH, PIO_BUILD_ENV_NAME)
    print(f"--- Upload to {port} completed ---")


def find_files_in_directory(directory, filename, subdir=None, allow_empty=False):
    """
    Searches for a file(s) in a subdir of the specified directory and returns its full path if found.
    """
    subdirs = []
    if subdir != None:
        # use os for robustness
        while subdir != "":
            subdirs.insert(0, os.path.split(subdir)[1])
            subdir = os.path.split(subdir)[0]

    filenames = filename
    if type(filename) is not list:
        filenames = [filename]

    files_found = [None for _ in range(len(filenames))]
    
    for i in range(len(files_found)):
        files_found[i] = find_files_in_directory_helper(directory, filenames[i], subdirs)
        if files_found[i] is not None:
            directory = files_found[i][:files_found[i].index(subdirs[-1]) + len(subdirs[-1]) + 1] if subdirs != [] else directory
            subdirs = []
        else:
            print(f"File '{filenames[i]}' not found in directory '{directory}' or its subdirectories.", file=sys.stderr)
            if not allow_empty:
                input("Press any key to return to menu: ")
                sys.exit(1)
    
    return files_found if len(files_found) > 1 else files_found[0]


def find_files_in_directory_helper(directory, target_file, subdirs: list):
    if subdirs != []:
        for root, dirs, _ in os.walk(directory):
            if subdirs[0] in dirs:
                print(f"Found subdir '{subdirs[0]}' in '{root}'")
                result = find_files_in_directory_helper(os.path.join(root, subdirs[0]), target_file, subdirs[1:])
                if result is not None:
                    return result
        return None
    else:
        for root, _, files in os.walk(directory):
            if target_file in files:
                return os.path.join(root, target_file)
        return None


def upload_prebuilt_binary(port, project_path, build_env_name):
    """
    Flashes a prebuilt binary file to the specified ESP32 port using esptool.py directly.
    """
    bootloader_bin_path, partitions_bin_path, firmware_bin_path = find_files_in_directory(f"{project_path}/.pio", ["bootloader.bin", "partitions.bin", "firmware.bin"], subdir=build_env_name)

    boot_app0_bin_path = find_files_in_directory("/home/pfr/.platformio/packages/framework-arduinoespressif32/", "boot_app0.bin")

    # Validate all binary paths exist
    for path in [bootloader_bin_path, partitions_bin_path, boot_app0_bin_path, firmware_bin_path]:
        if not os.path.isfile(path):
            print(f"Error: Required binary file '{path}' not found.", file=sys.stderr)
            print("Please ensure the path is correct, and the project has been built correctly.", file=sys.stderr)
            input("Press any key to return to menu: ")
            sys.exit(1)

    print(f"\n--- Starting multi-binary flash to ESP32-brain board on {port} ---")

    # Locate esptool.py within PlatformIO's installed packages
    # This path is relative to the PLATFORMIO_PYTHON_PATH's parent directory
    esptool_path, python_interp_path = find_files_in_directory("/home/pfr/.platformio", ["esptool.py", "python"])

    for path in [esptool_path, python_interp_path]: 
        if not os.path.isfile(path):
            print(f"Error: esptool.py not found at expected path: {path}", file=sys.stderr)
            print("Please ensure PlatformIO Core is installed correctly and PLATFORMIO_PYTHON_PATH is accurate.", file=sys.stderr)
            sys.exit(1)

    print(f"\n--- Starting direct flash of prebuilt binary to ESP32-brain board on {port} ---")
    command = [
        python_interp_path, # Use the specific Python interpreter from PlatformIO's penv
        esptool_path,
        "--chip", FLASH_CHIP_TYPE,
        "--port", port,
        "--baud", str(UPLOAD_BAUD_RATE),
        "--before", "default_reset",
        "--after", "hard_reset",
        "write_flash",
        "-z",
        "--flash_mode", str(FLASH_MODE),
        "--flash_freq", str(FLASH_FREQ),
        "--flash_size", str(FLASH_SIZE),
        BOOTLOADER_OFFSET, bootloader_bin_path,
        PARTITIONS_OFFSET, partitions_bin_path,
        BOOT_APP0_OFFSET, boot_app0_bin_path,
        FIRMWARE_OFFSET, firmware_bin_path
    ]
    run_command(command, capture_output=False)
    print(f"--- Direct flash to {port} completed ---")


def monitor_device(port, project_path):
    """
    Monitors the serial output from the specified ESP32 port.
    This will run indefinitely until interrupted by the user (Ctrl+C).
    """
    print(f"\n--- Starting serial monitor for ESP32-brain board on {port} ---")
    print("Press Ctrl+C to stop monitoring and return to the main menu.")
    # The --port (-p) flag specifies the target port for monitoring
    # We do NOT capture output here, so it streams directly to the terminal.
    run_command(["pio", "device", "monitor", "-p", port], cwd=project_path, capture_output=False)
    print(f"--- Serial monitor for {port} stopped ---")


def main():
    """
    Main function to orchestrate finding the board, uploading, 
    monitoring.
    """
    if not os.path.isdir(PIO_PROJECT_PATH):
        print(f"Error: Project path '{PIO_PROJECT_PATH}' does not exist or is not a directory.", file=sys.stderr)
        print("Please update PIO_PROJECT_PATH in the script.", file=sys.stderr)
        sys.exit(1)

    try:
        # 1. Find the ESP32-brain board's serial port
        brain_board_port = find_brain_board_port(BRAIN_BOARD_IDENTIFIER)

        # 2. Prompt for boot mode on the ESP32-brain board
        x = "help"
        while x == "help":
            x = input("Put ESP32 into boot mode if you haven't already. Press Enter to continue or type 'help' for instructions: ").strip().lower()
            if x == "help":
                print("\nTo put the ESP32-brain board into boot mode:")
                print("1. Hold down the BOOT button on the board.")
                print("2. While holding BOOT, press and release the RESET button.")
                print("3. Release the BOOT button after a second.")


        # 3. Upload the project
        upload_project(brain_board_port, PIO_PROJECT_PATH, PIO_BUILD_ENV_NAME)

        # 4. Monitor the device
        monitor_device(brain_board_port, PIO_PROJECT_PATH)

        print("\nAll operations completed successfully.")

    except KeyboardInterrupt:
        print("\nOperation cancelled by user (Ctrl+C). Exiting gracefully.", file=sys.stderr)
        sys.exit(0) # Exit cleanly on user interrupt
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}", file=sys.stderr)
        input("Press any key to return to menu: ")
        sys.exit(1)

if __name__ == "__main__":
    main()
