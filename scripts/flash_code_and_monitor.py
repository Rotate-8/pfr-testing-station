import sys
import os
from open_serial_monitor import run_command, find_brain_board_port, monitor_serial_output_and_prompt_operations

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

# Path to your PlatformIO project directory (where platformio.ini is located)
# This is crucial for 'pio run' to find your project.
PIO_PROJECT_PATH = "/r8/pfr-motor-controllers"
PIO_BUILD_ENV_NAME = "motor-controller"

# Identifier for your ESP32-brain board.
# Based on your 'pio device list --json-output', the FT232R chip is unique to your brain board.
# The ESP32-S3 typically shows "USB Single Serial" or similar.
FLASH_MODE = "dio"  # Default flash mode
# TODO: CHANGE THIS BACK WHEN USING A BRAIN BOARD
BRAIN_BOARD_IDENTIFIER = "FT232R"  # Updated to match the FT232R chip identifier
# BRAIN_BOARD_IDENTIFIER = "USB Single Serial" # <<< CHANGE THIS TO "FT232R"

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

# --- Helper Functions ---

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


def find_files_in_incomplete_directory(directory, filename, subdir=None, allow_empty=False):
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
        files_found[i] = find_files_in_incomplete_directory_helper(directory, filenames[i], subdirs)
        if files_found[i] is not None:
            directory = files_found[i][:files_found[i].index(subdirs[-1]) + len(subdirs[-1]) + 1] if subdirs != [] else directory
            subdirs = []
        else:
            print(f"File '{filenames[i]}' not found in directory '{directory}' or its subdirectories.", file=sys.stderr)
            if not allow_empty:
                input("Press any key to return to menu: ")
                sys.exit(1)
    
    return files_found if len(files_found) > 1 else files_found[0]


def find_files_in_incomplete_directory_helper(directory, target_file, subdirs: list):
    if subdirs != []:
        for root, dirs, _ in os.walk(directory):
            if subdirs[0] in dirs:
                print(f"Found subdir '{subdirs[0]}' in '{root}'")
                result = find_files_in_incomplete_directory_helper(os.path.join(root, subdirs[0]), target_file, subdirs[1:])
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
    bootloader_bin_path, partitions_bin_path, firmware_bin_path = find_files_in_incomplete_directory(f"{project_path}/.pio", ["bootloader.bin", "partitions.bin", "firmware.bin"], subdir=build_env_name)

    boot_app0_bin_path = find_files_in_incomplete_directory("/home/pfr/.platformio/packages/framework-arduinoespressif32/", "boot_app0.bin")

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
    esptool_path, python_interp_path = find_files_in_incomplete_directory("/home/pfr/.platformio", ["esptool.py", "python"])

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
        "--after", "no_reset" # no reset in order to be consistent across boards that can and cant be remotely reset
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


def main():
    """
    Main function to orchestrate finding the board, uploading, 
    monitoring.
    """ 
    try:
        instant_exit = False

        if not os.path.isdir(PIO_PROJECT_PATH):
            raise ValueError(f"Error: Project path '{PIO_PROJECT_PATH}' does not exist or is not a directory. Please check and/or clone directory again.", file=sys.stderr)
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

        input("Reset the board and press enter to continue: ")
        
        # 5. Monitor the device
        monitor_serial_output_and_prompt_operations(brain_board_port)

        print("\nAll operations completed successfully.")

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"\nError occurred: {COLOR_RED}{e}{COLOR_RESET}", file=sys.stderr)
    finally:
        if not instant_exit:
            input("Press any key to return to menu: ")
        sys.exit(1)

if __name__ == "__main__":
    main()
