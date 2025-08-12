"""
Burns a new I2C address to an encoder plugged into an ESP32-S3 board.

This script connects to the ESP32-S3 via serial, prompts the user for the
current and new I2C addresses, and sends the appropriate commands to change
the address. It uses PlatformIO CLI to identify the correct serial port.

Author: Paras + Vouk
Date: 2025-8-6
Project: Motor-Controller Station
Language: Python 3.12

Usage:
    Host must have PlatformIO CLI and pyserial installed.
    $ python3 burn_addr.py

Variable Typing Requirements:
    - All constants and configuration variables are explicitly typed.
    - Function arguments and return types are annotated where possible.
    - Serial port and subprocess results are typed for clarity.

Documentation:
    The code follows documentation guidelines (§SWE-061 / PEP 257, Google format).
    Every public function and module begins with a clear 72-char summary line,
    then a blank line, then extended detail if needed.
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
#   │ run_pio_command             │
#   │ find_serial_port            │
#   │ reset_esp32                 │
#   │ print_banner                │
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
import serial
import time
import sys
import subprocess
import json

# ──────────────────────────────────────────────────────────────────────────
# Constants & configuration
# -------------------------------------------------------------------------

COLOR_RED: str = "\033[91m"
COLOR_YELLOW: str = "\033[93m"
COLOR_RESET: str = "\033[0m"

BAUD: int = 115_200
RESET_PULSE_MS: int = 50      # Duration EN low → high
BOOT_SETTLE_MS: int = 300     # Give firmware time to boot
RESPONSE_TIMEOUT: float = 2.0 # Seconds to collect firmware banner

ESP32_S3_IDENTIFIER: str = "USB Single Serial"  # Sub‑string from `pio` desc


# ──────────────────────────────────────────────────────────────────────────
# Helper functions
# -------------------------------------------------------------------------

def run_pio_command(command: list[str], capture_output: bool = False, text: bool = False) -> subprocess.CompletedProcess:
    """
    Runs a PlatformIO shell command and handles common errors.

    Args:
        command (list[str]): The command and arguments to run.
        capture_output (bool): Whether to capture stdout/stderr.
        text (bool): If True, output is returned as string.

    Returns:
        subprocess.CompletedProcess: The result of the subprocess run.

    Raises:
        SystemExit: If the command fails or PlatformIO is not found.
    """
    try:
        result = subprocess.run(
            command,
            check=True, # Raise CalledProcessError for non-zero exit codes
            capture_output=capture_output,
            text=text
        )
        return result
    except subprocess.CalledProcessError as e:
        print(f"Error: PlatformIO command '{' '.join(e.cmd)}' failed with exit code {e.returncode}", file=sys.stderr)
        if capture_output:
            print("STDOUT:\n", e.stdout, file=sys.stderr)
            print("STDERR:\n", e.stderr, file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(f"Error: 'pio' command not found. Make sure PlatformIO Core is installed and in your PATH.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred while running pio command: {e}", file=sys.stderr)
        sys.exit(1)

def find_serial_port() -> str:
    """
    Finds the serial port for the ESP32-S3 board using 'pio device list --json-output'.

    Returns:
        str: The serial port name (e.g., 'COM3' or '/dev/ttyACM0').

    Raises:
        SystemExit: If no suitable device is found or JSON cannot be parsed.
    """
    print(f"Searching for ESP32-S3 board with identifier: '{ESP32_S3_IDENTIFIER}'...")
    try:
        result = run_pio_command(["pio", "device", "list", "--json-output"], capture_output=True, text=True)
        devices = json.loads(result.stdout)
    except json.JSONDecodeError:
        print("Error: Could not parse JSON output from 'pio device list'.", file=sys.stderr)
        sys.exit(1)

    s3_port = None
    found_devices = []

    for dev in devices:
        description = dev.get("description", "").lower()
        hwid = dev.get("hwid", "").lower()

        # Check if the identifier is in the description or hardware ID
        if ESP32_S3_IDENTIFIER.lower() in description or ESP32_S3_IDENTIFIER.lower() in hwid:
            found_devices.append(dev)

    if not found_devices:
        print(f"{COLOR_RED}Error: No ESP32-S3 board found matching identifier '{ESP32_S3_IDENTIFIER}'.{COLOR_RESET}", file=sys.stderr)
        print("Please ensure the board is connected and powered, and verify its identifier.", file=sys.stderr)

        input("Press any key to return to menu: ")
        sys.exit(1)
    elif len(found_devices) > 1:
        print(f"{COLOR_YELLOW}Warning: Multiple ESP32-S3 devices found matching identifier '{ESP32_S3_IDENTIFIER}'. Using the first one found: {found_devices[0]['port']}{COLOR_RESET}", file=sys.stderr)
        print("Consider using a more specific identifier if this is not intended.\n", file=sys.stderr)
        s3_port = found_devices[0]["port"]
    else:
        s3_port = found_devices[0]["port"]
        print(f"Successfully identified ESP32-S3 on port: {s3_port}\n")

    return s3_port


def reset_esp32(port: str) -> serial.Serial:
    """
    Resets the ESP32-S3 by toggling DTR/RTS lines on the given serial port.

    Args:
        port (str): The serial port to use.

    Returns:
        serial.Serial: The open serial connection after reset.
    """
    ser = serial.Serial(port, BAUD, timeout=1, rtscts=False, dsrdtr=False)
    ser.setRTS(True)     # BOOT high → normal boot
    ser.setDTR(False)    # EN low
    time.sleep(RESET_PULSE_MS/1000.0)
    ser.setDTR(True)     # EN high → run firmware
    time.sleep(BOOT_SETTLE_MS/1000.0)
    return ser


def print_banner(ser: serial.Serial) -> bool:
    """
    Reads and prints the ESP32 firmware banner, looking for the I2C prompt.

    Args:
        ser (serial.Serial): The open serial connection.

    Returns:
        bool: True if the I2C prompt is detected, False otherwise.
    """
    lines, end = [], time.time() + 1.0
    i2c_prompt = False
    while time.time() < end:
        l = ser.readline().decode(errors='ignore').strip()
        if l:
            # Filter out firmware's own interactive prompts
            if l.startswith("What is the sensor's I2C address"):
                return True
            print("  ", l)
    return False

def main() -> None:
    """
    Main program flow for burning a new I2C address to the sensor.

    Steps:
        1. Find and reset the ESP32-S3 board.
        2. Prompt user for current and new I2C addresses.
        3. Send commands to burn the new address.
        4. Print firmware output and exit.
    """
    # 1) Reset & show banner
    port: str = find_serial_port()
    print(f"Resetting ESP32 on {port} @ {BAUD} baud…")
    ser: serial.Serial = serial.Serial(port, BAUD) # Open serial connection here
    ser = reset_esp32(port)
    if not print_banner(ser):
        print("\nError: No I2C prompted raised by ESP32. Check the ESP32-S3 is connected and try resetting.", file=sys.stderr)
        input("Press any key to return to menu: ")
        ser.close()
        sys.exit(1)

    # 2) Single round of user prompts
    sensor_addr: str = input("\nCurrent I²C address [default 0x40]: ").strip() or "0x40"

    print(f"\n>> Sensor addr → {sensor_addr}")
    ser.write((sensor_addr + "\n").encode())
    time.sleep(0.1)

    while True:
        line: str = ser.readline().decode('utf-8', errors='ignore')
        if "error state" in line.lower():
            print("Invalid I²C address detected, restarting ESP32\n")
            time.sleep(2)
            main()
        elif "do you want to" in line.lower():
            break

    print('\n>>WARNING: Make sure you only burn a new address to a sensor once.')
    valid_addr: bool = False
    while not valid_addr:
        new_addr: str = input("Input new I²C address to burn (e.g. 0x41): ").strip()
        try:
            dec_addr: int = int(new_addr, 16)
        except ValueError:
            print("Please enter a valid hex number.")
            continue
        if dec_addr > 2**10 - 1:
            print("Please enter a hex number that is less than 10 bits large")
        else:
            valid_addr = True

    # 3) Send commands in sequence

    print(f">> Action → BURN")
    ser.write(("b \n").encode())
    time.sleep(0.05)

    print(f">> New addr → {new_addr}")
    ser.write((new_addr + "\n").encode())

    ser.flush()

    # 4) Read and print firmware output (filter prompts)
    print("\n-- Firmware output --")
    end: float = time.time() + RESPONSE_TIMEOUT
    while time.time() < end:    
        l: str = ser.readline().decode(errors='ignore').rstrip()
        if not l:
            continue
        print(l)

    ser.close()
    print("\nDone.")

if __name__=="__main__":
    """
    Script entry point. Handles top-level exceptions and user exit.
    """
    try:
        instant_exit: bool = False
        main()
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        if not instant_exit:
            input("Press enter to return to menu: ")
