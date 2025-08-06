"""
burn_addr.py

1) Reset the ESP32‑S3 into its normal firmware mode
2) Show its startup banner (filtering out the firmware’s own prompts)
3) Prompt once for:
     • Sensor I²C address (hex, default 0x40)
     • Action: Read (r) or Burn (b)
     • New address (if burning)
4) Send all inputs in one sequence
5) Print firmware output (filtering its prompts)
"""
import serial, time, sys, subprocess, json

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

BAUD = 115200
RESET_PULSE_MS = 50
BOOT_SETTLE_MS = 300
RESPONSE_TIMEOUT = 2.0  # seconds

ESP32_S3_IDENTIFIER = "USB Single Serial"

def run_pio_command(command, capture_output=False, text=False):
    """
    Helper function to run a PlatformIO shell command and handle common errors.
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

def find_serial_port():
    """
    Finds the serial port for the ESP32-S3 board using 'pio device list --json-output'.
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
        port = dev.get("port")
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

def reset_esp32(port):
    ser = serial.Serial(port, BAUD, timeout=1, rtscts=False, dsrdtr=False)
    ser.setRTS(True)     # BOOT high → normal boot
    ser.setDTR(False)    # EN low
    time.sleep(RESET_PULSE_MS/1000.0)
    ser.setDTR(True)     # EN high → run firmware
    time.sleep(BOOT_SETTLE_MS/1000.0)
    return ser

def print_banner(ser):
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

def main():
    # 1) Reset & show banner
    port = find_serial_port()
    print(f"Resetting ESP32 on {port} @ {BAUD} baud…")
    ser = serial.Serial(port, BAUD) # Open serial connection here
    ser = reset_esp32(port)
    if not print_banner(ser):
        print("\nError: No I2C prompted raised by ESP32. Check the ESP32-S3 is connected and try resetting.", file=sys.stderr)
        input("Press any key to return to menu: ")
        ser.close()
        sys.exit(1)

    # 2) Single round of user prompts
    sensor_addr = input("\nCurrent I²C address [default 0x40]: ").strip() or "0x40"

    print(f"\n>> Sensor addr → {sensor_addr}")
    ser.write((sensor_addr + "\n").encode())
    time.sleep(0.1)

    while True:
        line = ser.readline().decode('utf-8', errors='ignore')
        if "error state" in line.lower():
            print("Invalid I²C address detected, restarting ESP32\n")
            time.sleep(2)
            main()
        elif "do you want to" in line.lower():
            break

    print('\n>>WARNING: Make sure you only burn a new address to a sensor once.')
    valid_addr = False
    while not valid_addr:
        new_addr = input("Input new I²C address to burn (e.g. 0x41): ").strip()
        try:
            dec_addr = int(new_addr, 16)
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
    end = time.time() + RESPONSE_TIMEOUT
    while time.time() < end:    
        l = ser.readline().decode(errors='ignore').rstrip()
        if not l:
            continue
        print(l)

    ser.close()
    print("\nDone.")

if __name__=="__main__":
    try:
        main()
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        input("Press any key to return to menu: ")
        sys.exit(1)
