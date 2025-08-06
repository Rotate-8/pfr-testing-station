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

"""
ONLY burns a new I2C address to the ES32-s3 board.

The script connects to the ESP32-S3 via serial, prompts the user for the
current and new I2C addresses, and sends the appropriate commands to change
the address. ** Currently only tested with ESP32-S3, hasn't been verified 
with the main microcontroller. **

Author: Paras + Vouk  •  Date: 2025-8-6
Project: Motor-Controller Station  •  Language: Python 3.12

Usage-host must have PlatformIO CLI and pyserial installed.:
   $ python3 burn_addr.py

** IMPORTANT*******************************************************
The code follows NASA-style documentation guidelines (§SWE-061 / PEP 257
Google format) so every public function and module begins with a clear
72-char summary line, then a blank line, then extended detail.
*******************************************************************
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import PurePath
from typing import List, Optional

import serial

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

def run_pio_command(cmd: List[str], *, capture_output: bool = False,
                    text: bool = False) -> subprocess.CompletedProcess:
    """Invoke PlatformIO and surface any errors immediately.

    Args:
        cmd: Full command line broken into a list, e.g.
            ``["pio", "device", "list", "--json-output"]``.
        capture_output: If ``True`` the subprocess’ STDOUT/ERR are
            captured and returned for later parsing.
        text: Forwarded to :pyfunc:`subprocess.run` so output is returned
            as *str* instead of *bytes*.

    Returns:
        The :class:`subprocess.CompletedProcess` instance on success.

    Raises:
        SystemExit: On *any* failure, after printing a user‑friendly
            message. NASA SWE‑061 requires early termination on
            unexpected states to avoid undefined behaviour.
    """
    try:
        return subprocess.run(
            cmd,
            check=True,
            capture_output=capture_output,
            text=text,
        )
    except subprocess.CalledProcessError as exc:
        print(
            f"Error: PlatformIO command '{' '.join(exc.cmd)}' failed with "
            f"exit code {exc.returncode}",
            file=sys.stderr,
        )
        if capture_output:
            print("STDOUT:\n", exc.stdout, file=sys.stderr)
            print("STDERR:\n", exc.stderr, file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(
            "Error: 'pio' command not found. Ensure PlatformIO Core is "
            "installed and in your PATH.",
            file=sys.stderr,
        )
        sys.exit(1)
    except Exception as exc:  # noqa: BLE001
        print(
            f"Unexpected error while running pio command: {exc}",
            file=sys.stderr,
        )
        sys.exit(1)


def find_serial_port() -> str:
    """Identify the ESP32‑S3’s serial port via ``pio device list``.

    Returns:
        The *tty* (``/dev/ttyUSB0``) or *COM* port string.

    Exits:
        With ``sys.exit(1)`` if no matching, or multiple ambiguous,
        devices are detected.
    """
    print(
        "Searching for ESP32‑S3 board with identifier: "
        f"'{ESP32_S3_IDENTIFIER}'…"
    )

    try:
        result = run_pio_command(
            ["pio", "device", "list", "--json-output"],
            capture_output=True,
            text=True,
        )
        devices = json.loads(result.stdout)
    except json.JSONDecodeError:
        print("Error: malformed JSON from 'pio device list'.",
              file=sys.stderr)
        sys.exit(1)

    matches: List[dict] = [
        d for d in devices
        if ESP32_S3_IDENTIFIER.lower() in d.get("description", "").lower()
        or ESP32_S3_IDENTIFIER.lower() in d.get("hwid", "").lower()
    ]

    if not matches:
        print(
            f"{COLOR_RED}Error: No ESP32‑S3 detected matching "
            f"'{ESP32_S3_IDENTIFIER}'.{COLOR_RESET}",
            file=sys.stderr,
        )
        sys.exit(1)

    if len(matches) > 1:
        print(
            f"{COLOR_YELLOW}Warning: multiple ESP32‑S3 boards match. "
            "Using the first: "
            f"{matches[0]['port']}{COLOR_RESET}",
            file=sys.stderr,
        )

    port: str = matches[0]["port"]
    print(f"Found ESP32‑S3 on {port}\n")
    return port


def reset_esp32(port: str) -> serial.Serial:  # type: ignore[valid-type]
    """Pulse the EN pin to reboot firmware then return an open Serial obj.

    Args:
        port: The tty / COM port string returned by
            :pyfunc:`find_serial_port`.

    Returns:
        An **open** :class:`serial.Serial` instance ready for read/write.
    """
    ser = serial.Serial(port, BAUD, timeout=1, rtscts=False, dsrdtr=False)
    ser.setRTS(True)          # BOOT high → normal boot
    ser.setDTR(False)         # EN low
    time.sleep(RESET_PULSE_MS / 1000)
    ser.setDTR(True)          # EN high → run firmware
    time.sleep(BOOT_SETTLE_MS / 1000)
    return ser


def print_banner(ser: serial.Serial) -> bool:  # type: ignore[valid-type]
    """Stream the firmware banner for a second and detect I²C prompt.

    Args:
        ser: The open serial connection.

    Returns:
        ``True`` if the firmware emitted its *I²C address prompt* (meaning
        it is ready for commands); ``False`` otherwise.
    """
    end_time = time.time() + 1.0
    while time.time() < end_time:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("What is the sensor's I2C address"):
            return True
        print("  ", line)
    return False

# ──────────────────────────────────────────────────────────────────────────
# Main program flow
# -------------------------------------------------------------------------

def main() -> None:  # noqa: C901  (cyclomatic okay for a CLI tool)
    """Interactive CLI for reading or burning an I²C address.

    The sequence is
      1. Identify and reset the ESP32‑S3.
      2. Display its banner and confirm readiness.
      3. Prompt the operator for current and new addresses.
      4. Send the command sequence (read or burn).
      5. Echo firmware output for logging.
    """
    port = find_serial_port()
    print(f"Resetting ESP32 on {port} @ {BAUD} baud…")

    ser = reset_esp32(port)
    if not print_banner(ser):
        print(
            "Error: ESP32 did not prompt for I²C address. Check wiring and "
            "try again.",
            file=sys.stderr,
        )
        sys.exit(1)

    # ─── Operator prompts ────────────────────────────────────────────────
    sensor_addr = input("\nCurrent I²C address [default 0x40]: ").strip() or "0x40"
    print(f"\n>> Sensor addr → {sensor_addr}")
    ser.write(f"{sensor_addr}\n".encode())
    time.sleep(0.1)

    # Wait for firmware to ask next question
    while True:
        if "do you want to" in ser.readline().decode("utf‑8", "ignore").lower():
            break

    print("\nWARNING: Burn a new address only once per sensor!")

    while True:
        new_addr = input("Input new I²C address to burn (e.g. 0x41): ").strip()
        try:
            dec_addr = int(new_addr, 16)
            if dec_addr < 2 ** 10:  # 10‑bit I²C limit
                break
            print("Address must be < 10 bits (0x3FF).")
        except ValueError:
            print("Please enter a valid hex number.")

    # ─── Send burn sequence ──────────────────────────────────────────────
    print(">> Action → BURN")
    ser.write(b"b \n")
    time.sleep(0.05)

    print(f">> New addr → {new_addr}")
    ser.write(f"{new_addr}\n".encode())
    ser.flush()

    # ─── Read firmware response ──────────────────────────────────────────
    print("\n-- Firmware output --")
    stop = time.time() + RESPONSE_TIMEOUT
    while time.time() < stop:
        out = ser.readline().decode(errors="ignore").rstrip()
        if out:
            print(out)

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"\nFatal error: {exc}", file=sys.stderr)
        sys.exit(1)
