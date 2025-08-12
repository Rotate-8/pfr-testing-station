#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys

RUST_PROJECT_DIR = "/r8/pfr-rust-nodes"
BLE_CMD = ["cargo", "run", "-p", "pfr_ble_cli"]

def main():
    if not os.path.isdir(RUST_PROJECT_DIR):
        print(f"ERROR: '{RUST_PROJECT_DIR}' does not exist.", file=sys.stderr)
        sys.exit(1)

    if shutil.which("cargo") is None:
        print("ERROR: 'cargo' not found in PATH.", file=sys.stderr)
        sys.exit(1)

    print("Opening BLE CLI… (Ctrl+C to exit)")
    try:
        # Inherit your terminal's stdin/stdout/stderr for full interactivity
        result = subprocess.run(BLE_CMD, cwd=RUST_PROJECT_DIR)
        sys.exit(result.returncode)
    except KeyboardInterrupt:
        # Let Ctrl+C cleanly stop the CLI
        sys.exit(130)
    except FileNotFoundError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
