"""
Clones the motor-controller repository and builds it using PlatformIO.

This script creates a target directory, clones the Rotate-8 motor-controller
repository (with submodules), and runs `pio run` to build the project.
It handles errors and prompts the user before overwriting any existing repo.

Author: Paras + Vouk  •  Date: 2025-8-6
Project: Motor-Controller Station  •  Language: Python 3.12

Usage-host must have PlatformIO CLI and git installed:
   $ python3 clone_and_build_w_doc.py

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

import os
import subprocess
import sys
import time

# ──────────────────────────────────────────────────────────────────────────
# Constants & configuration
# -------------------------------------------------------------------------

R8_DIR = "/r8"
REPO_NAME = "pfr-motor-controllers"
REPO_DIR = os.path.join(R8_DIR, REPO_NAME)
REPO_URL = "git@github.com:Rotate-8/pfr-motor-controllers.git"

# ──────────────────────────────────────────────────────────────────────────
# Helper functions
# -------------------------------------------------------------------------

def run_command(command, cwd=None, capture_output=True):
    """Run a shell command and print output/errors.

    Args:
        command: List of command arguments, e.g. ["git", "clone", ...].
        cwd: Directory to run the command from.
        capture_output: If True, captures and prints stdout/stderr.

    Returns:
        True if the command succeeded, False otherwise.
    """
    print(f"Executing command: {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=True,
            capture_output=capture_output,
            text=True
        )
        print("STDOUT:\n", result.stdout)
        if result.stderr:
            print("STDERR:\n", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error: Command '{' '.join(e.cmd)}' failed with exit code {e.returncode}")
        print("STDOUT (if any):\n", e.stdout)
        print("STDERR:\n", e.stderr)
        return False
    except FileNotFoundError:
        print(f"Error: Command '{command[0]}' not found. Make sure it's installed and in your PATH.")
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return False

# ──────────────────────────────────────────────────────────────────────────
# Main program flow
# -------------------------------------------------------------------------

def main():
    """Clone the repo and build it using PlatformIO.

    Steps:
      1. Create the /r8 directory if it doesn't exist.
      2. Prompt before overwriting any existing repo directory.
      3. Clone the repository with submodules.
      4. Build the project using PlatformIO.
    """
    try:
        os.makedirs(R8_DIR, exist_ok=True)
    except OSError as e:
        print(f"Error: Failed to create directory '{R8_DIR}': {e}")
        sys.exit(1)

    if os.path.exists(REPO_DIR):
        print(f"Directory '{REPO_DIR}' already exists.")
        if input("Are you sure you want to overwrite it? (y/n): ").strip().lower() != 'y':
            print("Exiting without changes.")
            time.sleep(2)
            sys.exit(0)
        else:
            try:
                subprocess.run(["rm", "-rf", REPO_DIR], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error removing directory: {e}")
                sys.exit(1)

    print(f"Cloning repository: {REPO_URL} into {REPO_DIR}...")
    if not run_command(["git", "clone", "--recurse-submodules", "-j8", REPO_URL], cwd=R8_DIR):
        print("Git clone failed.")
        sys.exit(1)

    print(f"Building motor-controllers project in {REPO_DIR}...")
    if not run_command(["pio", "run"], cwd=REPO_DIR, capture_output=False):
        print("'pio run' failed. Exiting.")
        sys.exit(1)

    print("Script completed successfully.")

# ──────────────────────────────────────────────────────────────────────────
# Script entry point
# -------------------------------------------------------------------------

if __name__ == "__main__":
    main()
