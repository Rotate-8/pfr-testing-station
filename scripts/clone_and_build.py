import os
import subprocess

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

def run_command(command, cwd=None, capture_output=True):
    """
    Helper function to run a shell command and handle errors.
    """
    print(f"Executing command: {' '.join(command)}")
    try:
        # Use check=True to raise a CalledProcessError if the command returns a non-zero exit code
        # capture_outp  ut=True captures stdout and stderr
        # text=True decodes stdout/stderr as text
        result = subprocess.run(command, cwd=cwd, check=True, capture_output=capture_output, text=True)
        print("STDOUT:\n", result.stdout)
        if result.stderr:
            print("STDERR:\n", result.stderr) # Print stderr even if command succeeded (e.g., warnings)
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

def main():
    """
    Main function to execute the git clone and pio run operations.
    """
    try:
        instant_exit = False 

        R8_DIR = "/r8"
        REPO_NAME = "pfr-motor-controllers"
        REPO_DIR = os.path.join(R8_DIR, REPO_NAME)
        REPO_URL = "git@github.com:Rotate-8/pfr-motor-controllers.git"

        # 1. Create the /r8 directory if it doesn't exist
  
        os.makedirs(R8_DIR, exist_ok=True) # exist_ok=True prevents error if dir already exists


        # 2. Navigate to the /r8 directory (or rather, ensure commands run from there)
        # For subprocess.run, we use the 'cwd' argument instead of os.chdir for each command.

        # 3. Clone the repository with submodules
        if os.path.exists(REPO_DIR):
            print(f"Directory '{REPO_DIR}' already exists.")
            if input("Are you sure you want to overwrite it? (y/n): ").strip().lower() != 'y':
                print("Exiting without changes.\n")
                return
            else:
                subprocess.run(["rm", "-rf", REPO_DIR], check=True)

        print(f"Cloning repository: {REPO_URL} into {REPO_DIR}...")
        if not run_command(["git", "clone", "--recurse-submodules", "-j8", REPO_URL], cwd=R8_DIR):
            raise Exception("Git clone failed.")
            

        # 4. Run 'pio run' inside the cloned repository directory
        print(f"Building motor-controllers project in {REPO_DIR}...")
        if not run_command(["pio", "run"], cwd=REPO_DIR, capture_output=False):
            raise Exception("\'pio run\' failed. Exiting.")
        
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR:{e}{COLOR_RESET}")
    finally:
        if not instant_exit:
            input("Press enter to return to main menu: ")

if __name__ == "__main__":
        main()
