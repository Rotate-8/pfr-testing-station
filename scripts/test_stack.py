import os
import sys
import subprocess

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from start_menu import CLI_AUTOMATION_SCRIPT

WORK_DIR     = '/r8/pfr-software'
SETUP_SCRIPT = os.path.join(WORK_DIR, 'install/setup.bash')
BRINGUP_CMD  = 'ros2 launch pfr_launch trike-without-teleop-bringup.launch.yaml'
TELEOP_CMD   = 'ros2 run pfr_teleop pfr_teleop'

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

def die(msg: str):
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)

def main():
    try:
        instant_exit = False
        # 1) cd into workspace
        if not os.path.isdir(WORK_DIR):
            die(f"working directory not found: {WORK_DIR}")
        os.chdir(WORK_DIR)

        # 2) verify setup script
        if not os.path.isfile(SETUP_SCRIPT):
            die(f"setup script not found: {SETUP_SCRIPT}")

        # 3) turn off zenoh so it can be launched by setup script
        print("Turning of zenoh endpoint")
        subprocess.run(
        ['sudo', '/usr/bin/systemctl', 'stop', 'zenohd.service'],
        check=True,
        capture_output=True,
        text=True)

        # 3) launch bring-up in background (no output)
        print("Running ROS2 bring-up in background...")
        subprocess.Popen([
            'bash', '-i', '-c',
            f'source "{SETUP_SCRIPT}" && exec {BRINGUP_CMD}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


        # 4) prompt user
        input("\nBring-up launched in the background.  Please click the reset button on the microcontroller then press Enter to continue.")

        # 5) run teleop in foreground
        print("Running pfr_teleop in this terminal…")
        try:
            proc = subprocess.Popen([
                'bash', '-i', '-c',
                f'source "{SETUP_SCRIPT}" && exec {TELEOP_CMD}'
            ])
            proc.wait()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received. Terminating teleop process...")
            proc.terminate()
            proc.wait()

        if input("\nWould you like to change motor controller settings back to standard? (y/n): ").lower().stirp() == "y":
            subprocess.run(
            ['python3', CLI_AUTOMATION_SCRIPT, '--mode=zenoh', '--reset_settings'],
            check=True,
            capture_output=False
            )

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR: {e}{COLOR_RESET}")
    finally:
        subprocess.run(
        ['sudo', '/usr/bin/systemctl', 'start', 'zenohd.service'],
        check=True,
        capture_output=True,
        text=True)
        if not instant_exit:
            print("Press enter to return to main menu: ")
        sys.exit()

if __name__ == '__main__':
    main()