import os
import sys
import subprocess

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from start_menu import CLI_AUTOMATION_SCRIPT

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

        # 1) turn off zenoh so it can be launched by setup script
        print("Turning of zenoh endpoint")
        subprocess.run(
        ['sudo', '/usr/bin/systemctl', 'stop', 'zenohd.service'],
        check=True,
        capture_output=True,
        text=True)

        # 2) launch bring-up in background (no output)
        print("Running ROS2 bring-up in background...")
        subprocess.Popen([
            'bash', '-i', '-c',
            f'exec {BRINGUP_CMD}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 3) prompt user
        input("\nBring-up launched in the background.  Please click the reset button on the microcontroller then press Enter to continue.")

        # 4) run teleop in foreground
        print("Running pfr_teleop in this terminal…")
        try:
            proc = subprocess.Popen([
                'bash', '-i', '-c',
                f'exec {TELEOP_CMD}'
            ])
            proc.wait()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received. Terminating teleop process...")
            proc.terminate()
            proc.wait()

        # 5) prompt user as to whether they want to reset motor controller settings
        if input("\nWould you like to change motor controller settings back to standard? (y/n): ").lower().strip() == "y":
            subprocess.run(
            ['python3', CLI_AUTOMATION_SCRIPT, '--mode=zenoh', '--reset_settings'],
            check=True,
            capture_output=False,
            )
            instant_exit = True # user will already be prompted to return

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
            input("\nPress enter to return to main menu: ")
        sys.exit()

if __name__ == '__main__':
    main()