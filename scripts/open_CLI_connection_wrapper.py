import subprocess
import sys
from open_serial_monitor import CLI_AUTOMATION_SCRIPT

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

if __name__ == '__main__':
    instant_exit = False
    try:
        mode = input("Would you like to use [Z]enoh or [B]luetooth to open CLI: ").strip().lower()
        while mode[0] not in ("z", "b"):
            mode = input("Would you like to use [Z]enoh or [B]luetooth to open CLI: ").strip().lower()
        
        if mode =='b':
            mode_flag = "--mode=BLE"
        else:
            mode_flag = "--mode=ZENOH"

        adjustment_option = input("\nWould you like to automatically [S]et settings to testing mode [R]eset motor controller settings to robot standard or [M]anually adjust: ").strip().lower()
        while adjustment_option not in ('s', 'r', 'm'):
            adjustment_option = input("Would you like to automatically [S]et settings to testing mode [R]eset motor controller settings to robot standard or [M]anually adjust: ").strip().lower()
        
        if adjustment_option == 's':
            adjustment_flag = "--noreset_settings"
        elif adjustment_option == 'r':
            adjustment_flag = "--reset_settings"
        else:
            adjustment_flag = "--noautomate_commands"

        subprocess.run(['python3', CLI_AUTOMATION_SCRIPT, mode_flag, adjustment_flag], capture_output=False)
    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}ERROR: {e}{COLOR_RESET}")
        if not instant_exit:
            input("Press enter to return: ")
        sys.exit()
