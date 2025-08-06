import os
import subprocess
import sys
from open_serial_monitor import BLUETOOTH_FILE_PATH
import time

rust_project_directory = "/r8/pfr-rust-nodes"
rust_cmd = ["cargo", "run", "-p", "pfr_ble_cli"]

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

def adjust_settings(process):
    CMDS = {
        "zenoh-endpoints": "tcp/10.8.10.109:7447",
        "wifi-ssid": "twistedfields",
        "wifi-pass": "alwaysbekind",
        "steer-zero-on-boot": "0",
        "drive-motor-control-type": "torque",
        "steer-motor-control-type": "torque"
    }

    for command in CMDS.keys():
        val = CMDS[command]
        process.stdin.write(f"settings {command} {val}")
        if f"\'{command}\' set to \'{val}\'" not in process.stdout.readline():
            process.stdin.write("settings help")
            curr_line = ""
            while curr_line:
                settings_list += curr_line
                curr_line = process.stdout.readline()
            raise Exception(f"Failed to set {command} to {val}. Current list of available settings:{COLOR_RESET}\n{settings_list}")



def connect_to_bluetooth_cli(process, selected_adapter=None):
    FOUND_ADAPTERS_LINES = ["found", "ble adapter"]
    ADAPTER_PROMPT = "select an adapter to use"
    SCAN_MSG = "starting 3 second scan"
    read_lines = True
    while True:
        output_line = process.stdout.readline()
            
        if read_lines:
            sys.stdout.write(output_line)
        sys.stdout.flush()

        if FOUND_ADAPTERS_LINES[0] in output_line.lower() and FOUND_ADAPTERS_LINES[1] in output_line.lower():
            print("Adapter list started.")
            start_time = time.time()
            num_adapters = int(output_line[output_line.lower().index(FOUND_ADAPTERS_LINES[0]) + len(FOUND_ADAPTERS_LINES[0]):output_line.lower().index(FOUND_ADAPTERS_LINES[1])].strip())
            selected_adapter = None 
            while time.time() - start_time < 10:
                output_line = process.stdout.readline()
                sys.stdout.write(output_line)
                sys.stdout.flush()
                if output_line and ADAPTER_PROMPT in output_line.lower():
                    if num_adapters == 1:
                        print("\nSelecting the first adapter...")
                        selected_adapter = 0
                        # Automatically select the first adapter by sending a newline.
                        process.stdin.write("\n")
                        process.stdin.flush()
                        break
                    else:
                        while selected_adapter is None:
                            try: 
                                selected_adapter = int(input().strip())
                                process.stdin.write(f"{selected_adapter}\n")
                            except ValueError:
                                print("Please enter a valid number for the adapter selection.")
            if selected_adapter == None:
                raise Exception("Timeout waiting for adapter query...")

        
        # If the program is asking for a different input, we can let the user type it.
        # You would add more `elif` checks for other prompts you want to automate.
        elif SCAN_MSG in output_line:
            read_lines = False
            bluetooth_confirmed = 0
            motor_in_list_to_connect = None
            start_time = time.time()
            scanned_lines = ""
            print("Connecting brain board...")
            # Give the program an extra second to do the scan
            motor_selected = False
            while time.time() - start_time < 4:
                output_line = process.stdout.readline()
                scanned_lines += output_line
                # always chooses the first one I guess I don't know what else to do
                if not motor_selected and "motor_0_ble" in output_line.lower():
                    motor_in_list_to_connect = int(output_line[output_line.index("{") + 1:output_line.index("}")].strip())
                    motor_selected = True
                    print(f"Found potential motor controller: {motor_in_list_to_connect}, connecting...")
                elif "enter a peripheral index" in output_line.lower():
                    print("Entering selected motor into prompt...")
                    process.stdin.write(f"{motor_in_list_to_connect}\n")
                    # Give the serial output time to respond
                    time.sleep(2)
                    if os.path.isfile(BLUETOOTH_FILE_PATH):
                        with open(BLUETOOTH_FILE_PATH, "r") as f:   
                            bluetooth_confirmed = f.read().strip()
                            # ignore if file is empty
                            if bluetooth_confirmed != "":
                                bluetooth_confirmed = int(bluetooth_confirmed)
                    if bluetooth_confirmed == 1:
                        print("Successfully connected to desired motor controller.")
                        return True
                    else:
                        print("Connected to different motor controller in BLE list, rerunning...")
                        return False
            print("While loop finished")
            if motor_in_list_to_connect is None:
                raise Exception(f"No valid motor controller found in the following scan results:\n{scanned_lines}")
            else:
                raise Exception("Found potential motor controller in ble list but timeout waiting to input selection.")    
                
    

def main():
    instant_exit = False
    print("=== Launching BLE CLI using pfr-rust-nodes ===")
    if not os.path.isdir(rust_project_directory):
        with open('r8/pfr-rust-nodes', 'w') as f:
            f.write(f"Error: The directory '{rust_project_directory}' does not exist.")
        return

    try:
        process = subprocess.Popen(
            rust_cmd,
            cwd=rust_project_directory,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1 # This makes it line-buffered, so we get output line by line.
        )
        # Keep retrying connection
        while not connect_to_bluetooth_cli(process):
            process = subprocess.Popen(
            rust_cmd,
            cwd=rust_project_directory,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1 # This makes it line-buffered, so we get output line by line.
            )
        print("Bluetooth connected successfully, adjusting settings")
        adjust_settings(process)
        print("Settings adjusted successfully.")
    except KeyboardInterrupt:
        instant_exit = True
    except FileNotFoundError:
        print(f"{COLOR_RED}\nERROR: 'cargo' command not found. Is Rust installed and in your PATH?{COLOR_RESET}")
    except Exception as e:
        print(f"{COLOR_RED}\nERROR: {e}{COLOR_RESET}")
    finally:
        # A good practice is to make sure the process is closed,
        # even if an error occurs.
        if 'process' in locals() and process.poll() is None:
            process.kill()
            print("\nProcess was terminated.")
        if not instant_exit:
            input("\nPress any key to return to close this pane: ")
        sys.exit()

if __name__ == "__main__":
    main()