import os
import subprocess
import sys
from open_serial_monitor import BLUETOOTH_FILE_PATH, read_bluetooth_status
import time
import socket
import queue
import threading
from absl import app
from absl import flags
from flash_code_and_monitor import find_files_in_incomplete_directory

flags.DEFINE_string(
    "mode",
    default="ble",
    help="Which connection mode to use: 'zenoh' or 'ble' (default: 'ble')"
)
flags.DEFINE_bool(
    "reset_settings",
    default=False,
    help="Whether to reset settings or set settings. Default is False."
)
flags.DEFINE_bool(
    "automate_commands",
    default=True,
    help="Whether to automatically adjust settings once in CLI."
)
flags.DEFINE_bool(
    "board_awaiting_wifi",
    default=False,
    help="Whether board is awaiting wifi credentials."
)

motor_controller_project_directory = "/r8/pfr-motor-controllers"

rust_project_directory = "/r8/pfr-rust-nodes"
ble_cmd = ["cargo", "run", "-p", "pfr_ble_cli"]

pfr_software_directory = "/r8/pfr-software"
zenoh_cmd = ["ros2", "run", "pfr_tools", "motor_controller_cli"]

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'

ROBOT_ZENOH_ENDPOINT = 'udp/10.50.50.1:7777'


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
ipv4 = s.getsockname()[0]
ZENOH_LOCATOR = 'tcp'
ZENOH_PORT = '7447'

CMDS = {
        "zenoh-endpoint": [f"{ZENOH_LOCATOR}/{ipv4}:{ZENOH_PORT}"],
        "wifi-ssid": ["twistedfields"],
        "wifi-pass": ["alwaysbekind"],
        "steer-zero-on-boot": ["0"],
        "drive-motor-control-type": ["torque"],
        "steer-motor-control-type": ["torque"]
    }

def read_until_message(q, messages, timeout, print_output=False, compare_func=None):
    start_time = time.time()
    if type(messages) == str:
        messages = [messages]
    elif type(messages) != list:
        raise Exception("Invalid message to listen for passed in CLI connection script...")
    while time.time() - start_time < timeout:
        messages_present = True # set to True until proven false
        try:
            output_line = q.get(block=True, timeout=0.1)
            # if print_output:
                # print(output_line)

            # Just give up if an error occurs
            if "[error]" in output_line.lower():
                return False
            
            for message in messages:
                if not compare_func:
                    if message not in output_line.lower():
                        messages_present = False
                        break
                else:
                    if not compare_func(output_line.lower(), message):
                        messages_present = False
                        break
            if messages_present:
                return output_line.lower()

        except queue.Empty:
            # No output was available in the queue within the timeout, so we continue the loop
            pass
            
    return False


def count_unshared_characters(str1, str2):
    str1 = str1.replace("\n", "").replace(" ", "")
    str2 = str2.replace("\n", "").replace(" ", "")
    unshared_count = 0
    for i in range(min(len(str1), len(str2))):
        if str1[i] != str2[i]:
            unshared_count += 1
    return unshared_count + abs(len(str1) - len(str2))


def send_and_confirm_command(process, output_queue, setting, val):
    print(f"Setting {setting} to {val}")
    process.stdin.write(f"settings {setting} {val}\n")
    if not read_until_message(output_queue, "set to", 0.3, print_output=False):
        raise Exception(f"Failed to set {setting} to {val}.")


def get_settings_list(process, output_queue):
    process.stdin.write("settings help\n")
    settings_list = []
    time.sleep(2)
    # I know this is super shit, but it works
    while not output_queue.empty():
        while not output_queue.empty():
            line = output_queue.get(block=True, timeout=0.1)
            if line and line != '\n':
                settings_list.append(line.strip())
    return settings_list


def reset_settings(process, output_queue, zenoh_endpoint_addr=None):
    settings_list = get_settings_list(process, output_queue)
    settings_file = find_files_in_incomplete_directory(motor_controller_project_directory, "settings.yaml", subdir='motor-controller', silent=True)
    print(f"Using settings file: {settings_file}")
    with open(settings_file, 'r') as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue

            cleaned_line = line.replace(' ', '').replace('_', '-').replace('\n', '')
            if '#' in cleaned_line:
                cleaned_line = cleaned_line[:cleaned_line.index('#')]
            split_line = cleaned_line.split(':')

            if len(split_line) == 2 and split_line[0] and split_line[1]:
                if split_line[1].lower() == 'false':
                    split_line[1] = 0
                elif split_line[1].lower() == 'true':
                    split_line[1] = 1
                if split_line[0] in settings_list:
                    send_and_confirm_command(process, output_queue, split_line[0], split_line[1])
                else:
                    raise Exception(f"setting named {split_line[0]} from settings.yaml file not found in settings list. Current settings list:\n{COLOR_RESET}{settings_list}")


def setup_settings(process, output_queue):    
    process.stdin.write("settings reset\n")
    for command in CMDS.keys():
        val = CMDS[command][0]
        send_and_confirm_command(process, output_queue, command, val)

def isolate_mac_addr(mac_addr_line):
    """
    Isolate the script name from the full path.
    """
    if '|' in mac_addr_line:
        mac_addr_line = mac_addr_line.split('|')[-1]
    return mac_addr_line.strip()

def check_equal_mac_addr(mac_addr1, mac_addr2):
    """
    Check if two MAC addresses are equal based of hex value.
    """
    mac_addr1 = isolate_mac_addr(mac_addr1).replace(' ', '').replace("\n", '').lower().split(":")
    mac_addr2 = isolate_mac_addr(mac_addr2).replace(' ', '').replace("\n", '').lower().split(":")
    mac_addrs = [mac_addr1, mac_addr2]
    for i, addr in enumerate(mac_addrs):
        # deals with the line from pfr_ble_cli having the name of the device
        if len(addr) == 7:
            mac_addrs[i] = addr[1:]
        elif len(addr) != 6:
            return False
    mac_addr1, mac_addr2 = mac_addrs[0], mac_addrs[1]
    for i in range(len(mac_addr1)):
        try:
            if int(mac_addr1[i], 16) != int(mac_addr2[i], 16):
                return False
        except ValueError:
            raise Exception(f"Invalid MAC address format: either {mac_addr1[i]} or {mac_addr2[i]} is malformed.")
    return True


def connect_to_bluetooth_cli(process, output_queue):
    FOUND_ADAPTERS_LINES = ["found", "ble adapter"]
    ADAPTER_PROMPT = "select an adapter to use"
    SCAN_MSG = "starting 3 second scan"
    CONNECTED_MSG = "cli found"


    adapters_found_line = read_until_message(output_queue, FOUND_ADAPTERS_LINES, timeout=5)
    if adapters_found_line:
        num_adapters = int(adapters_found_line[adapters_found_line.index(FOUND_ADAPTERS_LINES[0]) + len(FOUND_ADAPTERS_LINES[0]):adapters_found_line.index(FOUND_ADAPTERS_LINES[1])].strip())
        selected_adapter = None 
        if read_until_message(output_queue, ADAPTER_PROMPT, timeout=2):
            if num_adapters == 1:
                print("\nSelecting the first adapter...")
                selected_adapter = 0
                # Automatically select the first adapter by sending a newline.
                process.stdin.write("\n")
                process.stdin.flush()
            else:
                while selected_adapter is None:
                    try: 
                        selected_adapter = int(input().strip())
                        process.stdin.write(f"{selected_adapter}\n")
                    except ValueError:
                        print("Please enter a valid number for the adapter selection.")
        else:
            process.terminate()
            raise Exception("Timeout waiting for adapter query...")

    
    # If the program is asking for a different input, we can let the user type it.
    # You would add more `elif` checks for other prompts you want to automate.
    if read_until_message(output_queue, SCAN_MSG, timeout=1):
        # read_lines = False
        bluetooth_confirmed = 0
        motor_in_list_to_connect = None
        print("Connecting brain board...\n")

        # Give the program an extra second to do the scan
        _, mac_addr, _ = read_bluetooth_status()
        if mac_addr:
            print(f"Using mac address to connect to motor controller...")
            motor_controller_output = read_until_message(output_queue, mac_addr, 4, compare_func=check_equal_mac_addr)
        else:
            print(f"{COLOR_YELLOW}Warning: No mac address found, using name to connect to motor controller.{COLOR_RESET}")
            motor_controller_output = read_until_message(output_queue, "motor_0_ble", 4)
        
        if motor_controller_output:
            motor_in_list_to_connect = int(motor_controller_output[motor_controller_output.index("{") + 1:motor_controller_output.index("}")].strip())
        else:
            process.terminate()
            raise Exception("No motors with correct MAC address found in bluetooth scan.")  
        
        if read_until_message(output_queue, "enter a peripheral index", 1):
            time.sleep(0.2)
            process.stdin.write(f"{motor_in_list_to_connect}\n")
        else:
            process.terminate()
            raise Exception("Timeout in prompt to enter desired motor. Make sure pfr_ble_cli is unchanged.")
        
        if read_until_message(output_queue, CONNECTED_MSG, 5, print_output=True):
            time.sleep(1)
            print("Connected to motor controller CLI, verifying connection...\n")
            process.stdin.write(f"ping\n")
            read_until_message(output_queue, "pong", 1)
            time.sleep(0.5)
            bluetooth_confirmed, _, _ = read_bluetooth_status()
            if bluetooth_confirmed == 1:
                print("Connection verified!\n")
                return True
            else:
                print("Connection to wrong device! Restarting...\n")
                return False
        else:
            print("Timeout in motor connection, restarting...\n")
            return False # connection timeout usually means it was just the wrong one
        
# here just so main() reads more nicely
def connect_to_zenoh_cli(output_queue):
    if read_until_message(output_queue, "motor found", timeout=5):
        print("Connected using zenoh cli\n")
        return True
    else:
        return False

def _is_port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False

def _launch_rmw_zenohd_background(host: str, port: int, extra_args=None):
    """
    Start `ros2 run rmw_zenoh_cpp rmw_zenohd` in the background **only if**
    nothing is already listening on host:port. Returns the Popen handle if we
    started it, or None if something is already up (or we couldn't start it).
    """
    if _is_port_open(host, port):
        print(f"Zenoh router already appears to be listening on {host}:{port}.")
        return None

    cmd = ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
    if extra_args:
        cmd += list(extra_args)

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # new process group (Unix)
        )
        print("Started rmw_zenohd in the background.")
        input("press RESET/power-cycle the motor controller so it re-announces over Zenoh. Press enter when done")
        # Give the router a moment to bind its sockets
        time.sleep(2.0)
        return proc
    except FileNotFoundError:
        print("ERROR: 'ros2' not found. Ensure ROS 2 is installed and on your PATH.")
    except Exception as e:
        print(f"ERROR: failed to start rmw_zenohd: {e}")
    return None

            
# This is necessary because pfr_ble_cli has a phantom motor_0_ble that freezes it, 
# so in order to make the timeout on the connection work you must thread output and quit
def start_process_and_output_thread(command, cwd):
    output_queue = queue.Queue()

    # This function will run in a separate thread
    def enqueue_output(out, queue):
        for line in iter(out.readline, ''):
            queue.put(line)

    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1 # makes it line-buffered, so we get output line by line.
    )

    thread = threading.Thread(target=enqueue_output, args=(process.stdout, output_queue), daemon=True)
    thread.start()

    return process, output_queue

def check_valid_zenoh_endpoint(ip_str):
    head_present = ""
    heads = ["tcp/", "udp/"]
    for head in heads:
        if head in ip_str:
            head_present = head
            break
    if not head_present:
        return False
    
    try:
        tail = ip_str.split(":")[1]
        if not tail or not tail.isdigit():
            return False
    except IndexError:
        return False
    
    ip_ints = ip_str.replace(head_present, "").split(":").split(".")
    if len(ip_ints) != 4:
        return False
    return True

def loop_connection_attempts(mode):
    if mode == "ble":
            process, output_queue = start_process_and_output_thread(ble_cmd, rust_project_directory)
            # Keep retrying connection
            while not connect_to_bluetooth_cli(process, output_queue):
                process, output_queue = start_process_and_output_thread(ble_cmd, rust_project_directory)
            print("Bluetooth connected successfully, adjusting settings")
    elif mode == "zenoh":
        # 1) Ensure a zenoh router is running (idempotent: only starts if port is free)
        _launch_rmw_zenohd_background(ipv4, ZENOH_PORT)

        max_tries = 3
        process, output_queue = start_process_and_output_thread(zenoh_cmd, pfr_software_directory)
        tries = 1
        # 2) Tell the user to reset, right away (helps discovery)
        print("If you don't see the CLI connect, press RESET/power-cycle the motor controller.")

        while not connect_to_zenoh_cli(output_queue) and tries < max_tries + 1:
            tries += 1
            if tries == max_tries + 1:
                raise Exception("Unable to connect. Make sure the motor controller is on Zenoh and rmw_zenohd is running.")
            print(f"\nRetrying connection to motor controller CLI over Zenoh {tries - 1}/{max_tries - 1}...")
            print("Tip: press RESET/power-cycle the motor controller, then wait a few seconds.")
            # (Re)spawn your CLI process for the next attempt
            process, output_queue = start_process_and_output_thread(zenoh_cmd, pfr_software_directory)
    return process, output_queue


def main(argv):
    try:
        instant_exit = False
        # Access flag values
        mode = flags.FLAGS.mode.lower().strip()
        reset_settings_flag = flags.FLAGS.reset_settings
        automate_commands = flags.FLAGS.automate_commands
        board_awaiting_wifi = flags.FLAGS.board_awaiting_wifi

        if mode == "ble" and not os.path.exists(BLUETOOTH_FILE_PATH):
            instant_exit = True
            raise Exception(f"Make sure that the serial monitor script is running to use ble connection mode in order to verify connection to device.")

        print(f"\n=== {"Resetting" if reset_settings_flag else "Setting"} settings using {mode} CLI ===")

        if not os.path.isdir(rust_project_directory):
            with open('r8/pfr-rust-nodes', 'w') as f:
                f.write(f"{COLOR_RED}ERROR: The directory '{rust_project_directory}' does not exist.{COLOR_RESET}")
            return

    
        process, output_queue = loop_connection_attempts(mode)

        if automate_commands:
            if reset_settings_flag:
                ip_str = input(f"Enter new endpoint for zenoh endpoint address. [default: {ROBOT_ZENOH_ENDPOINT}]: ")
                if not check_valid_zenoh_endpoint(ip_str):
                    print(f"{COLOR_YELLOW}Warning: Invalid zenoh endpoint address. Using default.{COLOR_RESET}")
                    ip_str = ROBOT_ZENOH_ENDPOINT
                reset_settings(process, output_queue, ip_str)
            else:
                setup_settings(process, output_queue)
            print("Settings adjusted successfully.")
        else:
            print("You can now enter commands in the CLI. Type 'exit' to quit.")
            while not output_queue.empty():
                output_queue.get(block=True, timeout=0.1)
            while True:
                command = input("CLI> ").strip()
                if command.lower() == "exit":
                    break
                process.stdin.write(f"{command}\n")
                process.stdin.flush()
                time.sleep(0.3)
                # I know this is super shit, but it works
                while not output_queue.empty():
                    while not output_queue.empty():
                        print(output_queue.get(block=True, timeout=0.1))

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}\nERROR: {e}{COLOR_RESET}")
    finally:
        if not instant_exit:
            input("\nPress enter to return: ")
        if 'process' in locals() and process.poll() is None:
            if board_awaiting_wifi:
                process.stdin.write('wifi-connect\n')
                time.sleep(0.2)
            process.kill()
            print("\nProcess was terminated.")
        sys.exit()

if __name__ == "__main__":
    app.run(main)
