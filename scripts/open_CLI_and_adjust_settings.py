import os
import subprocess
import sys
from open_serial_monitor import BLUETOOTH_FILE_PATH, read_bluetooth_status, write_bluetooth_status
import time
import socket
import queue
import threading
from absl import app
from absl import flags

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

rust_project_directory = "/r8/pfr-rust-nodes1"
ble_cmd = ["cargo", "run", "-p", "pfr_ble_cli"]

pfr_software_directory = "/r8/pfr-software"
zenoh_cmd = ["ros2", "run", "pfr_tools", "motor_controller_cli"]

COLOR_RED = '\033[91m'
COLOR_YELLOW = '\033[93m'
COLOR_RESET = '\033[0m'


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
CMDS = {
        "zenoh-endpoint": [f"tcp/{s.getsockname()[0]}:7447", f"udp/10.50.50.1:7777"],
        "wifi-ssid": ["twistedfields"],
        "wifi-pass": ["alwaysbekind"],
        "steer-zero-on-boot": ["0", "1"],
        "drive-motor-control-type": ["torque", "linear velocity"],
        "steer-motor-control-type": ["torque", "position"]
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
        process.stdin.write("settings help\n")
        settings_list = ""
        time.sleep(0.5)
        while not output_queue.empty():
            # print(output_queue.get(block=True, timeout=0.1))
            curr_line = output_queue.get(block=True, timeout=0.1)
            if curr_line == "\n":
                continue
            unshared_chars = count_unshared_characters(setting, curr_line)
            if unshared_chars > 0 and unshared_chars < 4:
                raise Exception(f"Failed to set {setting} to {val}. Make sure setting hasn't been changed to {curr_line.replace("\n", '')}.")
            settings_list += curr_line
        print("Finished adding settings to settings list")
        raise Exception(f"Failed to set {setting} to {val}. Current list of available settings:{COLOR_RESET}\n{settings_list}")


def reset_settings(process, output_queue, zenoh_endpoint_addr=None):
    for command in CMDS.keys():
        if len(CMDS[command]) == 2:
            if zenoh_endpoint_addr and command == "zenoh-endpoint":
                val = zenoh_endpoint_addr
                continue
            else:
                val = CMDS[command][1]
        else:
            continue
        send_and_confirm_command(process, output_queue, command, val)


def setup_settings(process, output_queue):    
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
        max_tries = 3
        process, output_queue = start_process_and_output_thread(zenoh_cmd, pfr_software_directory)
        tries = 1
        while not connect_to_zenoh_cli(output_queue) and tries < max_tries + 1:
            tries += 1
            if tries == max_tries + 1:
                raise Exception("Unable to connect. Make sure motor controller is connected over zenoh.")
            print(f"\nRetrying connection to motor controller CLI over zenoh {tries - 1}/{max_tries - 1}...")
            process, output_queue = start_process_and_output_thread(zenoh_cmd, pfr_software_directory)
    else:
        raise Exception("Invalid CLI connection method passed to CLI setting adjusetment script.")
    
    return process, output_queue


def motor_controller_test_ready(process, output_queue):
    """
    Function to run the motor controller test configuration.
    It will open a tmux session and run the test script.
    """
    for command in CMDS.keys():
        process.stdin.write(f"settings {command}\n")
        time.sleep(0.2)
        while not output_queue.empty():
            curr_line = output_queue.get(block=True, timeout=0.1).lower()
            if CMDS[command][0] not in curr_line or CMDS[command][0] == 0 and "false" not in curr_line:
                return False
    return True
 

def main(argv):
    try:
        instant_exit = False
        # Access flag values
        mode = flags.FLAGS.mode.lower().strip()
        reset_settings_flag = flags.FLAGS.reset_settings
        automate_commands = flags.FLAGS.automate_commands

        if mode == "ble" and not os.path.exists(BLUETOOTH_FILE_PATH):
            instant_exit = True
            raise Exception(f"Make sure that the serial monitor script is running to use ble connection mode in order to verify connection to device.")

        print(f"=== {"Resetting" if reset_settings_flag else "Setting"} settings using {mode} CLI ===")

        if not os.path.isdir(rust_project_directory):
            with open('r8/pfr-rust-nodes', 'w') as f:
                f.write(f"{COLOR_RED}ERROR: The directory '{rust_project_directory}' does not exist.{COLOR_RESET}")
            return

    
        process, output_queue = loop_connection_attempts(mode)

        if automate_commands:
            if reset_settings_flag:
                ip_str = input("Enter new for zenoh endpoint address. [default: udp/10.50.50.1:7777]: ")
                if not check_valid_zenoh_endpoint(ip_str):
                    print(f"{COLOR_YELLOW}Warning: Invalid zenoh endpoint address. Using default.{COLOR_RESET}")
                reset_settings(process, output_queue, ip_str)
            else:
                setup_settings(process, output_queue)
            print("Settings adjusted successfully.")
        else:
            print("You can now enter commands in the CLI. Type 'exit' to quit.")
            # clear queue before entering CLI commands
            while not output_queue.empty():
                output_queue.get(block=True, timeout=0.1)
            while True:
                command = input("CLI> ").strip()
                if command.lower() == "exit":
                    break
                process.stdin.write(f"{command}\n")
                process.stdin.flush()
                time.sleep(0.2)
                while not output_queue.empty():
                    print(output_queue.get(block=True, timeout=0.1))

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}\nERROR: {e}{COLOR_RESET}")
    finally:
        # A good practice is to make sure the process is closed,
        # even if an error occurs.
        if not instant_exit:
            if motor_controller_test_ready(process, output_queue):
                if os.path.exists(BLUETOOTH_FILE_PATH):
                    write_bluetooth_status(test_settings_applied=1)
            input("\nPress enter to return: ")
        if 'process' in locals() and process.poll() is None:
            process.kill()
            print("\nProcess was terminated.")
        sys.exit()

if __name__ == "__main__":
    app.run(main)
