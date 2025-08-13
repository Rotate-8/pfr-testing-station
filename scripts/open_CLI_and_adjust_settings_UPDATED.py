import os
import subprocess
import sys
from open_serial_monitor import read_bluetooth_status, BLUETOOTH_FILE_PATH
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
        "zenoh-endpoint": [f"tcp/{s.getsockname()[0]}:7447", f"tcp/:7447"],
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
            if print_output:
                print(output_line)

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
    if not read_until_message(output_queue, "set to", 0.3):
        process.stdin.write("settings help\n")
        settings_list = ""
        time.sleep(0.3)
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


def reset_settings(process, output_queue, zenoh_endpoint_ip):
    for command in CMDS.keys():
        if len(CMDS[command]) == 2:
            if command == "zenoh-endpoint":
                val = f'tcp/{zenoh_endpoint_ip}:7447'
                continue
            val = CMDS[command][1]
        else:
            continue
        send_and_confirm_command(process, output_queue, command, val)


def setup_settings(process, output_queue):    
    for command in CMDS.keys():
        val = CMDS[command][0]
        send_and_confirm_command(process, output_queue, command, val)


def isolate_mac_addr(script_name):
    """
    Isolate the script name from the full path.
    """
    return script_name.split("/")[-1] if "/" in script_name else script_name

def check_equal_mac_addr(mac_addr1, mac_addr2):
    """
    Check if two MAC addresses are equal based of hex value.
    """
    mac_addr1 = mac_addr1.replace(' ', '').replace("\n", '').lower().split(":")
    mac_addr2 = mac_addr2.replace(' ', '').replace("\n", '').lower().split(":")
    mac_addrs = [mac_addr1, mac_addr2]
    for i, addr in enumerate(mac_addrs):
        # deals with the line from pfr_ble_cli having the name of the device
        if len(addr) == 7:
            mac_addrs[i] = addr[1:]
        elif len(addr) != 6:
            return False
    mac_addr1, mac_addr2 = mac_addrs[0], mac_addrs[1]
    print(f"Comparing {mac_addr1} and {mac_addr2}")
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
        _, mac_addr = read_bluetooth_status()
        if mac_addr:
            print(f"Using mac address {mac_addr} to connect to motor controller...")
            motor_controller_output = read_until_message(output_queue, mac_addr, 4, print_output=True, compare_func=check_equal_mac_addr)
        else:
            print(f"{COLOR_YELLOW}Warning: No mac address found, using name to connect to motor controller.{COLOR_RESET}")
            motor_controller_output = read_until_message(output_queue, "motor_0_ble", 4, print_output=True)
        
        if motor_controller_output:
            motor_in_list_to_connect = int(motor_controller_output[motor_controller_output.index("{") + 1:motor_controller_output.index("}")].strip())
        else:
            process.terminate()
            raise Exception("No motors with correct MAC address found in bluetooth scan.")  
        
        if read_until_message(output_queue, "enter a peripheral index", 1):
            time.sleep(0.2)
            print(f"Entering ble device {motor_in_list_to_connect} into prompt...")
            process.stdin.write(f"{motor_in_list_to_connect}\n")
        else:
            process.terminate()
            raise Exception("Timeout in prompt to enter desired motor. Make sure pfr_ble_cli is unchanged.")
        
        if read_until_message(output_queue, CONNECTED_MSG, 5, print_output=True):
            time.sleep(1)
            print("Connected to motor controller CLI, verifying connection...\n")
            process.stdin.write(f"ping\n")
            time.sleep(1)
            bluetooth_confirmed = read_bluetooth_status()
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
        bufsize=1 # This makes it line-buffered, so we get output line by line.
    )

    thread = threading.Thread(target=enqueue_output, args=(process.stdout, output_queue), daemon=True)
    thread.start()

    return process, output_queue

def convert_to_zenoh_ip(ip_str):
    head = "tcp/"
    if head in ip_str:
        ip_str = ip_str[ip_str.index(head) + len(head)]
    tail = ":7447"
    if tail in ip_str:
        ip_str = ip_str[ip_str.index(tail)]
    
    ip_ints = ip_str.split(".")
    if len(ip_ints) == 4:
        try:
            ip_ints = [int(v) for v in ip_ints]
        except ValueError:
            return "0.0.0.0"
        return ip_str
    else:
        return "0.0.0.0"


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



def _prompt_user_mode():
    while True:
        try:
            choice = input(
                "\nChoose an option:\n"
                "  [A] Run automated settings flow (set/reset)\n"
                "  [M] Explore manually in the CLI\n"
                "  [Q] Quit\n"
                "Enter choice (A/M/Q): "
            ).strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "q"
        if choice in ("a", "m", "q"):
            return choice
        print("Please enter A, M, or Q.")

def _drain_and_print_queue(q, duration_sec=0.75):
    """Print any CLI output that appears within duration_sec (non-blocking)."""
    end = time.time() + duration_sec
    printed_any = False
    while time.time() < end:
        try:
            line = q.get(timeout=0.1)
            # Avoid double newlines; CLI may include '\n'
            print(line, end="" if line.endswith("\n") else "\n")
            printed_any = True
        except queue.Empty:
            pass
    if not printed_any:
        try:
            line = q.get(timeout=0.2)
            print(line, end="" if line.endswith("\n") else "\n")
        except queue.Empty:
            pass

def _interactive_cli_loop(process, output_queue, run_automation_cb):
    """
    Minimal pass-through shell:
      - Send what you type to the CLI process.
      - Briefly tail and print output after each command.
      - Special commands:
          :automation  -> run the provided automation callback
          :quit / :q   -> exit manual mode
          :help        -> show this help
    """
    print(
        "\n--- Manual CLI mode ---\n"
        "Type CLI commands and press Enter. Special commands:\n"
        "  :automation   run automated settings now\n"
        "  :quit or :q   exit\n"
        "  :help         show this help\n"
    )
    while True:
        try:
            user_line = input("cli> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting manual mode.")
            break

        if user_line in (":quit", ":q"):
            break
        if user_line == ":help":
            print("Commands: :automation, :quit, :q, :help — otherwise text is sent to the CLI.")
            continue
        if user_line == ":automation":
            run_automation_cb()
            _drain_and_print_queue(output_queue, duration_sec=1.0)
            continue

        if process.poll() is not None:
            print("CLI process ended.")
            break
        try:
            process.stdin.write(user_line + "\n")
            process.stdin.flush()
        except BrokenPipeError:
            print("CLI process is no longer available.")
            break

        _drain_and_print_queue(output_queue, duration_sec=1.0)


def main(argv):
    instant_exit = False
    # Access flag values
    mode = flags.FLAGS.mode.lower().strip()
    reset_settings_flag = flags.FLAGS.reset_settings

    print(f"=== {"Resetting" if reset_settings_flag else "Setting"} settings using {mode} CLI ===")

    if not os.path.isdir(rust_project_directory):
        with open('r8/pfr-rust-nodes', 'w') as f:
            f.write(f"{COLOR_RED}ERROR: The directory '{rust_project_directory}' does not exist.{COLOR_RESET}")
        return

    try:
        process, output_queue = loop_connection_attempts(mode)

        # Ask the user whether to automate or explore manually
        choice = _prompt_user_mode()
        if choice == "q":
            print("Exiting without changes.")
            return

        def _run_automation():
            if reset_settings_flag:
                ip_str = input("Enter new ip for zenoh endpoint (Ipv4 address only), or leave blank to set placeholder: ")
                ip_str = convert_to_zenoh_ip(ip_str)
                reset_settings(process, output_queue, ip_str)
            else:
                setup_settings(process, output_queue)

        if choice == "a":
            _run_automation()
            print("Settings adjusted successfully.")
        else:
            _interactive_cli_loop(process, output_queue, run_automation_cb=_run_automation)

    except KeyboardInterrupt:
        instant_exit = True
    except Exception as e:
        print(f"{COLOR_RED}\nERROR: {e}{COLOR_RESET}")
    finally:
        # A good practice is to make sure the process is closed,
        # even if an error occurs.
        if 'process' in locals() and process.poll() is None:
            process.kill()
            print("\nProcess was terminated.")
        if not instant_exit:
            input("\nPress enter to return: ")
        sys.exit()

if __name__ == "__main__":
    app.run(main)