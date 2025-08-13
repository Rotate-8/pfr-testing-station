import curses
import sys
import subprocess
import os

# Define the menu options and their corresponding placeholder actions

menu_items = [
    "Upload and monitor code on ESP32",
    "Test motor control stack",
    "Open Motor Controller CLI",
    "Reassign encoder I2C address",
    "Exit to terminal"
]

 # Adjust this path as needed
SCRIPTS_DIR = "scripts"

SERIAL_MONITOR_SCRIPT = "open_serial_monitor.py"
FLASH_CODE_SCRIPT =  "flash_code_and_monitor.py"
TEST_STACK_SCRIPT = "test_stack.py"
BURN_ADDR_SCRIPT = "burn_addr.py"
CLONE_AND_BUILD_SCRIPT = "clone_and_build.py"
CLI_AUTOMATION_SCRIPT = "open_CLI_and_adjust_settings.py"
OPEN_BLE_SCRIPT = "open_CLI_and_adjust_settings_UPDATED.py "

scripts_list = [
    SERIAL_MONITOR_SCRIPT,
    FLASH_CODE_SCRIPT,
    TEST_STACK_SCRIPT,
    BURN_ADDR_SCRIPT,
    CLONE_AND_BUILD_SCRIPT,
    OPEN_BLE_SCRIPT
]

def run_external_script(stdscr, script_name):
    """
    Helper function to run an external Python script.
    It temporarily exits curses mode, runs the script, then re-enters curses mode.
    """
    stdscr.refresh()
    curses.endwin()
    os.system('clear') # clear terminal ahead of calling subprocess
    if "monitor" in script_name:
        if "TMUX" not in os.environ:
            session_name = "serial_monitor_session"
            tmux_cmd = (
                f'tmux new-session -d -s {session_name} "bash -c \\"clear; echo \'Click Ctrl+B followed by arrow keys to navigate windows. \nPress Ctrl+C in script window to return to menu.\'; exec bash\\"" && '
                f'tmux split-window -h -t {session_name} "cd {SCRIPTS_DIR} && python3 {script_name}; tmux kill-session -t {session_name}" && '
                f'tmux select-pane -t {session_name}:0.1 && '
                f'tmux attach-session -t {session_name}'
            )
            os.system(tmux_cmd)
        else:
            message = "printf 'Click Ctrl+B followed by left arrow to interact with this pane.\\n' > /dev/tty"
            tmux_cmd = (
                f'tmux split-window -h "cd {SCRIPTS_DIR} && python3 {script_name}; tmux kill-session" && '
                f'tmux send-keys -t 0 "{message}" Enter && '
                f'tmux select-pane -R'
            )
            subprocess.run(tmux_cmd, shell=True)
    elif script_name == TEST_STACK_SCRIPT:
        session_name = "serial_monitor_session"
        tmux_cmd = (
            f'tmux new-session -d -s {session_name} "cd {SCRIPTS_DIR} && python3 {script_name}; tmux kill-session -t {session_name}" && '
            f'tmux split-window -h -t {session_name} "cd {SCRIPTS_DIR} && python3 {SERIAL_MONITOR_SCRIPT}; tmux kill-session -t {session_name}" && '
            f'tmux select-pane -t {session_name}:0.0 && '
            f'tmux attach-session -t {session_name}'
        )
        os.system(tmux_cmd)
    else:

        # Execute the external Python script
        try:
            subprocess.run(["python3", script_name], cwd=SCRIPTS_DIR, check=True)
        # check=False means it won't raise an exception for non-zero exit codes,
        # allowing you to inspect result.returncode if needed.
        except KeyboardInterrupt:
            pass
        # Re-enter curses mode
    stdscr.refresh()


def main(stdscr):
    """
    Main function for the curses application.
    Initializes curses, handles menu navigation, and executes selected options.
    """
    # Initialize curses settings
    curses.curs_set(0)  # Hide the cursor
    stdscr.keypad(True) # Enable special keys (like arrow keys)
    stdscr.nodelay(False) # Wait for input
    stdscr.clear()

    # Check for color support and initialize colors
    if curses.has_colors():
        curses.start_color()
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE) # For selected item
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLACK) # Default text

    current_row_idx = 0 # Index of the currently highlighted menu item

    # Main loop for menu interaction
    while True:
        stdscr.clear() # Clear the screen for each redraw

        # Get screen dimensions
        h, w = stdscr.getmaxyx()

        # Display title
        title = "Motor Controller Workstation"
        x_title = w // 2 - len(title) // 2
        y_title = h // 2 - len(menu_items) // 2 - 2 # Position above menu
        stdscr.addstr(y_title, x_title, title, curses.A_BOLD)

        # Display menu items
        for idx, item in enumerate(menu_items):
            x = w // 2 - len(item) // 2
            y = h // 2 - len(menu_items) // 2 + idx
            if idx == current_row_idx:
                # Highlight the selected item
                if curses.has_colors():
                    stdscr.attron(curses.color_pair(1))
                stdscr.addstr(y, x, item)
                if curses.has_colors():
                    stdscr.attroff(curses.color_pair(1))
            else:
                # Default text color
                if curses.has_colors():
                    stdscr.attron(curses.color_pair(2))
                stdscr.addstr(y, x, item)
                if curses.has_colors():
                    stdscr.attroff(curses.color_pair(2))

        stdscr.refresh() # Update the screen

        # Get user input
        key = stdscr.getch()

        # Handle navigation keys
        if key == curses.KEY_UP:
            current_row_idx = max((current_row_idx - 1), 0)
        elif key == curses.KEY_DOWN:
            current_row_idx = min((current_row_idx + 1), len(menu_items) - 1)
        elif key == curses.KEY_ENTER or key in [10, 13]: # Enter key
            # Execute the selected action
            if current_row_idx <= 5:
                stdscr.clear()
                run_external_script(stdscr, scripts_list[current_row_idx])
            else:
                break
            

# Entry point for the script
if __name__ == '__main__':

    try:
        # Wrapper function to safely initialize and deinitialize curses
        curses.wrapper(main)
    except curses.error as e:
        print(f"Curses error: {e}")
        print("This script requires a terminal that supports curses.")
        print("Try running it in a standard Linux terminal or within 'screen'.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        sys.exit(1)
    finally:
        os.system('clear')
        print("Call 'python3 start_menu.py' to restart the menu.")
        sys.stdout.flush() # IMPORTANT: Flush the output buffer
        sys.exit(0)
