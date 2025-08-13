import curses
import sys
import subprocess
import os

# ---------------------------
# Menu model (kept from your file, with small fixes)
# ---------------------------

initial_menu = [
    "Flash and Test ESP32 with auto BLE",  # index 0 -> dropdown (menu_items_1)
    "Reassign sensor I2C address",         # index 1 -> direct action
    "More...",                             # index 2 -> dropdown (menu_items_2)
    "Exit"                                 # index 3 -> exit
]

menu_items_1 = [
    "Upload and monitor code on ESP32",
    "Test motor control stack"
]

menu_items_2 = [
    "Open serial monitor for ESP32",
    "Open Bluetooth CLI for Motor Controller"
]

# Dropdowns: which top index has which submenu list
submenus = {
    0: menu_items_1,
    2: menu_items_2
}

# Top-level direct actions
# 1 -> burn i2c; 3 -> exit
top_actions = {
    1: "burn_addr.py",
    3: "exit"
}

# Map (top_idx, sub_idx) -> script filename (bare names so tmux/cwd logic works)
submenu_scripts = {
    (0, 0): "flash_code_and_monitor.py",
    (0, 1): "test_stack.py",
    (2, 0): "open_CLI_and_adjust_settings.py",  # (label says serial monitor; mapping kept as you set)
    (2, 1): "open_ble.py"
}

# Adjust this path as needed
SCRIPTS_DIR = "scripts"

# (Keeping your named constants; used by run_external_script logic)
SERIAL_MONITOR_SCRIPT = "open_serial_monitor.py"
FLASH_CODE_SCRIPT =  "flash_code_and_monitor.py"
TEST_STACK_SCRIPT = "test_stack.py"
BURN_ADDR_SCRIPT = "burn_addr.py"
CLONE_AND_BUILD_SCRIPT = "clone_and_build.py"
CLI_AUTOMATION_SCRIPT = "open_CLI_and_adjust_settings.py"
OPEN_BLE_SCRIPT = "open_ble.py"

scripts_list = [
    SERIAL_MONITOR_SCRIPT,
    FLASH_CODE_SCRIPT,
    TEST_STACK_SCRIPT,
    BURN_ADDR_SCRIPT,
    CLONE_AND_BUILD_SCRIPT,
    OPEN_BLE_SCRIPT
]

# ---------------------------
# Execution helpers (kept behavior)
# ---------------------------

def run_external_script(stdscr, script_name: str):
    """
    Helper to run an external Python script.
    Keeps your tmux behavior intact:
      - 'monitor' scripts open a split tmux session
      - test_stack opens 2-pane tmux (left: test, right: serial monitor)
      - everything else runs via python3 with cwd=SCRIPTS_DIR
    """
    stdscr.refresh()
    curses.endwin()
    os.system('clear')  # clear terminal ahead of calling subprocess

    # IMPORTANT: tmux branches expect bare filenames and cwd=SCRIPTS_DIR
    # We normalized submenu_scripts to bare filenames so this works unchanged.

    if "monitor" in script_name:
        # serial/flash monitor behavior (tmux panes)
        if "TMUX" not in os.environ:
            session_name = "serial_monitor_session"
            tmux_cmd = (
                f'tmux new-session -d -s {session_name} '
                f'"bash -c \\"clear; echo \'Click Ctrl+B followed by arrow keys to navigate windows. '
                f'\\nPress Ctrl+C in script window to return to menu.\'; exec bash\\"" && '
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
        # two-pane tmux: left test, right serial monitor
        session_name = "serial_monitor_session"
        tmux_cmd = (
            f'tmux new-session -d -s {session_name} "cd {SCRIPTS_DIR} && python3 {script_name}; tmux kill-session -t {session_name}" && '
            f'tmux split-window -h -t {session_name} "cd {SCRIPTS_DIR} && python3 {SERIAL_MONITOR_SCRIPT}; tmux kill-session -t {session_name}" && '
            f'tmux select-pane -t {session_name}:0.0 && '
            f'tmux attach-session -t {session_name}'
        )
        os.system(tmux_cmd)

    else:
        # Generic execution: run inside scripts dir
        try:
            subprocess.run(["python3", script_name], cwd=SCRIPTS_DIR, check=False)
        except KeyboardInterrupt:
            pass

    # Re-enter curses mode
    stdscr.refresh()

# ---------------------------
# UI helpers
# ---------------------------

def _validate_wiring():
    # Ensure every submenu item has a script mapping and no extra mappings exist
    for top_idx, items in submenus.items():
        for j in range(len(items)):
            if (top_idx, j) not in submenu_scripts:
                raise ValueError(f"Missing script mapping for submenu item ({top_idx}, {j})")
    for (ti, sj) in submenu_scripts:
        if ti not in submenus or sj >= len(submenus[ti]):
            raise ValueError(f"Unreachable submenu mapping: ({ti}, {sj})")
    for ti in top_actions:
        if ti < 0 or ti >= len(initial_menu):
            raise ValueError(f"top_actions index out of range: {ti}")

def _draw_text(stdscr, y, x, text, selected=False):
    if curses.has_colors():
        stdscr.attron(curses.color_pair(1 if selected else 2))
    stdscr.addstr(y, x, text)
    if curses.has_colors():
        stdscr.attroff(curses.color_pair(1 if selected else 2))

def _draw_ui(stdscr, current_top_idx, in_submenu, current_sub_idx):
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # Title
    title = "Motor Controller Workstation"
    y_title = max(1, h // 2 - len(initial_menu) // 2 - 2)
    x_title = max(0, w // 2 - len(title) // 2)
    stdscr.addstr(y_title, x_title, title, curses.A_BOLD)

    # Top-level menu
    top_start_y = y_title + 2
    for idx, item in enumerate(initial_menu):
        has_dropdown = idx in submenus and len(submenus[idx]) > 0
        suffix = " ▶" if has_dropdown else ""
        label = item + suffix
        y = top_start_y + idx
        x = max(1, w // 2 - len(label) // 2)
        _draw_text(stdscr, y, x, label, selected=(idx == current_top_idx and not in_submenu))

    # Dropdown panel
    if in_submenu and current_top_idx in submenus:
        items = submenus[current_top_idx]
        top_item_y = top_start_y + current_top_idx
        x_anchor = max(1, w // 2 - len(initial_menu[current_top_idx]) // 2)

        drop_start_y = top_item_y + 1
        if drop_start_y + len(items) >= h - 1:
            drop_start_y = max(1, top_item_y - len(items) - 1)

        for j, s in enumerate(items):
            y = drop_start_y + j
            x = x_anchor
            label = f"  • {s}"
            _draw_text(stdscr, y, x, label, selected=(j == current_sub_idx))

    stdscr.refresh()

def _wrap(idx, n):
    return (idx + n) % n if n else 0

# ---------------------------
# Curses main
# ---------------------------

def main(stdscr):
    """
    Curses UI with dropdown submenus for indices 0 and 2.

    Controls:
      Up/Down: move selection (wraps)
      Enter/Right: open submenu (if available) or run top-level action
      Left/Esc/Backspace: close submenu
      Enter in submenu: run submenu action
      q: quit
    """
    _validate_wiring()

    # Initialize curses settings
    curses.curs_set(0)   # Hide the cursor
    stdscr.keypad(True)  # Enable special keys (like arrow keys)
    stdscr.nodelay(False)
    stdscr.clear()

    # Colors
    if curses.has_colors():
        curses.start_color()
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)  # selected
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLACK)  # normal

    current_top_idx = 0
    current_sub_idx = 0
    in_submenu = False

    def open_submenu_if_available():
        nonlocal in_submenu, current_sub_idx
        if current_top_idx in submenus and submenus[current_top_idx]:
            in_submenu = True
            current_sub_idx = 0
            return True
        return False

    def run_top_action():
        action = top_actions.get(current_top_idx)
        if action == "exit":
            return "exit"
        if isinstance(action, str) and action:
            run_external_script(stdscr, action)
        return None

    def run_submenu_action():
        items = submenus.get(current_top_idx, [])
        if not items:
            return
        script = submenu_scripts.get((current_top_idx, current_sub_idx))
        if isinstance(script, str) and script:
            run_external_script(stdscr, script)

    # Main loop
    while True:
        _draw_ui(stdscr, current_top_idx, in_submenu, current_sub_idx)
        key = stdscr.getch()

        if key in (ord('q'),):
            break

        if not in_submenu:
            if key == curses.KEY_UP:
                current_top_idx = _wrap(current_top_idx - 1, len(initial_menu))
            elif key == curses.KEY_DOWN:
                current_top_idx = _wrap(current_top_idx + 1, len(initial_menu))
            elif key in (curses.KEY_RIGHT, curses.KEY_ENTER, 10, 13):
                # Try to open submenu; otherwise run top-level action
                if not open_submenu_if_available():
                    result = run_top_action()
                    if result == "exit":
                        break
        else:
            if key == curses.KEY_UP:
                current_sub_idx = _wrap(current_sub_idx - 1, len(submenus[current_top_idx]))
            elif key == curses.KEY_DOWN:
                current_sub_idx = _wrap(current_sub_idx + 1, len(submenus[current_top_idx]))
            elif key in (curses.KEY_LEFT, 27, curses.KEY_BACKSPACE, 127, 8):
                in_submenu = False
            elif key in (curses.KEY_ENTER, 10, 13):
                run_submenu_action()
                # Stay open after action; set `in_submenu = False` if you prefer auto-close

# Entry point
if __name__ == '__main__':
    try:
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
        sys.stdout.flush()  # IMPORTANT: Flush the output buffer
        sys.exit(0)
