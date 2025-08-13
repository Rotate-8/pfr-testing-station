#!/usr/bin/env python3
import curses
from curses import textpad
import sys
import subprocess
import os

# ---------------------------
# Menu model (kept from your file, with small fixes)
# ---------------------------

initial_menu = [
    "Flash and Test ESP32 with auto BLE",  # index 0 -> dropdown (menu_items_1)
    "Reassign sensor I2C address",         # index 1 -> direct action
    "More",                             # index 2 -> dropdown (menu_items_2)
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
# NOTE: sub_idx here refers to the ORIGINAL submenu index (0-based), not including the "Back" row.
submenu_scripts = {
    (0, 0): "flash_code_and_monitor.py",
    (0, 1): "test_stack.py",
    (2, 0): "open_serial_monitor.py",
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
    if script_name == "exit":
        # Treat "Exit" entries in submenus as exit of the whole program
        sys.exit(0)

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
# UI helpers (reworked: submenu opens in its own panel)
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

def _attr_for(stdscr, selected: bool):
    if curses.has_colors():
        return curses.color_pair(1) if selected else curses.color_pair(2)
    # Fallback: reverse for selected, normal otherwise
    return curses.A_REVERSE if selected else curses.A_NORMAL

def _draw_main(stdscr, current_top_idx):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    title = "Motor Controller Workstation"
    y_title = max(1, h // 2 - len(initial_menu) // 2 - 3)
    x_title = max(0, w // 2 - len(title) // 2)
    stdscr.addstr(y_title, x_title, title, curses.A_BOLD)

    

    top_start_y = y_title + 3
    for idx, item in enumerate(initial_menu):
        has_dropdown = idx in submenus and len(submenus[idx]) > 0
        suffix = " ▶" if has_dropdown else ""
        label = item + suffix
        y = top_start_y + idx
        x = max(2, w // 2 - len(label) // 2)
        stdscr.addstr(y, x, label, _attr_for(stdscr, selected=(idx == current_top_idx)))

    stdscr.refresh()

def _draw_submenu_panel(stdscr, top_idx, current_sub_display_idx):
    """
    Draws a centered panel for the submenu:
      Row 0 is always '← Back to Main Menu' (not part of submenu_scripts mapping).
      The rest mirror submenus[top_idx].
    """
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    header = f"{initial_menu[top_idx]} — Submenu"
    items = ["← Back to Main Menu"] + submenus.get(top_idx, [])

    # Compute panel size based on content
    max_len = max(len(s) for s in items + [header])
    pad_w = 6
    pad_h = 6
    box_w = min(w - 4, max_len + pad_w)
    box_h = min(h - 4, len(items) + pad_h)

    start_y = (h - box_h) // 2
    start_x = (w - box_w) // 2
    end_y = start_y + box_h - 1
    end_x = start_x + box_w - 1

    # Draw border rectangle
    textpad.rectangle(stdscr, start_y, start_x, end_y, end_x)

    # Title centered
    title_y = start_y + 1
    stdscr.addstr(title_y, start_x + (box_w - len(header)) // 2, header, curses.A_BOLD)

    # Help line
    

   # Items (centered)
    list_start_y = title_y + 3
    inner_w = box_w - 4  # inside the border (2px padding on each side)

    for i, s in enumerate(items):
        y = list_start_y + i
        if y >= end_y:
            break

        # Build the display label (no left padding; center it instead)
        display = "← Back to Main Menu" if i == 0 else f"{s}"

        # Truncate if too long for the inner width
        if len(display) > inner_w:
            display = display[:inner_w]

    # Center within the inner box
        x = start_x + 2 + (inner_w - len(display)) // 2

        stdscr.addstr(
            y, x, display,
            _attr_for(stdscr, selected=(i == current_sub_display_idx))
        )


    stdscr.refresh()

def _wrap(idx, n):
    return (idx + n) % n if n else 0

# ---------------------------
# Curses main
# ---------------------------

def main(stdscr):
    """
    Curses UI with separate-panel submenus for indices 0 and 2.

    Controls:
      Up/Down: move selection (wraps)
      Enter/Right: open submenu (if available) or run top-level action
      Left/Esc/Backspace: close submenu panel and return to main
      Enter in submenu: run submenu action (row 0 is Back)
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
    current_sub_display_idx = 0  # includes "Back" at index 0
    in_submenu = False

    def open_submenu_if_available():
        nonlocal in_submenu, current_sub_display_idx
        if current_top_idx in submenus and submenus[current_top_idx]:
            in_submenu = True
            current_sub_display_idx = 0  # default to Back row
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
        """
        current_sub_display_idx = 0 means Back to Main.
        Otherwise map to original submenu index (display_idx - 1).
        """
        if current_sub_display_idx == 0:
            return "back"

        original_idx = current_sub_display_idx - 1
        script = submenu_scripts.get((current_top_idx, original_idx))
        if script == "exit":
            return "exit"
        if isinstance(script, str) and script:
            run_external_script(stdscr, script)
        return None

    # Main loop
    while True:
        if not in_submenu:
            _draw_main(stdscr, current_top_idx)
        else:
            _draw_submenu_panel(stdscr, current_top_idx, current_sub_display_idx)

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
            submenu_len = 1 + len(submenus.get(current_top_idx, []))  # 1 for Back row
            if key == curses.KEY_UP:
                current_sub_display_idx = _wrap(current_sub_display_idx - 1, submenu_len)
            elif key == curses.KEY_DOWN:
                current_sub_display_idx = _wrap(current_sub_display_idx + 1, submenu_len)
            elif key in (curses.KEY_LEFT, 27, curses.KEY_BACKSPACE, 127, 8):  # Left/Esc/Backspace
                in_submenu = False
            elif key in (curses.KEY_ENTER, 10, 13):
                result = run_submenu_action()
                if result == "back":
                    in_submenu = False
                elif result == "exit":
                    break
                # Otherwise stay in submenu after running the script

# Entry point
if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except curses.error as e:
        print(f"Curses error: {e}")
        print("This script requires a terminal that supports curses.")
        print("Try running it in a standard Linux terminal or within 'screen'.")
    except SystemExit:
        # Allow clean exits triggered by submenu "Exit"
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        sys.exit(1)
    finally:
        os.system('clear')
        print("Call 'python3 start_menu.py' to restart the menu.")
        sys.stdout.flush()  # IMPORTANT: Flush the output buffer
        sys.exit(0)
