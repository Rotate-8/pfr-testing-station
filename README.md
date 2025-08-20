
# PFR Testing Station & Encoder I2C Address Reassigner

A Raspberry Pi–powered test station for automating ESP32-S3 firmware flashing, settings configuration, interactive I²C address reassignment, and motor controller testing.

---


## Table of Contents
1. [Setup](#setup)

	 1.1 [Hardware Requirements](#hardware-requirements)

	 1.2 [Software Setup](#software-setup)

	 1.3 [Pi Configuration](#pi-configuration)

	 1.4 [Firmware Flashing](#firmware-flashing)

2. [User Interface](#user-interface)

	2.1 [Wiring](#wiring)

	2.2 [Main Menu](#main-menu)

	2.3 [Flashing and Testing Submenu](#flashing-and-testing)

	2.4 [Encoder I2C Address Reassignment](#reassign-sensor-i2c-address)

	2.5 [More Menu](#more-menu)


---

## Setup

### Hardware Requirements
**Materials:**
1. Raspberry Pi
2. Keyboard & Monitor
3. FTDI Cable
4. Power adapter with Anderson connectors

### Software Setup
1. **Raspberry Pi Bringup:**  
	Follow the [Raspberry Pi bringup steps](https://app.clickup.com/9014308583/v/dc/8cmpvq7-594/8cmpvq7-3234).

2. **Install ROS packages:**  
	```bash
	cd /r8/pfr-sofware
	packages-up-to pfr_control
	```

3. **Clone pfr-rust-nodes repository:**
	```bash
	cd /r8
	git clone https://github.com/Rotate-8/pfr-rust-nodes.git
	cd pfr-rust-nodes
	git checkout version/0.4.0
	cd ..
	```
	In order to work with this repo pfr-rust-nodes must be edited. Line 104 of main.rs in pfr_ble_cli was changed to
	```rust
	println!(" - {{{:03}}} {}| {}", i, props.local_name.unwrap(), peripheral.address());
	```
	This allows the code to find the desired motor controller over BLE using its mac address.

4. **Install Python dependencies:**  
	```bash
	pip install absl-py
	pip install pyserial
	```

### Pi Configuration
To allow the Pi user to start and stop the zenohd service without a password:

1. Open the sudoers file with visudo:
	```bash
	sudo visudo
	```
2. Add the following line at the end of the file:
	```
	pfr ALL=(ALL) NOPASSWD: /usr/bin/systemctl start zenohd.service, /usr/bin/systemctl stop zenohd.service
	```
3. Open ~/.profile with editor of your choice and add:
	```bash
	# Check if it's a local TTY login and not an SSH session
	if [ -z "$SSH_CONNECTION" ] && [ -z "$SSH_CLIENT" ] && tty | grep -q '/dev/tty'; then
		 /usr/bin/python3 ~/start_menu.py
	fi
	```
4. Optional, but reccomended: automatically source pfr-software in ~/.bashrc:
	```bash
	source /r8/pfr-software/install/setup.bash
	```

---

## User Interface

The PFR Testing Station provides an interactive menu and automation scripts for motor controller testing and address reassignment.

### Wiring
1. Motor Controller
	- Plug in FTDI cable running from the pi into your board. Make sure all of the encoders are connected correctly, and ensure the output is wired to the motors appropriately before starting. Then use the power adapter at the station to turn it on.
2. Encoder
	- There should be a wires coming off of a board marked ESP32 S3. Connect it appropriately to the encoder you wish to burn a new address to and run the script.

### Main Menu
1. If not already on, turn on the pi and it will open the main menu upon boot
2. It will have the following options:
	- Flash to and Test Motor Controller (submenu)
	- Reassign encoder I2C address (script)
	- More (submenu)

### Flashing and Testing 
This is the hub for testing the motor controller on the wheel module at the testing station. The options are:
1. **Upload and monitor code on ESP32**
	- **USE:** This script will help you flash the correct firmware and setup the settings over the bluetooth CLI
2. **Test Stack**
	- **USE:** Skip to the last step of the previous script. Will use the trike launch file launch zenoh and run pfr_teleop node with a window with serial monitor. Useful if you have setup settings for testing already.
<span style="color:#d9534f;">⚠️WARNING: Do not run both linear and angular motors together. Both move the wheel forward/backward and one could break if they are working against each other. Test if each moves the wheel in both directions separately. ⚠️</span>

### Reassign Encoder I2C Address
- **USE**: Will reassign the sensors I2C address. 
- **DEBUGGING:** The firmware of the ESP32 S3 meant for burning the AS5600L addresses is not meant to be updated, many parts of this script are hard-coded under the assumption that the firmware will remain constant, since it shouldn't need to change over time to stay functional.

### More Menu
Important functionality for setting up a motor controller board for testing without flashing new code onto it:
1. **Open serial monitor for ESP32**
	- **USE:** Will open serial monitor with terminal window. This serial monitor will guide you through steps to get to testing the motor controller in the same way as the "Upload and monitor code on ESP32" script. Specfically, it will prompt you to setup settings using the motor controller CLI if it sees the wifi credentials are not set, and it will prompt you if you want to launch the testing script after the settings are set for testing or if the motor controller enter the ready state. 
	- **DEBUGGING:** 
	Entering ready state could happen with the wrong settings and/or zenoh endpoint, you have to keep track of that if you decide to test using this option.

2. **Open CLI for Motor Controller**
	- **USE:** Opens the CLI to adjust motor controller settings. You have 2 options to respond to before you open the CLI: bluetooth/zenoh, and if you want to set for testing/reset to robot/manually adjust settings. 
	- **DEBUGGING:**
		- <span style="color:#f7b731;">This script is not smart enough to tell you if you can or can't use BLE / Zenoh to connect at the current moment, you have to keep track of this yourself.</span> Wrong mode will either give an error or infinite loop of connection attempts if there are other motors with the same name being scanned for over bluetooth; regardless, killing the program is the most advisable option in the wrong mode. 
		- Resetting the settings involves using the settings.yaml file within the motor_controller project of pfr-motor-controllers. If this filename gets updated pfr-motor-controllers should be recloned on the pi.
---
