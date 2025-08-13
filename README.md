
# PFR Testing Station & Encoder I2C Address Reassigner

A Raspberry Pi–powered test station for automating ESP32-S3 firmware builds, flashing, settings configuration, interactive I²C address reassignment, and motor controller testing.

---

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Software Setup](#software-setup)
3. [Pi Configuration](#pi-configuration)
4. [Firmware Flashing](#firmware-flashing)


---

## Hardware Requirements
**Materials:**
1. Raspberry Pi
2. Keyboard & Monitor
3. FTDI Cable
4. Power adapter with Anderson connectors

---

## Software Setup
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
    println(" - {{{:03}}} {} {}", i, props.local_name.unwrap(), peripheral.address());
    ```
    This allows the code to find the desired motor controller over BLE using its mac address.
    

4. **Install Python dependencies:**  
	```bash
	pip install absl-py
    pip install pyserial
	```

---

## Pi Configuration
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
    
---