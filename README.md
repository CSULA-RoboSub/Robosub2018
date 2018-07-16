# Robosub 2018

Uses ROS Lunar for Ubuntu 17.04 
and python 2.7.13

## First time setup:

### In terminal:
- git clone https://github.com/CSULA-RoboSub/Robosub2018.git or git@github.com:CSULA-RoboSub/Robosub2018.git
- cd Robosub2018/robosub/scripts
- python installer.py

#### After installation:
- Restart computer
- cd ~/robosub_ws/
- catkin_make

## Running the program:

### In terminal:
- ./robosub_cli.sh

or

- ./robosub_gui.sh

## General Usage:
- open a new terminal (ctrl+alt+t ubuntu shortcut) and enter following command:
	roscore

- open a new terminal and enter following commands:
	cd robosub/scripts/
	python robosub_cli.py

- open a new terminal and enter following command:
	rosrun ez_async_data ez_async_data

- open a new terminal and enter following commands:
	cd pathfinder_dvl/scripts/
	python pathfindDvl.py

- open a new terminal and enter following command:
	rosrun rosserial_python serial_node.py

- open a new terminal and enter following command:
	rosrun hardware_interface hardware_interface

- once all terminals are open and running the corresponding programs
	- on the robosub_cli.py terminal enter '?' without quotes to see
	  available commands
	- this command must be first typed for the AUV to maneuver:
		motor on
	- this command can be typed to kill the motors after:
		motor off
	- to run AUV manually with keyboard control type this command
	  a list of keybindings will appear:
		navigation keyboard
	- to run the Computer Vision test task type this command:
		navigation cv 1
	  to stop:
		navigation cv 0