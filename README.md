# Robosub 2019

Uses ROS Melodic for Ubuntu 18.04
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
- enter command to navigate to workspace:
	roscd

- enter command to navigate to the workspace src folder:
	cd ../src

- enter command to run script:
	./auv_start.bash

#### or:

- open a new terminal (ctrl+alt+t ubuntu shortcut) and enter following command:
	roscore

- open a new terminal and enter following commands:
		roscd pathfinder_dvl/scripts/
		python pathfindDvl.py

- open a new terminal and enter following command:
	rosrun ez_async_data ez_async_data

- open a new terminal and enter following command:
	rosrun rosserial_python serial_node_mega.py

- open a new terminal and enter following command:
		rosrun hardware_interface hardware_interface

- open a new terminal and enter following command:
		rosrun rosserial_python serial_node.py

- open a new terminal and enter following commands:
		roscd robosub/scripts/
		python cli_robosub.py


#### then:

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
	- to run the Computer Vision test task type this command to show list:
		task ?
	- to run all the task though the order which they are listed:
		navigation cv 1
	- to cancel any task that is run enter:
		task
