#!/bin/bash
gnome-terminal -x sh -c "roscore; bash"
sleep 1
gnome-terminal -x sh -c "cd pathfinder_dvl/scripts/; python pathfindDvl.py; bash"
sleep 1
gnome-terminal -x sh -c "rosrun rosserial_python serial_node_mega.py; bash"
sleep 1
gnome-terminal -x sh -c "rosrun ez_async_data ez_async_data; bash"
sleep 1
gnome-terminal -x sh -c "rosrun hardware_interface hardware_interface; bash"
sleep 3
gnome-terminal -x sh -c "rosrun rosserial_python serial_node_uno.py; bash"
sleep 1
gnome-terminal -x sh -c "cd robosub/scripts/; python cli_robosub.py; bash"

# gnome-terminal -x sh -c "rosrun rosserial_python serial_node_servo.py; bash"
