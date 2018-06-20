#!/bin/bash
gnome-terminal -x sh -c "cd robosub/scripts/; python robosub_cli.py; bash"
sleep 1
gnome-terminal -x sh -c "rosrun rosserial_python serial_node.py; bash"
gnome-terminal -x sh -c "rosrun ez_async_data ez_async_data; bash"
#gnome-terminal -x sh -c "cd pathfinder_dvl/scripts/; python pathfindDvl.py; bash"
gnome-terminal -x sh -c "rosrun hardware_interface hardware_interface; bash"

