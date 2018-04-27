source /opt/ros/lunar/setup.bash

mkdir -p ~/robosub_ws
mv ../../../Robosub2018 ~/robosub_ws/src

cd ~/robosub_ws/
catkin_make

source devel/setup.bash

echo "source ~/robosub_ws/devel/setup.bash" >> ~/.bashrc

cp ~/robosub_ws/src/robosub/scripts/scripts/robosub_cli.sh ~
cp ~/robosub_ws/src/robosub/scripts/scripts/robosub_gui.sh ~