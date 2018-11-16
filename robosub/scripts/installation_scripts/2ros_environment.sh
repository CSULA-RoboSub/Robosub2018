source /opt/ros/melodic/setup.bash

mkdir -p ~/robosub_ws
mv ../../../Robosub2018 ~/robosub_ws/src

cd ~/robosub_ws/
catkin_make

source devel/setup.bash

echo "source ~/robosub_ws/devel/setup.bash" >> ~/.bashrc

cp ~/robosub_ws/src/robosub/scripts/scripts/cli_robosub.sh ~
cp ~/robosub_ws/src/robosub/scripts/scripts/gui.robosub.sh ~
