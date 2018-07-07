sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update

sudo apt-get -y install ros-lunar-desktop-full

apt-cache search ros-lunar

sudo rosdep init
rosdep update

echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
