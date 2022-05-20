n#!/bin/bash
#
# ROS setup ################################################################################################################

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl xterm

# Add keys for ROS and Docker
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

sudo apt update
sudo apt install ros-melodic-desktop-full catkin

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep
sudo apt install ros-melodic-rosbridge-server ros-melodic-rosbridge-msgs ros-melodic-rosbridge-library

sudo rosdep init
rosdep update

# Create catkin workspace
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# GRVC-UAL setup + MAVROS ################################################################################################
sudo apt-get install libeigen3-dev ros-$(rosversion -d)-geodesy ros-$(rosversion -d)-joy
sudo apt install -y ros-$(rosversion -d)-mavros ros-$(rosversion -d)-mavros-extras

cd ~/catkin_ws/src
git clone https://github.com/grvcTeam/grvc-ual.git
echo "Please only select MAVROS backend at this stage"
cd ~/catkin_ws/src/grvc-ual && ./configure.py
cd ~/catkin_ws && catkin_make

sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager

# PX4 SITL ###############################################################################################################
sudo apt install libgstreamer1.0-dev python-jinja2 python-pip
pip install numpy toml
cd ~/catkin_ws/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo
cd ~/catkin_ws
catkin_make

# XXXXXXX software #######################################################################################################
cd ~/catkin_ws/src
git clone -b demo-scenario https://github.com/XXXXXX/XXXXXX_ros.git
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Docker for AFI #########################################################################################################
echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

sudo /etc/init.d/docker start
sudo usermod -aG docker $USER
echo "Now log out and re-login as your user"
echo "Ensure afi_core.tar is available at ~/afi_image"
echo "Then run..."
echo
echo "source ./install2.sh"
