# To be run after logging out
# REQUIREMENTS
# The docker image at ~/afi_image/afi_core.tar

cd ~/afi_images
docker load < afi_core.tar

sudo apt install ros-melodic-cmake-modules ros-melodic-roslint libyaml-cpp-dev ros-melodic-tf2-geometry-msgs gazebo9

cd ~/catkin_ws
catkin_make

cd ~/source
git clone https://github.com/jrharbin-york/jgea.git

source ~/catkin_ws/devel/setup.bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

