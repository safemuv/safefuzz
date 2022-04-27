#!/bin/bash

echo $PATH
echo "Sourcing"
. /opt/ros/melodic/setup.bash
. ~/catkin_ws/devel/setup.bash

JC=`cat java_cmd_string`
xterm -hold -e /bin/bash -l -c "roslaunch safemuv_gazebo_simulation record_rosbag.launch bagfile:=$1"
