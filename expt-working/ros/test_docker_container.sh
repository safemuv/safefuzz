#!/bin/sh
# Arg the directory name of the scenario
xterm -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared/ afi_core /bin/bash -c "source /catkin_ws/devel/setup.bash && roslaunch /catkin_ws/src/safemuv_shared/$1/launch/multi_afi_bringup_$1.launch" &

