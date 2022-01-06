#!/bin/sh

rm -rf $HOME/.ros/log

echo "Starting roslaunch launchers"
xterm -hold -e /bin/bash -l -c "roslaunch safemuv_gazebo_simulation aerolab_sim.launch" &
sleep 25
echo "Starting SAFEMUV Docker container"
xterm -hold -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared afi_core /bin/bash -c 'source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared aerolab_afi_bringup.launch' &
xterm -e docker run -it --network host metrics_test &
sleep 15
echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "roslaunch safemuv_rviz_visualization aerolab_safemuv_visualization.launch" &
sleep 15
xterm -e /bin/bash -l -c "roslaunch safemuv_mission aerolab_single_mission.launch"
