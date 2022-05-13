#!/bin/bash

#rm -rf $HOME/.ros/log

echo $PATH
echo "Sourcing"
. /opt/ros/melodic/setup.bash
. ~/catkin_ws/devel/setup.bash

echo "Starting roslaunch launchers"
xterm -hold -e /bin/bash -l -c "roslaunch safemuv_gazebo_simulation prepare_sim.launch" &
sleep 25
echo "Starting SAFEMUV Docker container"
xterm -hold -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared afi_core /bin/bash -c 'source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared multi_afi_bringup.launch' &
xterm -hold -e docker run -it --network host safemuv_metrics &
sleep 15
echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "roslaunch safemuv_rviz_visualization safemuv_simulation_visualization.launch" &
sleep 15
#echo "RANDOM delay"
#sleep $[ ( $RANDOM % 3 )  + 1 ]s
xterm -hold -e /bin/bash -l -c "roslaunch safemuv_mission multi_mission.launch"
