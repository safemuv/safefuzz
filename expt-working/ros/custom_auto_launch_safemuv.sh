#!/bin/sh
rm -rf $HOME/.ros/log

echo "Starting roslaunch launchers"
xterm -hold -e /bin/bash -l -c "roslaunch safemuv_scenarios $1_sim.launch" &
sleep 25
echo "Starting SAFEMUV Docker container"
xterm -hold -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared/ afi_core /bin/bash -c "source /catkin_ws/devel/setup.bash && roslaunch /catkin_ws/src/safemuv_shared/scenarios/$1/launch/$1_afi_bringup.launch" &
xterm -hold -e docker run -it --network host safemuv_metrics &
sleep 15
#echo "Starting roslaunch launchers"
#xterm -e /bin/bash -l -c "roslaunch safemuv_rviz_visualization safemuv_simulation_visualization.launch" &
sleep 15
xterm -hold -e /bin/bash -l -c "roslaunch safemuv_scenarios $1_mission.launch"
