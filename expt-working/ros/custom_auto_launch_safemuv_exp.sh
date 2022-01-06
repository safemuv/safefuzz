#!/bin/sh

rm -rf $HOME/.ros/log

SCEN_LAUNCH_BASE="$HOME/catkin_ws/src/scenario_generation/safemuv_scenarios/scenarios/"

echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "roslaunch $SCEN_LAUNCH_BASE/$1/launch/$1_sim.launch" &
sleep 15
echo "Starting SAFEMUV Docker container"
xterm -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared afi_core /bin/bash -c "source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared $1_afi_bringup.launch" &
xterm -e docker run -it --network host metrics_test &
sleep 15
#echo "Starting roslaunch launchers"
#xterm -e /bin/bash -l -c "roslaunch safemuv_rviz_visualization aerolab_safemuv_visualization.launch" &
sleep 15
xterm -e /bin/bash -l -c "roslaunch $SCEN_LAUNCH_BASE/$1/launch/$1_mission.launch" &
