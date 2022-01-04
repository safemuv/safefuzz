#!/bin/sh

rm -rf $HOME/.ros/log

SCEN_LAUNCH_BASE="$HOME/catkin_ws/src/scenario_generation/safemuv_scenarios/scenarios/"

echo "Starting roslaunch launchers"
# Sim or lab? - should be 3rd argument?
xterm -hold -e /bin/bash -l -c "roslaunch $SCEN_LAUNCH_BASE/$1/launch/$2_sim.launch" &
sleep 15
echo "Starting SAFEMUV Docker container"
xterm -hold -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared afi_core /bin/bash -c "source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared $2_afi_bringup.launch" &
xterm -e docker run -it --network host safemuv_metrics &
sleep 15
#echo "Starting roslaunch launchers"
#xterm -e /bin/bash -l -c "roslaunch safemuv_rviz_visualization aerolab_safemuv_visualization.launch" &
sleep 15
xterm -e /bin/bash -l -c "roslaunch $SCEN_LAUNCH_BASE/$1/launch/$2_mission.launch" &
