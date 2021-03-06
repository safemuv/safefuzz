#!/bin/sh

rm -rf $HOME/.ros/log

SCEN_LAUNCH_BASE="$HOME/catkin_ws/src/scenario_generation/safemuv_scenarios/scenarios/"

echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "source ~/catkin_ws/devel/setup.bash && roslaunch $SCEN_LAUNCH_BASE/$1/launch/$1_lab.launch" &
sleep 5
xterm -e /bin/bash -l -c "source ~/catkin_ws/devel/setup.bash && roslaunch safemuv_gazebo_simulation rosbridge.launch" &
sleep 10
echo "Starting SAFEMUV Docker container"
xterm -e docker run --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared afi_core /bin/bash -c "source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared $1_afi_bringup.launch" &
xterm -e docker run -it --network host metrics_test_2 &
sleep 15
#echo "Starting roslaunch launchers"
#xterm -e /bin/bash -l -c "source ~/catkin_ws/devel/setup.bash && roslaunch safemuv_rviz_visualization aerolab_safemuv_visualization.launch" &
sleep 15
xterm -e /bin/bash -l -c "source ~/catkin_ws/devel/setup.bash && roslaunch $SCEN_LAUNCH_BASE/$1/launch/$1_mission.launch" &
