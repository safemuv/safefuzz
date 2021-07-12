#!/bin/sh

rm -rf /home/jharbin/.ros/log

echo "Starting SAFEMUV Docker container"
docker run -it --network host -v /home/$USER/catkin_ws/src/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared afi_core bash
sleep 10
echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "roslaunch safemuv_gazebo_simulation prepare_sim.launch" &
sleep 5
xterm -e /bin/bash -l -c "docker exec -it SAFEMUV1 bash -c 'source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared multi_afi_bringup.launch'" &
sleep 5
xterm -e /bin/bash -l -c "roslaunch safemuv_launchers safemuv_visualizator.launch" &
sleep 10
xterm -e /bin/bash -l -c "roslaunch safemuv_mission multi_mission.launch" &

# in the new version, only launch 1 container
# Pedro's new version

