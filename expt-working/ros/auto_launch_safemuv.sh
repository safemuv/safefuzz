#!/bin/sh
# docker run -i --name "SAFEMUV" --network host -v /home/jharbin/catkin_ws/src/safemuv/safemuv_shared:/catkin_ws/src/safemuv_shared afi_base_software bash
echo "Starting SAFEMUV Docker container"
docker start SAFEMUV
sleep 10
echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "roslaunch safemuv_launchers safemuv_prepare_simulation.launch" &
sleep 5
xterm -e /bin/bash -l -c "roslaunch safemuv_launchers safemuv_launch_obstacle_publisher.launch" &
sleep 5
xterm -e /bin/bash -l -c "roslaunch safemuv_launchers safemuv_visualizator.launch" &
xterm -e /bin/bash -l -c "docker exec -it SAFEMUV bash -c 'source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared shared_launcher.launch'" &
sleep 5
xterm -e /bin/bash -l -c "rosrun safemuv_mission state_machine.py"


