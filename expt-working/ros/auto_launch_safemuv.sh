#!/bin/sh

# TODO: reduce the time delays
echo "Starting SAFEMUV Docker containers"
# To setup the images for the first run
#docker run --name SAFEMUV1 -it --network host -v /home/jharbin/catkin_ws/src/safemuv/safemuv_shared:/catkin_ws/src/safemuv_shared afi_base_software_multi bash
#docker run --name SAFEMUV2 -it --network host -v /home/jharbin/catkin_ws/src/safemuv/safemuv_shared:/catkin_ws/src/safemuv_shared afi_base_software_multi bash
docker start SAFEMUV1
docker start SAFEMUV2
sleep 10
echo "Starting roslaunch launchers"
xterm -e /bin/bash -l -c "roslaunch safemuv_gazebo_simulation prepare_sim.launch" &
sleep 5
xterm -e /bin/bash -l -c "roslaunch safemuv_launchers safemuv_launch_obstacle_publisher.launch" &
sleep 5
xterm -e /bin/bash -l -c "roslaunch safemuv_launchers safemuv_visualizator.launch" &
xterm -e /bin/bash -l -c "docker exec -it SAFEMUV1 bash -c 'source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared afi_uav_1.launch'" &
xterm -e /bin/bash -l -c "docker exec -it SAFEMUV2 bash -c 'source /catkin_ws/devel/setup.bash && roslaunch safemuv_shared afi_uav_2.launch'" &
sleep 5
xterm -e /bin/bash -l -c "rosrun safemuv_mission state_machine.py" &
xterm -e /bin/bash -l -c "rosrun safemuv_mission state_machine_2.py"

