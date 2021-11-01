#!/bin/sh

killall -9 roslaunch
killall -9 python
killall -9 python2
killall -9 python3
killall -9 gzclient
killall -9 gzserver
killall -9 rviz
killall -9 px4
killall -9 rosout
killall -9 rosmaster
killall -9 rosbridge_server
killall -9 rosbridge_client
killall -9 ual_backend_mavros_server
killall -9 obstacle_publisher_3d
killall -9 RelocatePlane_node
killall -9 static_transform_publisher
killall -9 mavros_node

docker stop $(docker ps -a -q)
docker kill $(docker ps -q)

kill $(ps aux | grep '[c]ore.ATLASMain' | awk '{print $2}')
kill $(ps aux | grep 'ROSLauncher' | awk '{print $2}')

# Ensure the YAML file is always refreshed properly from the original backup
YAML_FILE=$HOME/catkin_ws/src/safemuv_ros/safemuv_situational_awareness/config/calibration_points.yaml
cp $YAML_FILE.original $YAML_FILE
