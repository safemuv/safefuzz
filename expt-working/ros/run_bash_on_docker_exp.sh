#!/bin/sh
docker run -it --network host -v /home/$USER/catkin_ws/src/safemuv_exp/safemuv_ros/safemuv_shared/:/catkin_ws/src/safemuv_shared/ afi_core /bin/bash
