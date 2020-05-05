#!/bin/bash
# Launch MOOS
cd ./middleware-java/moos-sim && ./launch.sh &
# Wait until all the MOOSDBs are ready
until [ ps -Af | grep MOOSD[B] | wc -l ]; do
sleep 1
done
