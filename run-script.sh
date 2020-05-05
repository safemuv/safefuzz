#!/bin/bash
# Launch MOOS
cd ./middleware-java/moos-sim && ./launch.sh &
# Wait until all the MOOSDBs are ready
until [ ps -Af | grep MOOSD[B] | wc -l ]; do
sleep 1
done

# Need to remove the GUI from middleware for automated runs
# Supply a file name for the fault instance definition file
java -cp jar-created/atlas-test.jar middleware.core.ATLASMain ./testrandom.fif &
sleep 1
java -cp jar-created/atlas-test.jar atlascollectiveintgenerator.runner.CollectiveIntRunner &
