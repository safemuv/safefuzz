#!/bin/bash
# Launch MOOS
cd ./middleware-java/moos-sim && ./launch.sh &
sleep 10
# Need to remove the GUI from middleware for automated runs
# Supply a file name for the fault instance definition file
java -cp jar-created/atlas-test.jar middleware.core.ATLASMain ./testrandom.fif &
sleep 5
java -cp jar-created/atlas-test.jar atlascollectiveintgenerator.runner.CollectiveIntRunner &
