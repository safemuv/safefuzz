#!/bin/sh

FUZZ_CONF_DIR=$HOME/academic/atlas/atlas-middleware/expt-working/ros/fuzz-configs
echo "Testing all the avoidance points"
echo "Running the set_velocity test - should cause an avoidance violation"
sleep 1
./run_repeated_experiment.sh $FUZZ_CONF_DIR/test-closeapproach.csv 1 test-closeapproach.res
echo "Running the calibration_point test - should cause a calibration error and distorted geometry (misaligned in rviz)"
sleep 10
./run_repeated_experiment.sh $FUZZ_CONF_DIR/ros-calib-test.csv 1 test-calib.res
echo "Running the path update test - calibration should be correct again!"
echo "The path should be fuzzed with distortions"
sleep 10
./run_repeated_experiment.sh $FUZZ_CONF_DIR/ros-fuzztest-path.csv 1 test-paths.res
echo "Done"


