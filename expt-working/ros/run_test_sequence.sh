#!/bin/sh

FUZZ_CONF_DIR=$HOME/academic/atlas/atlas-middleware/expt-working/ros/fuzz-configs
echo "Testing a null fuzzing experiment"
./run_repeated_experiment.sh $FUZZ_CONF_DIR/empty-fuzzing-file.csv 1 empty-fuzzing-file.res
sleep 10
echo "Testing all the avoidance points"
echo "Running the set_velocity test - should cause an avoidance violation"
sleep 10
./run_repeated_experiment.sh $FUZZ_CONF_DIR/test-closeapproach.csv 1 test-closeapproach.res
echo "Running the calibration_point test - should cause a calibration error and distorted geometry (misaligned in rviz)"
sleep 10
./run_repeated_experiment.sh $FUZZ_CONF_DIR/ros-calib-test.csv 1 test-calib.res
echo "Running the path update test - calibration should be correct again!"
echo "The path should be fuzzed with distortions"
sleep 10
./run_repeated_experiment.sh $FUZZ_CONF_DIR/ros-fuzztest-path.csv 1 test-paths.res
echo "Done"
echo "Testing overlapping experiments"
./run_repeated_experiment.sh $FUZZ_CONF_DIR/setvelocity-overlap-test.csv 1 test-overlap.res


