#!/bin/sh

FUZZ_CONF_DIR=$HOME/academic/atlas/atlas-middleware/expt-working/ros/fuzz-configs
echo "Testing condition result from experiment"
./run_repeated_experiment.sh $FUZZ_CONF_DIR/test-condition-operation.csv 1 test-cond-op.res
