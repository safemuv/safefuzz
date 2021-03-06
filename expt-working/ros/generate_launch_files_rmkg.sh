#!/bin/sh

SCENARIO_ID=$1
TEST_NUM=$2
FUZZ_TOPICS=$3
FUZZ_CONFIG=$4
CONFIG_DIR=$5

FUZZ_SUFFIX="prime"

. ~/academic/safemuv/virtualenvs/venv/bin/activate

cd ~/catkin_ws/src/scenario_generation

if [ -z "${FUZZ_TOPICS}" ]; then
	STR="python3 ./scenario_generation/scripts/rmkg_ros $SCENARIO_ID --test-id $TEST_NUM --fuzzing-config $FUZZ_CONFIG --config-files $CONFIG_DIR --fuzzing-suffix $FUZZ_SUFFIX"
else
	STR="python3 ./scenario_generation/scripts/rmkg_ros $SCENARIO_ID --test-id $TEST_NUM --fuzzing-config $FUZZ_CONFIG --fuzz-topics $FUZZ_TOPICS --config-files $CONFIG_DIR --fuzzing-suffix $FUZZ_SUFFIX"
fi
echo $STR > /tmp/scen_cmd
eval $STR
