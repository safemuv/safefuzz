#!/bin/sh
JC=`cat java_cmd_string`
$JC fuzzexperiment.runner.main.RunExptRepeated_fuzzing $1 $2 $3 $4
