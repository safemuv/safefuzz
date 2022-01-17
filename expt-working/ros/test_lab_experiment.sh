#!/bin/sh
JC=`cat java_cmd_string`
$JC fuzzexperiment.runner.main.TestExperiment_Lab $1 $2 $3 $4
