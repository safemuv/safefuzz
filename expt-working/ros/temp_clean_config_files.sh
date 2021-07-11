#!/bin/sh

DIR=/home/jharbin/catkin_ws/src/safemuv/safemuv_situational_awareness/config
ORIG_FILE=$DIR/calibration_points.original
OUTPUT_FILE=$DIR/calibration_points.yaml
cp $ORIG_FILE $OUTPUT_FILE
echo "Copied original file"
