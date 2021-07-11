#!/bin/sh
echo "Convert passed: $1"
grep "uav_1" $1 > $1_uav1
grep "uav_2" $1 > $1_uav2
