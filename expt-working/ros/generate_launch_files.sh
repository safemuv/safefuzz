#!/bin/sh

# TODO: this is fixed to scenario S001
. ~/academic/safemuv/knowledge_graph/safemuv/bin/activate
cd ~/catkin_ws/src/scenario_generation
zenity --info --text $1
zenity --info --text $2
echo python3 ./scenario_generation/scripts/generate_launchers.py S001 --scenario $1 --fuzz-topics $2
python3 ./scenario_generation/scripts/generate_launchers.py S001 --scenario $1 --fuzz-topics $2
