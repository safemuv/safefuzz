#!/bin/sh

RESULT_FILE=$1

. ~/academic/safemuv/virtualenvs/venv/bin/activate

cd ~/academic/safemuv/knowledge_graph

STR="rmkg results csv --file $1"
echo $STR > /tmp/scen_cmd
eval $STR
