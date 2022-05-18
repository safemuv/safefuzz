#!/bin/sh

killall pAntler
sleep 3
killall -9 pAntler
killall -9 MOOSDB
killall -9 uFldNodeBroker
killall -9 pHelmIvP
killall -9 ATLASDBWatch
killall -9 pLogger
killall -9 pMarinePID
killall -9 uProcessWatch
killall -9 pHostInfo
killall -9 pShare
killall -9 pMarineViewer
killall -9 uSimMarine
killall -9 uFldHazardMgr
killall -9 pNodeReporter
killall -9 pBasicContactMgr
killall -9 uFldNodeComms
killall -9 uFldShoreBroker
killall -9 uFldHazardSensor
# Not using JAR files to launch it any more!
#pkill -f "atlas.jar"
#pkill -f "atlas-6robot-case-ci.jar"
#pkill -f "atlas-6robot-case.jar"
# Instead kill by finding
# not sure why pkill not working
kill $(ps aux | grep '[r]unner.CollectiveIntRunner' | awk '{print $2}')
kill $(ps aux | grep '[c]ore.ATLASMain' | awk '{print $2}')

