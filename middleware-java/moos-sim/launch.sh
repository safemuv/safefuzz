#!/bin/sh
. ./set_paths.sh
pAntler targ_shoreside.moos > console-logs/shoreside-out 2> console-logs/shoreside-err  &
pAntler targ_ella.moos > console-logs/ella-out 2> console-logs/ella-err  &
pAntler targ_frank.moos > console-logs/frank-out 2> console-logs/frank-err  &
