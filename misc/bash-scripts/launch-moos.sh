#!/bin/bash
# Launch MOOS
pAntler targ_shoreside.moos > console-logs/shoreside-out 2> console-logs/shoreside-err &
pAntler targ_ella.moos      > console-logs/ella-out      2> console-logs/ella-err &
pAntler targ_frank.moos     > console-logs/frank-out     2> console-logs/frank-err &
pAntler targ_gilda.moos     > console-logs/gilda-out     2> console-logs/gilda-err &
pAntler targ_henry.moos     > console-logs/henry-out     2> console-logs/henry-err &
