# Additional Results

This page contains additional results and discussion that could not be
included in the main body of the paper for space reasons

# Research Question 1
## Five Metrics Multi-Objective Optimisation

An experiment was performed involving multi-objective optimisation
upon all five metrics, using the time-based fuzzing, and all the
available fuzzing points (set_velocity, calibration points and
desired_path). This was continued for 200 iterations, with a
population of 20, consisted of 10 generations for the GA. The
non-dominated solution in the output population results are shown in
the table below:

From: 2022_04_29_12_14_timebased/jmetal-intermediate-nondom-eval200.csv

| ScenID | FuzzingTestNum | ORegionV | SpeedV | TimeLen | AvoidV | IRegionV |
| ------ | -------------- | -------- | ------ | ------- | ------ | -------- |
|F004 | 3 | 870.0 | 371.0 | 1071.34 | 7.0 | 21.0|
|F004 | 20 | 0.0 | 22.0 | 632.09 | 0.0 | 492.0|
|F004 | 156 | 0.0 | 0.0 | 15.13 | 0.0 | 0.0|
|F004 | 161 | 0.0 | 0.0 | 347.65 | 0.0 | 391.0|
|F004 | 167 | 305.0 | 183.0 | 956.63 | 0.0 | 95.0|
|F004 | 177 | 0.0 | 4.0 | 18.77 | 0.0 | 0.0|
|F004 | 188 | 810.0 | 140.0 | 461.55 | 229.0 | 7.0|
|F004 | 192 | 308.0 | 0.0 | 203.72 | 0.0 | 0.0|
|F004 | 199 | 1008.0 | 82.0 | 1333.39 | 97.0 | 0.0|
|F004 | 155 | 466.0 | 115.0 | 861.44 | 101.0 | 35.0|
|F004 | 59 | 25.0 | 102.0 | 664.94 | 1.0 | 357.0|
|F004 | 164 | 914.0 | 278.0 | 971.80 | 11.0 | 1.0|
|F004 | 163 | 78.0 | 12.0 | 194.48 | 20.0 | 8.0|
|F004 | 160 | 413.0 | 70.0 | 677.75 | 0.0 | 427.0|
|F004 | 180 | 685.0 | 167.0 | 248.26 | 0.0 | 18.0|
|F004 | 196 | 538.0 | 263.0 | 827.32 | 0.0 | 9.0|
|F004 | 184 | 421.0 | 193.0 | 474.28 | 2.0 | 66.0|
|F004 | 178 | 857.0 | 235.0 | 1111.19 | 11.0 | 10.0|
|F004 | 165 | 788.0 | 37.0 | 355.63 | 6.0 | 0.0|
|F004 | 157 | 510.0 | 147.0 | 873.64 | 0.0 | 23.0|

The non-dominated solutions consist of 20 configurations. Considering
the fuzzing points selected by the GA, 11 of the fuzzing
configurations used calibration point fuzzing.  desired_path fuzzing
was not used in the output population. Every record contained velocity
fuzzing, which contained a balance of random offset and fixed fuzzing
operations. All fuzzing configurations contained at least 2 records,
and 8 configurations contained 3 records (the maximum of the
population). Three of the configurations containing calibration
fuzzing achieved high values of inner region violations (around 100 or
latest, representing a significant proportion of time spent either
crashed or in contact with the vehicle, although calibration point
fuzzing was not required for this).

The configurations which achieved the best results will be analysed
below. The configuration FuzzingTestNum 156 contained approximately 15
seconds of fuzzing time which produced a null result, and thus
represents a zero point within the population. The configuration which
produced the largest IRegionV metric is FuzzingTestNum
20. This consisted a calibration point offset, combined with fixed
velocity fuzzing on **UAV 2** from 76 seconds to 133 seconds. The
cause of the avoidance violations was the velocity fuzzing producing a
crash into the front side of the vehicle, almost upon the
nose. **UAV 2** then skids along the front left of the vehicle,
coming to rest underneath the vehicle. Although this is a short
fuzzing burst, it has a high impact upon the **IRegionV** metric
due to the crash leading to continued requirement R1 violation.

The configuration which produced the largest **AvoidV** metric is
FuzzingTestNum 188. This consistent of a single fuzzing test record,
specifying randomised offset fuzzing starting at 105 seconds. In this
configuration, the random offset fuzzing causes both UAVs to drift and
soon both make a close approach to the vehicle. They proceed on a
common trajectory outside of the vehicle's left wing, almost becoming
"entangled" with each other. This accounts for the high
**AvoidV** metric value offset fuzzing value.  They also both
leave the permitted area to the leftmost wing side of the vehicle,
which leads to a correspondingly high **ORegionV** metric.

The configuration which produced the largest **ORegionV** metric
was FuzzingTestNum 199, which consisted of a very long fuzzing
duration overall. It incorporated 2 instances of velocity fuzzing
(random offset and fixed fuzzing upon both vehicles), together with
calibration fuzzing. The main contribution is from the velocity
fuzzing starting at 42 seconds, which drives both UAVs outside of the
topology, away from the aircraft. Following the completion of this
fuzzing record, the vehicles are still outside the region and remain
so for the rest of the simulation. This accounts for the very large
**ORegionV** metric (over 1000).

The configuration which produced the largest **SpeedV** metric
value, FuzzingTestNum 3, resulted from almost full duration random
offset fuzzing, which served to cause oscillations before bringing
**UAV 2** outside of the permitted region at excessive speed. 
The overspeed occurs on vehicle **UAV 2** first, which flies out
of the permitted area at high speed, and later **UAV 1** joins
it. This accounts for the also high **ORegionV** value.

TODO: Additional Discussion for RQ2 as specified in paper
