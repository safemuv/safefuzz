#!/bin/sh
# Real expt

#python3 ./mocap_time_series.py S004_0_2_2022-01-17-14-04-53.bag realexpt
#python3 ./velocity_time_series.py S004_0_2_2022-01-17-14-04-53.bag realexpt

#python3 ./groundtruth_time_series.py S004_0_0_2022-01-21-18-47-05.bag.sim sim
#python3 ./velocity_time_series.py S004_0_0_2022-01-21-18-47-05.bag.sim sim
find . -name "S004_10_*" -exec python3 ./groundtruth_time_series.py '{}' sim ';'
find . -name "S004_10_*" -exec python3 ./velocity_time_series.py '{}' sim ';'

