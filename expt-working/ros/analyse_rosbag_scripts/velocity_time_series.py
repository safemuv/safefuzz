#!/usr/bin/python3

import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import sys

filename = sys.argv[1]
tag = sys.argv[2]

print("Filename given:" + filename)

def read_topics(filename):
	b = bagreader(filename)
	d_ic = b.message_by_topic("/inspection_clock")
	d_odom = b.message_by_topic("/uav_1/ual/velocity")
	df_ic = pd.read_csv(d_ic)
	df_odom = pd.read_csv(d_odom)
	df_output = pd.merge_asof(df_ic, df_odom, on="Time", direction="nearest")
	df_output.to_csv("velocity-" + tag + ".csv")

read_topics(filename)
