#!/usr/bin/python3

import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import sys
import glob
import os

#filename = sys.argv[1]
#print("Filename given:" + filename)

def read_topics(filename):
    filestart = filename.split(".",1)[0]
    b = bagreader(filename)
    d_ic = b.message_by_topic("/inspection_clock")
    d_odom = b.message_by_topic("/uav_1/ground_truth/state")
    df_ic = pd.read_csv(d_ic)
    df_odom = pd.read_csv(d_odom, usecols=['Time', 'pose.pose.position.x', 'pose.pose.position.y', 'pose.pose.position.z'])
    df_output = pd.merge_asof(df_ic, df_odom, on="Time", direction="nearest")
    df_output.to_csv("merged-" + filestart + ".csv")

def reindex_file(filename):
    os.system("rosbag reindex " + filename)
        
def process_all_files():
    files = glob.glob("S004_10*.sim")
    print(files)
    for f in files:
            print("Reindexing and processing " + f)
            reindex_file(f)
            read_topics(f)
        
process_all_files()
