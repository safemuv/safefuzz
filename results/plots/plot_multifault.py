#!/usr/bin/python
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from numpy import genfromtxt

max_len = 1000
ypos_start = 0
ypos_gap = 3
fig,ax  = plt.subplots(1)

filename = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/multifault-working/multifault.res"

yticks_pos = []
yticks_labels = []
datafile = open(filename, 'r')
datareader = csv.reader(datafile)
data = []
counts = {}

for row in datareader:
    value = row[0]
    if value in counts:
        counts[value] += 1;
    else:
        counts[value] = 1

def set_axis_structure():
    plt.xlabel("Number of faults");
    plt.ylabel("Number of detections");
    plt.yticks(yticks_pos, yticks_labels);
    plt.title("Impact of faults upon missed detections in the whole CARS system");
    plt.savefig("multifault_detections", bbox_inches='tight');

print(counts);
plt.bar(counts.keys(), list(counts.values()), align='center');
set_axis_structure();
plt.show();
