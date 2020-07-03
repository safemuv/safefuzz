#!/usr/bin/python
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt

TIME_COL = 5
GENERATION_COL = 0
INITIAL_MAX = 0

INITIAL_TIME_VAL = 100000
FAULT_COL = 3

def plot_filename(filename):
    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle('Missed detection count and time length of faults during evolution process')
    my_data = genfromtxt(filename, delimiter=",")
    generation_nums = my_data[:,GENERATION_COL]
    times = my_data[:,TIME_COL]
    max_per_group = [INITIAL_MAX]*int(max(generation_nums)+1)
    fault_nums = my_data[:,FAULT_COL]
    time_for_max = [INITIAL_TIME_VAL]*int(max(generation_nums)+1)

    for i in range(0, len(fault_nums)):
        g = int(generation_nums[i])
        f = fault_nums[i]
        t = times[i]

        if (f >= max_per_group[g]):
            max_per_group[g] = f
            if (t < time_for_max[g]):
                time_for_max[g] = t

    x = range(0, 1+int(max(generation_nums)))
    ax1.plot(x, max_per_group, "-r")
    ax2.plot(x, time_for_max, "-g")

    plt.xlabel("Generation num")
#    ax2.set_xlabel("Generation num")
    ax1.set_ylabel("Number of missed detections")
    ax2.set_ylabel("Total length of time\nin all fault instances")

    plt.savefig("evolve_plot.png", bbox_inches='tight')

data_filename = "/home/jharbin/academic/atlas/atlas-middleware/results/28_06_2020_2154/evolve_results/evolveProgress.log"

plot_filename(data_filename)
