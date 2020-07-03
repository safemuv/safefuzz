#!/usr/bin/python
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt

def plot_filename_col(filename, col_num, axis, colour, dirflag, initial_val):
    my_data = genfromtxt(filename, delimiter=",")
    generation_nums = my_data[:,0]
    max_per_group = [initial_val]*int(max(generation_nums)+1)
    fault_nums = my_data[:,col_num]
    max_index_per_group = [0]*int(max(generation_nums)+1)

    for i in range(0, len(fault_nums)):
        g = int(generation_nums[i])
        f = fault_nums[i]
        print("G = " + str(g) + ",F = " + str(f));
        print(max_per_group[g]);
        if ((f > max_per_group[g]) and dirflag=="GT"):
            max_per_group[g] = f
            max_index_per_group[g] = i

        if ((f < max_per_group[g]) and dirflag=="LT"):
            max_per_group[g] = f

    x = range(0, 1+int(max(generation_nums)))
    axis.plot(x, max_per_group, colour)
    plt.savefig("evolve_plot.png")

# data_filename = "/home/jharbin/academic/atlas/atlas-middleware/results/evolve-old/28_jun/evolveProgress.log"
data_filename = "/home/jharbin/academic/atlas/atlas-middleware/results/28_06_2020_2154/evolve_results/evolveProgress.log"

fig, (ax1, ax2) = plt.subplots(2)
fig.suptitle('Missed detection count and time length of faults during evolution process')
plot_filename_col(data_filename, 3, ax1, '-r', "GT", 0)
plot_filename_col(data_filename, 6, ax2, '-g', "LT", 100000)
