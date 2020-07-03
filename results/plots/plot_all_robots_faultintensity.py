#!/usr/bin/python
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

from mpl_toolkits.axes_grid1 import make_axes_locatable

from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from numpy import genfromtxt

max_len = 1200
ypos_start = 0
ypos_gap = 3
fig,ax  = plt.subplots(1)
vehicle_name = sys.argv[1];

yticks_pos = []
yticks_labels = []

# Need to put an axis ylabel for each ypos that is used
def plot_range(ypos, filename, fault_intensity):
    my_data = genfromtxt(filename, delimiter=',')
    fault_nums = my_data[:,0]
    [index,counts] = np.unique(fault_nums, return_counts=True)
    # Get the start end time range for the particular experiment...
    starts = my_data[:,2]
    ends = my_data[:,3]
    lengths = ends - starts
    for i in range(0,len(index)):
        v_for_cell = counts[i] / 6
        v_for_cell = min(v_for_cell,1)
        flines = np.where(fault_nums==i)
        fline = flines[0][0]
        end_this_rect = starts[fline] + lengths[fline]
        # Need to increment ypos when the end of the line is reached
        rect = Rectangle((starts[fline],ypos),lengths[fline],1, color=(1,v_for_cell,1))
        if (end_this_rect >= max_len):
            label_text = str(lengths[fline]) + " - " + str(fault_intensity);
            yticks_pos.append(ypos+0.5);
            yticks_labels.append(label_text);
            ypos+=1;
        ax.add_patch(rect)
    return ypos

def set_colorbar():
    step = 1
    min = 0
    max = 6
    Z = [[0,0],[0,0]]
    # Setting up a colormap that's a simple transtion
    mymap = mpl.colors.LinearSegmentedColormap.from_list('mycolors',['white','red'])
    levels = range(min,max+step,step)
    colbar = plt.contourf(Z, levels, cmap=mymap)
    plt.clf();
    plt.colorbar(colbar);

def set_axis_structure():
    plt.ylim(0,22);
    plt.xlim(0,max_len);
    plt.xlabel("Mission time range");
    plt.yticks(yticks_pos, yticks_labels);
    plt.title("Impact of faults upon missed detections by UUV " + vehicle_name + "\n(intensity of colour is more missed detections)");

    divider = make_axes_locatable(plt.gca())
    ax_cb = divider.new_horizontal(size="5%", pad=0.05)
    mymap = mpl.colors.ListedColormap(colors = [(1,1,1),
                                                (1,5/6,1),
                                                (1,4/6,1),
                                                (1,3/6,1),
                                                (1,2/6,1),
                                                (1,1/6,1)])
    cb1 = mpl.colorbar.ColorbarBase(ax_cb, cmap=mymap, orientation='vertical',
                                    norm=mpl.colors.Normalize(vmin=0, vmax=6))
    plt.gcf().add_axes(ax_cb)

    plt.savefig("coverage_singlefault_" + vehicle_name, bbox_inches='tight');

plt.plot()
#base_file = "/home/jharbin/academic/atlas/atlas-middleware/results/24_06_2020_0118/speedfault-coverage/SPEEDFAULT-"
#base_file = "/home/jharbin/academic/atlas/atlas-middleware/results/17_05_2020_0100/SPEEDFAULT-"

# For the report4
base_file = "/home/jharbin/academic/atlas/atlas-middleware/results/30_06_2020_1004/SPEEDFAULT-"

#ypos_start = plot_range(ypos_start, base_file + vehicle_name + "2.0_goalDiscovery.res", 2.0)
#ypos_start+=ypos_gap
ypos_start = plot_range(ypos_start, base_file + vehicle_name + "3.0_goalDiscovery.res", 3.0)
ypos_start+=ypos_gap
ypos_start = plot_range(ypos_start, base_file + vehicle_name + "4.0_goalDiscovery.res", 4.0)
ypos_start+=ypos_gap
ypos_start = plot_range(ypos_start, base_file + vehicle_name + "5.0_goalDiscovery.res", 5.0)
set_axis_structure()

