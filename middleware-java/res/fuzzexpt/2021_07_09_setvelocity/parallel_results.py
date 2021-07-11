import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import pandas as pd

def parallel_coordinate_plot(data_filename, title, output_image):
    df = pd.read_csv(data_filename)
    fig, host = plt.subplots()

    # TODO: get the metric names working properly
    ynames = ['OuterRegionV', 'AvoidV', 'SpeedV', 'TimeLength']
    data_rows = df.shape[0]
    N = data_rows
    category = np.arange(data_rows)
    print(category)
    y1 = df["OutsideOfOuterRegionViolations"]
    y2 = df["AvoidanceViolationsCount"]
    y3 = df["SpeedViolationsCount"]
    y4 = df["FuzzingTimeLength"]

    # organize the data
    ys = np.dstack([y1, y2, y3, y4])[0]
    ymins = ys.min(axis=0)
    ymaxs = ys.max(axis=0)
    dys = ymaxs - ymins
    ymins -= dys * 0.05  # add 5% padding below and above
    ymaxs += dys * 0.05
    dys = ymaxs - ymins

    # transform all data to be compatible with the main axis
    zs = np.zeros_like(ys)
    zs[:, 0] = ys[:, 0]
    zs[:, 1:] = (ys[:, 1:] - ymins[1:]) / dys[1:] * dys[0] + ymins[0]

    axes = [host] + [host.twinx() for i in range(ys.shape[1] - 1)]
    for i, ax in enumerate(axes):
        ax.set_ylim(ymins[i], ymaxs[i])
        ax.spines['top'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        if ax != host:
            ax.spines['left'].set_visible(False)
            ax.yaxis.set_ticks_position('right')
            ax.spines["right"].set_position(("axes", i / (ys.shape[1] - 1)))

    host.set_xlim(0, ys.shape[1] - 1)
    host.set_xticks(range(ys.shape[1]))
    host.set_xticklabels(ynames, fontsize=14)
    host.tick_params(axis='x', which='major', pad=7)
    host.spines['right'].set_visible(False)
    host.xaxis.tick_top()
    host.set_title(title, fontsize=18)

    colors = plt.cm.tab10.colors
    for j in range(N):
        # to just draw straight lines between the axes:
        host.plot(range(ys.shape[1]), zs[j,:], c=colors[(category[j] - 1) % len(colors) ])

    plt.tight_layout()
    plt.savefig(output_image)
