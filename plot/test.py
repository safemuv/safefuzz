#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

# Create multiple rectangles here
rect = Rectangle((0,0),300,50)

fig,ax  = plt.subplots(1)
ax.add_patch(rect)
plt.ylim(0,300);
plt.xlim(0,1000);
plt.show()




# How to build the automatic run script

# while (work_to_do) {
# Generate a fault instance file for the experiment
# run it with the appropriate MOOSTimeWarp
# Copy the result file into a combined format - assessing the suitable metrics for it
# Kill any residual Java processes for the CI and the middleware
# If doing the mutation: use the results to guide mutation
# Otherwise: modify according to a predefined coverage pattern
# }
