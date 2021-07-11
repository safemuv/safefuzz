#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
from plot_current import parallel_coordinate_plot

parallel_coordinate_plot('final-pop-without-empty.res', 'Non-zero elements in final population', 'fuzzing_pop_setvelocity.pdf');