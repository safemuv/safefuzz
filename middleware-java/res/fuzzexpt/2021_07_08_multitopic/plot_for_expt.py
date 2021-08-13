#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
from parallel_results import parallel_coordinate_plot

parallel_coordinate_plot('final-pop-nondom.res', 'Metrics for final population', 'fuzzing_pop_multitopic.pdf');
