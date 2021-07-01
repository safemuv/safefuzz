#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from pandas.plotting import parallel_coordinates
data = pd.read_csv("final-pop-without-empty.res")
plt.figure();
parallel_coordinates(data, "ID");
plt.show();
