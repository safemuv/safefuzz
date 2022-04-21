#!/usr/bin/python3
import glob
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def plot_trajectory(filename, marker='.', real=False):
    df = pd.read_csv(filename)

    if real:
        x = df['pose.position.x']
        y = df['pose.position.y']
        z = df['pose.position.z']
    else:
        x = df['pose.pose.position.x']
        y = df['pose.pose.position.y']
        z = df['pose.pose.position.z']
        
    plt.scatter(x,y,z, marker = marker)

def plot_all():
    files = glob.glob("merged-S004_10_*.csv")
    print(files)
    for f in files:
        plot_trajectory(f, '.')

    plot_trajectory("merged-S004_10real.csv", marker="s", real=True)
    plt.show()


plot_all()
