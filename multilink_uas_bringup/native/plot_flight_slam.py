#!/usr/bin/env python3
"""Plot recorded GT and initially aligned SLAM trajectories."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

parser = argparse.ArgumentParser()
parser.add_argument('logdir', type=Path)
args = parser.parse_args()
data = np.loadtxt(args.logdir/'aligned_trajectory.csv', delimiter=',', skiprows=1)
t = data[:,0]-data[0,0]
fig = plt.figure(figsize=(12,5))
ax = fig.add_subplot(121, projection='3d')
ax.plot(*data[:,1:4].T,label='Gazebo GT')
ax.plot(*data[:,4:7].T,label='MIMOSA (initial alignment)')
ax.set(xlabel='x [m]',ylabel='y [m]',zlabel='z [m]')
ax.legend(loc='upper left',fontsize=9)
ax.xaxis.set_major_locator(MaxNLocator(4))
ax.yaxis.set_major_locator(MaxNLocator(4))
error = np.linalg.norm(data[:,1:4]-data[:,4:7],axis=1)
ax = fig.add_subplot(122)
ax.plot(t,error)
ax.set(xlabel='Time since first SLAM sample [s]',ylabel='Position error [m]')
ax.grid(alpha=.3)
fig.tight_layout()
fig.savefig(args.logdir/'flight_slam.png',dpi=160)
