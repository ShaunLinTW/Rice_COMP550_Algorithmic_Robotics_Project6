'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

# Function to plot rectangles
def plot_rectangle(ax, position):
    rect = Rectangle(position[:2], position[2], position[3], fill=False)
    ax.add_patch(rect)

# Create a figure with 1x2 subplots
fig, axes = plt.subplots(1, 2, figsize=(8, 4))
axes = axes.flatten()

# Decentralized Multi-robot Coordination 1
axes[0].axis('equal')
axes[0].set_xlim(-10, 10)
axes[0].set_ylim(-10, 10)

positions = [(0, 0, 0, 0)]

for position in positions:
    plot_rectangle(axes[0], position)

data = np.loadtxt('RVO2.txt')
axes[0].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[0].plot(0, 0, 'bs', markersize=2)
axes[0].plot(0, 0, 'rs', markersize=2)
axes[0].set_xlabel('X')
axes[0].set_ylabel('Y')
axes[0].set_title('Decentralized Multi-robot Coordination')

# Decentralized Multi-robot Coordination 2
axes[1].axis('equal')
axes[1].set_xlim(-10, 10)
axes[1].set_ylim(-10, 10)

positions = [(0, 0, 0, 0)]

for position in positions:
    plot_rectangle(axes[1], position)

data = np.loadtxt('RVO1.txt')
axes[1].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[1].plot(0, 0, 'bs', markersize=2)
axes[1].plot(0, 0, 'rs', markersize=2)
axes[1].set_xlabel('X')
axes[1].set_ylabel('Y')
axes[1].set_title('Decentralized Multi-robot Coordination')

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
