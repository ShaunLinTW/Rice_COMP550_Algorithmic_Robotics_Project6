'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''

#!/usr/bin/env python
import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors

from math import pi as PI
from math import atan2, sin, cos, sqrt


# ws_model is the workspace model
# X is the position of the robot
# U is the velocity of the robot
# goal is the goal of the robot
# time is the time of the simulation
# name is the name of the file
def visualize_traj_dynamic(ws_model, X, U, goal, time = None, name=None):
    figure = pyplot.figure()
    ax = figure.add_subplot(1,1,1)
    cmap = get_cmap(len(X))
    # plot square obstacles
    for square in ws_model['square_obstacles']:
        # maptplotlib.patches.Rectangle((x,y),width,height)
        rect = matplotlib.patches.Rectangle(
                (square[0]-square[2], square[1]-square[2]),
                square[2]*2,
                square[2]*2,
                facecolor= 'blue',
                fill = True,
                alpha=1)
        ax.add_patch(rect)

    # plot circular obstacles
    for hole in ws_model['circular_obstacles']:
        # maptplotlib.patches.Circle((x,y),radius)
        circle = matplotlib.patches.Circle(
                (hole[0], hole[1]),
                radius = hole[2],
                facecolor= 'red',
                fill = True,
                alpha=1)
        ax.add_patch(circle)

    # ---plot traj---
    for i in range(0,len(X)):
        #-------plot car
        robot = matplotlib.patches.Circle(
            (X[i][0],X[i][1]),
            radius = ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=1.0,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        #----------plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0], U[i][1], head_width=0.05, head_length=0.1, fc=cmap(i), ec=cmap(i))
        ax.text(X[i][0]-0.1, X[i][1]-0.1, r'$%s$' %i, fontsize=15, fontweight = 'bold',zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize =15,linewidth=3.0)
    if time:
        ax.text(2,5.5,'$t=%.1f s$' %time,
                fontsize=20, fontweight ='bold')                
    # ---set axes ---
    ax.set_aspect('equal')
    # set map size 7m by 7m
    ax.set_xlim(0.0, 7.0)
    ax.set_ylim(0.0, 7.0)
    # set map size 30m by 30m
    # ax.set_xlim(0.0, 30.0)
    # ax.set_ylim(0.0, 30.0)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi = 200)
        print("saved to ", name)
        #pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm  = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color    
