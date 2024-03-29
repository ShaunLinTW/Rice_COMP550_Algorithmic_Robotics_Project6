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



def visualize_traj_dynamic(ws_model, X, time = None, name=None):
    figure = pyplot.figure()
    ax = figure.add_subplot(1,1,1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
                (hole[0]-hole[2], hole[1]-hole[2]),
                2*hole[2], 2*hole[2],
                facecolor= 'red',
                fill = True,
                alpha=1)
        ax.add_patch(srec)
    # ---plot traj---
    for i in range(0,len(X)):
        cur_X = X[i]
        #-------plot car
        robot = matplotlib.patches.Circle(
            (cur_X.pos[0],cur_X.pos[1]),
            radius = ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=1.0,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        #----------plot velocity
        ax.arrow(cur_X.pos[0], cur_X.pos[1], cur_X.vel[0], cur_X.vel[1], head_width=0.05, head_length=0.1, fc=cmap(i), ec=cmap(i))
        #----------plot RVO
        # ax.arrow()
        ax.text(cur_X.pos[0]-0.1, cur_X.pos[1]-0.1, r'$%s$' %i, fontsize=15, fontweight = 'bold',zorder=3)
        ax.plot([cur_X.goal[0]], [cur_X.goal[1]], '*', color=cmap(i), markersize =15,linewidth=3.0)
    if time:
        ax.text(2,5.5,'$t=%.1f s$' %time,
                fontsize=20, fontweight ='bold')                
    # ---set axes ---
    ax.set_aspect('equal')
    # set map size 5m by 5m
    # ax.set_xlim(0, 5)
    # ax.set_ylim(0, 5)
    # set map size 7m by 7m
    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-1.0, 6.0)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi = 200)
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
