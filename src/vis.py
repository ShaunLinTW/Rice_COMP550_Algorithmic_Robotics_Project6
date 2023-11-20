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

import pygame

from math import pi as PI
from math import atan2, sin, cos, sqrt


def pygame_visualize(ws_model, X, U, goal, time = None, name=None):
    
    # Initialize Pygame
    pygame.init()
    # set map size 7m by 7m
    screen = pygame.display.set_mode((700,700))
    clock = pygame.time.Clock()
    outlineThickness = 2

    # Draw obstacles
    # [1.4, 3.5, 0.3, 0.3]
    # [2.8, 3.5, 0.3, 0.3]
    # [4.2, 3.5, 0.3, 0.3]
    # [5.6, 3.5, 0.3, 0.3]
    for hole in ws_model['circular_obstacles']:
        pygame.draw.rect(screen, (0,0,255), ((hole[0]-hole[2])*100, (hole[1]-hole[2])*100, 100*hole[2], 100*hole[2]), outlineThickness)
    # pygame.draw.rect(screen, (255,0,0), (1.0, 1.0, 10.0, 10.0), outlineThickness)
    # pygame.draw.rect(screen, (0,255,0), (100.0, 100.0, 10.0, 10.0), outlineThickness)
    # pygame.draw.rect(screen, (255,0,0), (200.0, 200.0, 10.0, 10.0), outlineThickness)
    # pygame.draw.rect(screen, (0,255,0), (300.0, 300.0, 10.0, 10.0), outlineThickness)
    # pygame.draw.rect(screen, (0,0,255), (400.0, 400.0, 10.0, 10.0), outlineThickness)
    # pygame.draw.rect(screen, (0,255,0), (500.0, 500.0, 10.0, 10.0), outlineThickness)
    # pygame.draw.rect(screen, (255,0,0), (590.0, 590.0, 10.0, 10.0), outlineThickness)


    # Draw robots
    # for i in range(len(X)):
    #     pygame.draw.circle(screen, (0,0,0), (X[i][0], X[i][1]), ws_model['robot_radius'])
    #     pygame.draw.line(screen, (0,0,0), (X[i][0], X[i][1]), (X[i][0] + U[i][0], X[i][1] + U[i][1]))
    #     pygame.draw.circle(screen, (0,255,0), (goal[i][0], goal[i][1]), 10)

    # Update display        
    pygame.display.flip()

    # Control frame rate
    clock.tick(30)

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
    for hole in ws_model['square_obstacles']:
        # maptplotlib.patches.Rectangle((x,y),width,height)
        srec = matplotlib.patches.Rectangle(
                (hole[0]-hole[2], hole[1]-hole[2]),
                2*hole[2], 2*hole[2],
                facecolor= 'blue',
                fill = True,
                alpha=1)
        ax.add_patch(srec)

    # plot circular obstacles
    for hole in ws_model['circular_obstacles']:
        # maptplotlib.patches.Rectangle((x,y),width,height)
        # srec = matplotlib.patches.Rectangle(
        #         (hole[0]-hole[2], hole[1]-hole[2]),
        #         2*hole[2], 2*hole[2],
        #         facecolor= 'blue',
        #         fill = True,
        #         alpha=1)
        # ax.add_patch(srec)
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
    # set map size 5m by 5m
    # ax.set_xlim(0, 5)
    # ax.set_ylim(0, 5)
    # set map size 7m by 7m
    ax.set_xlim(0.0, 7.0)
    ax.set_ylim(0.0, 7.0)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    # show
    # pyplot.show()
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

# visualize circles on square obstacle's edge
# if __name__ == '__main__':
#     #define workspace model
#     ws_model = dict()
#     #robot radius
#     ws_model['robot_radius'] = 0.2
#     #circular obstacles, format [x,y,rad]
#     #---no obstacles---#
#     ws_model['circular_obstacles'] = []
#     #---with obstacles---#
#     # [0, 3.5, 2.8]
#     # [7, 3.5, 2.8]
#     # ws_model['square_obstacles'] = [[0, 3.5, 2.8], [7, 3.5, 2.8]]
#     #---with obstacles---#
#     # [1.4, 3.5, 0.3]
#     # [2.8, 3.5, 0.3]
#     # [4.2, 3.5, 0.3]
#     # [5.6, 3.5, 0.3]
#     ws_model['square_obstacles'] = [[1.4, 3.5, 0.3], [2.8, 3.5, 0.3], [4.2, 3.5, 0.3], [5.6, 3.5, 0.3]]
#     #---with obstacles---#
#     # [3.5, 10.2, 4]
#     # [0, 5.5, 0.7]
#     # [7, 5.5, 0.7]
#     # [1, 3.5, 1.5]
#     # [6, 3.5, 1.5]
#     # [0, 1.5, 0.7]
#     # [7, 1.5, 0.7]
#     # [3.5, -3.2, 4]
#     # ws_model['square_obstacles'] = [[3.5, 10.2, 4], [0, 5.5, 0.7], [7, 5.5, 0.7], [1, 3.5, 1.5], [6, 3.5, 1.5], [0, 1.5, 0.7], [7, 1.5, 0.7], [3.5, -3.2, 4]]
#     # implement circles around edge for square obstacles

#     for square in ws_model['square_obstacles']:
#         radius = square[2] * 0.1
#         # in the square, the number of circles in a row
#         num_circles = int(square[2] / radius)
#         print("num_circles: ", num_circles)
#         # the distance between the center of two circles
#         distance = (square[2]) / (num_circles/2)
#         # Top and bottom edges
#         for i in range(num_circles):
#             # top edge
#             x = square[0] - square[2] + radius + i * distance
#             y = square[1] + square[2] - radius
#             ws_model['circular_obstacles'].append([x, y, radius])

#             # bottom edge
#             x = square[0] + square[2] - radius - i * distance
#             y = square[1] - square[2] + radius
#             ws_model['circular_obstacles'].append([x, y, radius])
#         # Left and right edges
#         for i in range(num_circles):
#             # left edge
#             x = square[0] - square[2] + radius
#             y = square[1] - square[2] + radius + i * distance
#             ws_model['circular_obstacles'].append([x, y, radius])
#             # right edge
#             x = square[0] + square[2] - radius
#             y = square[1] + square[2] - radius - i * distance
#             ws_model['circular_obstacles'].append([x, y, radius])
#         # print total number of circles
#     print("total circles: ", len(ws_model['circular_obstacles']))

#     # 2 robots with map size 7m by 7m
#     X = [[3.5, 1.0], [3.5, 6.0]]
#     # 4 robots with map size 7m by 7m, and in a H maze
#     # X = [[1.5, 5.5], [5.5, 5.5], [1.5, 1.5], [5.5, 1.5]]

#     # velocity of [vx,vy]
#     V = [[0,0] for i in range(len(X))]
#     # maximal velocity norm
#     V_max = [1.0 for i in range(len(X))]
#     # 2 robots with map size 7m by 7m
#     goal = [[3.5, 6.0], [3.5, 1.0]]
#     # 4 robots with map size 7m by 7m, and the goal of each robot is on the opposite side
#     # goal = [[5.5, 1.5], [1.5, 1.5], [5.5, 5.5], [1.5, 5.5]]

#     visualize_traj_dynamic(ws_model, X, V, goal, time = None, name=None)