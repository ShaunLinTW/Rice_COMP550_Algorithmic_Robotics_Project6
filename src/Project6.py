'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''

import sys
from math import cos, sin, tan, atan2, asin
from math import pi as PI

from RVO import RVO_update, reach, compute_V_des, reach
from vis import visualize_traj_dynamic
from vis import pygame_visualize



#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
#circular obstacles, format [x,y,rad]
#---no obstacles---#
ws_model['circular_obstacles'] = []
#---with obstacles---#
# [1.4, 3.5, 0.3]
# [2.8, 3.5, 0.3]
# [4.2, 3.5, 0.3]
# [5.6, 3.5, 0.3]
# ws_model['circular_obstacles'] = [[1.4, 3.5, 0.3], [2.8, 3.5, 0.3], [4.2, 3.5, 0.3], [5.6, 3.5, 0.3]]
# [0, 3.5, 3]
# [7, 3.5, 3]
# ws_model['circular_obstacles'] = [[0, 3.5, 3], [7, 3.5, 3]]
# [3.5, 10.2, 4]
# [0, 5.5, 0.7]
# [7, 5.5, 0.7]
# [1, 3.5, 1.5]
# [6, 3.5, 1.5]
# [0, 1.5, 0.7]
# [7, 1.5, 0.7]
# [3.5, -3.2, 4]
# ws_model['circular_obstacles'] = [[3.5, 10.2, 4], [0, 5.5, 0.7], [7, 5.5, 0.7], [1, 3.5, 1.5], [6, 3.5, 1.5], [0, 1.5, 0.7], [7, 1.5, 0.7], [3.5, -3.2, 4]]

#square obstacles, format [x,y,rad]
#---with obstacles---#
# [3.5, 3.5, 1.0]
# ws_model['square_obstacles'] = [[3.5, 3.5, 1.0]]
#---with obstacles---#
# [0, 3.5, 2.8]
# [7, 3.5, 2.8]
# ws_model['square_obstacles'] = [[0, 3.5, 2.8], [7, 3.5, 2.8]]
#---with obstacles---#
# [1.4, 3.5, 0.3]
# [2.8, 3.5, 0.3]
# [4.2, 3.5, 0.3]
# [5.6, 3.5, 0.3]
ws_model['square_obstacles'] = [[1.4, 3.5, 0.3], [2.8, 3.5, 0.3], [4.2, 3.5, 0.3], [5.6, 3.5, 0.3]]
#---with obstacles---#
# [3.5, 10.2, 4]
# [0, 5.5, 0.7]
# [7, 5.5, 0.7]
# [1.3, 3.5, 1.5]
# [5.7, 3.5, 1.5]
# [0, 1.5, 0.7]
# [7, 1.5, 0.7]
# [3.5, -3.2, 4]
# ws_model['square_obstacles'] = [[3.5, 10.2, 4], [0, 5.5, 0.7], [7, 5.5, 0.7], [1.3, 3.5, 1.5], [5.7, 3.5, 1.5], [0, 1.5, 0.7], [7, 1.5, 0.7], [3.5, -3.2, 4]]

ws_model['boundary'] = []

#------------------------------
#initialization for robot 
# position of [x,y]
# 2 robots with map size 5m by 5m
X = [[3.5, 1.0], [3.5, 6.0]]

# 4 robots with map size 7m by 7m, and in a H maze
# X = [[1.5, 5.5], [5.5, 5.5], [1.5, 1.5], [5.5, 1.5]]

# 14 robots with map size 7m by 7m, and 7 robots on each side in a line
# X = [[-0.5+1.0*i, 0.0] for i in range(7)] + [[-0.5+1.0*i, 5.0] for i in range(7)]

# 8 robots with map size 7m by 7m, and robots are in a circle
# X = [[3.5+3.0*cos(2*PI/8*i), 3.5+3.0*sin(2*PI/8*i)] for i in range(8)]

# velocity of [vx,vy]
V = [[0,0] for i in range(len(X))]
# maximal velocity norm
V_max = [1.0 for i in range(len(X))]

# goal of [x,y]
# 2 robots with map size 5m by 5m
goal = [[3.5, 6.0], [3.5, 1.0]]

# 4 robots with map size 7m by 7m, and the goal of each robot is on the opposite side
# goal = [[5.5, 1.5], [1.5, 1.5], [5.5, 5.5], [1.5, 5.5]]

# 14 robots with map size 7m by 7m, and the goal of each robot is on the opposite side
# goal = [[5.5-1.0*i, 5.0] for i in range(7)] + [[5.5-1.0*i, 0.0] for i in range(7)]

# 8 robots with map size 7m by 7m, and the goal of each robot is on the opposite side in a circle
# goal = [[3.5+3.0*cos(2*PI/8*i+PI), 3.5+3.0*sin(2*PI/8*i+PI)] for i in range(8)]


#------------------------------
#simulation setup
# total simulation time (s)
total_time = 50
# simulation step
step = 0.01

#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%10 == 0:
        # uncomment to save simulation with no obstacles
        # visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='../visualization/no_obstacles/snap%s.png'%str(t/10))
        # uncomment to save simulation with obstacles
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='../visualization/with_obstacles/snap%s.png'%str(t/10))
    t += 1

    # pygame_visualize(ws_model, X, V, goal, time=t*step)
    # t += 1

print('----------------------------------------')
print('Finished RVO Simulation')