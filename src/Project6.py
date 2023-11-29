'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''

import sys

from RVO import RVO_update, compute_V_des
from vis_original import visualize_traj_dynamic



#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
#circular obstacles, format [x,y,rad]
# no obstacles
# ws_model['circular_obstacles'] = []
# with obstacles
ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = [] 

#------------------------------
#initialization for robot 
# position of [x,y]
# 2 robots with map size 5m by 5m
# X = [[2.5, 1.0], [2.5, 4.0]]
# 14 robots with map size 7m by 7m
X = [[-0.5+1.0*i, 0.0] for i in range(7)] + [[-0.5+1.0*i, 5.0] for i in range(7)]
# 28 robots with map size 14m by 14m

# velocity of [vx,vy]
V = [[0,0] for i in range(len(X))]
# maximal velocity norm
V_max = [1.0 for i in range(len(X))]

# goal of [x,y]
# 2 robots with map size 5m by 5m
# goal = [[2.5, 4.0], [2.5, 1.0]]
# 14 robots with map size 7m by 7m
goal = [[5.5-1.0*i, 5.0] for i in range(7)] + [[5.5-1.0*i, 0.0] for i in range(7)]
# 28 robots with map size 14m by 14m


#------------------------------
#simulation setup
# total simulation time (s)
total_time = 20
# simulation step
step = 0.01

#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    print(f"--------------------- step {t}")
    # print position
    # for r in X:
    #     print(f"pos: {r}")
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    # print desired position
    # for v in V_des:
    #     print(f"desired velocity: {v}")

    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%10 == 0:
        # uncomment to save simulation with no obstacles
        # visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='../visualization/no_obstacles_mod/snap%s.png'%str(t/10))
        # uncomment to save simulation with obstacles
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='../visualization/with_obstacles_mod/snap%s.png'%str(t/10))
    t += 1
    
