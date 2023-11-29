'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''
import os
from pathlib import Path
import argparse
from RVO_refactor import RVO_update, Rob
# from vis import visualize_traj_dynamic
from vis_shaun import visualize_traj_dynamic
from math import sqrt

def square2circle(squareList):
    circles = []
    for square in squareList:
        # implement circles around edge for square obstacles
        radius = square[2]*0.1
        # in the square, the number of circles in a row
        num_circles = int(square[2] / (radius))
        # the distance between the center of two circles
        tiny_circle_distance = square[2] / (num_circles/2)
        # Top and bottom edges
        for i in range(num_circles):
            # top edge
            x = square[0] - square[2] + radius + i * tiny_circle_distance
            y = square[1] + square[2] - radius
            circles.append([x, y, radius])

            # bottom edge
            x = square[0] + square[2] - radius - i * tiny_circle_distance
            y = square[1] - square[2] + radius
            circles.append([x, y, radius])
        # Left and right edges
        for i in range(num_circles):
            # left edge
            x = square[0] - square[2] + radius
            y = square[1] - square[2] + radius + i * tiny_circle_distance
            circles.append([x, y, radius])
            # right edge
            x = square[0] + square[2] - radius
            y = square[1] + square[2] - radius - i * tiny_circle_distance
            circles.append([x, y, radius])

    return circles 

def runRVO(args):
    #------------------------------
    # create directory
    # Directory 
    directory = args.map + "_" + args.method
    # Parent Directory path 
    parent_dir = "../visualization/"
    # Path 
    path = Path(parent_dir) / directory
    Path(path).mkdir(parents=True, exist_ok=True)

    #------------------------------
    #define workspace model
    ws_model = dict()
    #robot radius
    ws_model['robot_radius'] = 0.2
    #initialize obstacles
    ws_model['square_obstacles'] = []
    ws_model['circular_obstacles'] = []

    #object container
    X = []

    #------------------------------
    if args.map == "hmap":
        # -------------- obstacle config
        # [3.5, 10.2, 4]
        # [0, 5.5, 0.7]
        # [7, 5.5, 0.7]
        # [1.3, 3.5, 1.5]
        # [5.7, 3.5, 1.5]
        # [0, 1.5, 0.7]
        # [7, 1.5, 0.7]
        # [3.5, -3.2, 4]
        ws_model['square_obstacles'] = [[3.5, 10.2, 4], [0, 5.5, 0.7], [7, 5.5, 0.7], [1.3, 3.5, 1.5], [5.7, 3.5, 1.5], [0, 1.5, 0.7], [7, 1.5, 0.7], [3.5, -3.2, 4]]
        # 4 robots with map size 7m by 7m, and in a H maze
        ws_model['circular_obstacles'] = square2circle(ws_model['square_obstacles'])
        X.append(Rob([1.5, 5.5],[5.5, 1.5]))
        X.append(Rob([5.5, 5.5],[1.5, 1.5]))
        X.append(Rob([1.5, 1.5],[5.5, 5.5]))
        X.append(Rob([5.5, 1.5],[1.5, 5.5]))

    elif args.map == "two":
        # -------------- obstacle config
        # 2 robots with map size 5m by 5m
        # X = [[2.5, 1.0], [2.5, 4.0]]
        X.append(Rob([2.5, 1.0], [2.5, 4.0]))
        X.append(Rob([2.5, 4.0], [2.5, 1.0]))

    elif args.map == "2ob":
        # -------------- obstacle config
        # 2 robots with map size 5m by 5m
        ws_model['square_obstacles'] = [[1.4, 3.5, 0.3], [2.8, 3.5, 0.3], [4.2, 3.5, 0.3], [5.6, 3.5, 0.3]]
        X.append(Rob([3.5, 1.0], [3.5, 6.0]))
        X.append(Rob([3.5, 6.0], [3.5, 1.0]))
    
    elif args.map == "14":
        # 14 robots with map size 7m by 7m
        for i in range(7):
            X.append(Rob([-0.5+1.0*i, 0.0], [5.5-1.0*i, 5.0]))
            X.append(Rob([-0.5+1.0*i, 5.0], [5.5-1.0*i, 0.0]))

    elif args.map == "14ob":
        ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
        # 14 robots with map size 7m by 7m
        for i in range(7):
            X.append(Rob([-0.5+1.0*i, 0.0], [5.5-1.0*i, 5.0]))
            X.append(Rob([-0.5+1.0*i, 5.0], [5.5-1.0*i, 0.0]))
            
    #------------------------------
    #simulation setup
    # total simulation time (s)
    total_time = 30
    # simulation step
    step = 0.01

    #------------------------------
    #simulation starts
    t = 0
    while t*step < total_time:
        print(f"--------------------- step {t}")
        # print position 
        # for r in X:
        #     print(f"pos: {r.pos}")
        # compute desired vel to goal
        reach = True
        for r in X:
            cur_reach = r.compute_goal_v()
            reach = reach and cur_reach

        if reach:
            print(f"All obstacles reached their goals in {t} steps")
            break
        # print desired position
        # for r in X:
        #     print(f"desired velocity: {r.hypo_vel}")
        # compute the optimal vel to avoid collision
        RVO_update(X, ws_model, args.method)

        # update position
        for r in X:
            r.update(step)

        #----------------------------------------
        # visualization
        if t%10 == 0:
            # uncomment to save simulation with obstacles
            fname = f"{path}" + "/snap%s.png"
            visualize_traj_dynamic(ws_model, X, time=t*step, name=fname%str(t/10))
        t += 1
    print(f"all image stored in {path}")
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run RVO simulation.')
    # parser.add_argument("num_rob", type=int,
    #                 help="number of robot")
    # parser.add_argument("num_obs", type=int,
    #                 help="number of obstacles")
    parser.add_argument("map", type=str, help="map name")
    parser.add_argument("method", type=str, help="round_s/dir_pri/vel_pri")
    
    args = parser.parse_args()
    runRVO(args)
