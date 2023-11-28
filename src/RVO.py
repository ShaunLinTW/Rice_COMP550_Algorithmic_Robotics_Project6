'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''

from math import ceil, floor, sqrt
import copy
import numpy

from math import cos, sin, tan, atan2, asin

from math import pi as PI



def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001


def RVO_update(X, V_des, V_current, ws_model):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius'] + 0.02 # add 0.02 to avoid collision
    V_opt = list(V_current)    
    for i in range(len(X)):
        vA = [V_current[i][0], V_current[i][1]]
        pA = [X[i][0], X[i][1]]
        RVO_BA_all = []
        # compute RVO for each robot
        for j in range(len(X)):
            if i!=j:
                vB = [V_current[j][0], V_current[j][1]]
                pB = [X[j][0], X[j][1]]
                # use RVO
                transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
                # use VO
                #transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                dist_BA = distance(pA, pB)
                theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
                if 2*ROB_RAD > dist_BA:
                    dist_BA = 2*ROB_RAD
                theta_BAort = asin(2*ROB_RAD/dist_BA)
                theta_ort_left = theta_BA+theta_BAort
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                theta_ort_right = theta_BA-theta_BAort
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                RVO_BA_all.append(RVO_BA)

        # compute RVO for each circular obstacle           
        for hole in ws_model['circular_obstacles']:
            # hole = [x, y, rad]
            vB = [0, 0]
            pB = hole[0:2]
            transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
            # over-approximation of circular outline
            OVER_APPROX_C2S = 1.05
            # rad is the radius of the over-approximate the circular obstacle
            rad = hole[2]*OVER_APPROX_C2S
            # if the robot is inside the circular obstacle, then the robot should not move
            if (rad+ROB_RAD) > dist_BA:
                dist_BA = rad+ROB_RAD
            theta_BAort = asin((rad+ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA-theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        # compute RVO for each square obstacle
        for square in ws_model['square_obstacles']:
            compute_RVO_square(pA, ROB_RAD, square, RVO_BA_all)

        
        vA_post = intersect(pA, V_des[i], RVO_BA_all) # compute the best velocity
        V_opt[i] = vA_post[:] # update the velocity
    return V_opt

def compute_RVO_square(pA, ROB_RAD, square, RVO_BA_all):

    circles = []
    
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

    # Compute RVO for each circle 
    for circle in circles:
        vB = [0, 0]
        pB = circle[0:2]
        # Same computations as before
        transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
        dist_BA = distance(pA, pB)
        theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
        # over-approximation of circular
        OVER_APPROX_C2S = 1.1
        rad = circle[2]*OVER_APPROX_C2S
        # if the robot is inside the circular obstacle, then the robot should not move
        if (rad+ROB_RAD) > dist_BA:
            dist_BA = rad+ROB_RAD
        theta_BAort = asin((rad+ROB_RAD)/dist_BA)
        theta_ort_left = theta_BA+theta_BAort
        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
        theta_ort_right = theta_BA-theta_BAort
        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
        RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD]
        RVO_BA_all.append(RVO_BA)

def dir_pri(pA, vA, RVO_BA_all, suitable_V, unsuitable_V, theta_v, norm_v):
    for theta in numpy.arange(0, PI, 0.1):
        for cur_theta in [theta_v+theta, theta_v-theta]:
            for vel_step in numpy.arange(0, norm_v, norm_v/5.0):
                for cur_vel in [norm_v+vel_step, norm_v-vel_step]:
                    new_v = [cur_vel*cos(cur_theta), cur_vel*sin(cur_theta)]
                    suit = True
                    for RVO_BA in RVO_BA_all:
                        p_0 = RVO_BA[0]
                        left = RVO_BA[1]
                        right = RVO_BA[2]
                        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                        theta_dif = atan2(dif[1], dif[0])
                        theta_right = atan2(right[1], right[0])
                        theta_left = atan2(left[1], left[0])
                        if in_between(theta_right, theta_dif, theta_left):
                            suit = False
                            break
                    if suit:
                        suitable_V.append(new_v)
                        return new_v
                    else:
                        unsuitable_V.append(new_v)
    return None

def vel_pri(pA, vA, RVO_BA_all, suitable_V, unsuitable_V, theta_v, norm_v):
    for vel_step in numpy.arange(0, norm_v, norm_v/5.0):
        for cur_vel in [norm_v+vel_step, norm_v-vel_step]:
            for theta in numpy.arange(0, PI, 0.1):
                for cur_theta in [theta_v+theta, theta_v-theta]:
                    new_v = [cur_vel*cos(cur_theta), cur_vel*sin(cur_theta)]
                    suit = True
                    for RVO_BA in RVO_BA_all:
                        p_0 = RVO_BA[0]
                        left = RVO_BA[1]
                        right = RVO_BA[2]
                        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                        theta_dif = atan2(dif[1], dif[0])
                        theta_right = atan2(right[1], right[0])
                        theta_left = atan2(left[1], left[0])
                        if in_between(theta_right, theta_dif, theta_left):
                            suit = False
                            break
                    if suit:
                        suitable_V.append(new_v)
                        return new_v
                    else:
                        unsuitable_V.append(new_v)
    return None

def intersect(pA, vA, RVO_BA_all):
    # print('----------------------------------------')
    # print('Start intersection test')
    norm_v = distance(vA, [0, 0])
    theta_v = atan2(vA[1], vA[0])
    suitable_V = []
    unsuitable_V = []
    # --------------using prioritized direction to generate velocities----------------
    # dir_pri_ans = dir_pri(pA, vA, RVO_BA_all, suitable_V, unsuitable_V, theta_v, norm_v)
    # if suitable_V:
    #     print('Suitable found')
    #     # for round research
    #     # for dir pri
    #     vA_post = dir_pri_ans[:]
    # --------------------------------------------------------------------------------

    # ---------------using prioritized velocity to generate velocities----------------
    # vel_pri_ans = vel_pri(pA, vA, RVO_BA_all, suitable_V, unsuitable_V, theta_v, norm_v)
    # if suitable_V:
    #     print('Suitable found')
    #     # for round research
    #     # for vel pri
    #     vA_post = vel_pri_ans[:]
    # --------------------------------------------------------------------------------

    # ---------------using polar coordinates to generate velocities-------------------
    for theta in numpy.arange(0, 2*PI, 0.1):
        for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)                
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)

    if suitable_V:
        print('Suitable found')
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
    # --------------------------------------------------------------------------------
    else: # if there is no suitable velocity, then select the velocity with the minimum time to collision, the minimum time is the most dangerous
        print('Suitable not found')
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                # dif is the vector from the center of the robot to the center of the obstacle
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    # dist_tg is the distance from the center of the robot to the tangent point
                    dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0               
                    # tc_v is the time to collision     
                    tc_v = dist_tg/distance(dif, [0,0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001 # this is to avoid the case that tc = 0, select the minimum tc because it is the most dangerous
        WT = 0.2 # this is the weight of time to collision, the smaller the more important
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA))) # select the velocity with the minimum cost
    return vA_post 

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def compute_V_des(X, goal, V_max):
    # print('----------------------------------------')
    # print('Start compute desired velocity')
    V_des = []
    for i in range(len(X)):
        dif_x = [goal[i][k]-X[i][k] for k in range(2)]
        norm = distance(dif_x, [0, 0])
        norm_dif_x = [dif_x[k]*V_max[k]/norm for k in range(2)]
        V_des.append(norm_dif_x[:])
        if reach(X[i], goal[i], 0.1):
            V_des[i][0] = 0
            V_des[i][1] = 0
    return V_des
            
def reach(p1, p2, bound=0.5):
    if distance(p1,p2)< bound:
        return True
    else:
        return False
    
    
