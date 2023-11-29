'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////
'''

from math import ceil, floor, sqrt, degrees
import copy
import numpy as np

from math import cos, sin, tan, atan2, asin
from math import pi as PI

#------------------------------
# Define robot class
class Rob():
    max_vel = [1.0, 1.0]

    @staticmethod
    def euc_distance(pose1, pose2):
        """ compute Euclidean distance for 2D """
        return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001
    
    def __init__(self, 
                 init_pos, goal, 
                 init_vel=[0,0], max_vel= [1.0, 1.0], 
                 rad=0.2, bias=0.1):
        self.rad = rad
        self.pos = init_pos
        self.new_vel = init_vel
        self.vel = init_vel
        self.hypo_vel = init_vel
        self.goal = goal
        self.max_vel = max_vel
        self.goal_bias = bias

    def reach_goal(self):
        """ compute whether goal is reached """
        # print(f"pos: {self.pos}, goal: {self.goal} ,diff: {self.euc_distance(self.pos, self.goal)}")
        if self.euc_distance(self.pos, self.goal) < self.goal_bias:
            return True
        else:
            return False
        
    def compute_goal_v(self):
        """ compute desired velocity towards goal """
        dif_dist = [self.goal[i] - self.pos[i] for i in range(2)]
        dist_norm = self.euc_distance(dif_dist, [0,0])

        # calculate normalized velocity with range(0, max_vel)
        self.hypo_vel = [dif_dist[i]*self.max_vel[i] / dist_norm for i in range(2)]
        if self.reach_goal():
            # print("reach goal!")
            self.hypo_vel = [0,0]
            return True
        return False
    
    def update(self, step):
        """ update position"""
        self.pos = [self.pos[i] + self.new_vel[i]*step for i in range(2)]
        self.vel = self.new_vel

def RVO_update(X, ws_model, method='round_s', alpha=0.5):
    """ calculate RVO for all object """
    rad_adj = 0
    for a in range(len(X)):
        A = X[a]
        RVO = []
        # Process RVO for each moving objects
        for b in range(len(X)):
            if b != a:
                B = X[b]
                # use Generalized RVO
                apex = [A.pos[i]+alpha*(A.vel[i] + B.vel[i]) for i in range(2)]
                dist_AB = Rob.euc_distance(A.pos, B.pos)
                # AB object centerline angle
                theta_AB = atan2(B.pos[1]-A.pos[1], B.pos[0]-A.pos[0])
                # print(f"theta_AB: {theta_AB}")
                # special case if two object collide
                if A.rad+rad_adj+B.rad+rad_adj > dist_AB:
                    dist_AB = A.rad+rad_adj+B.rad+rad_adj
                # print(f"dist_BA: {dist_AB}")
                # calculate left bound angle of RVO
                # print(f"minkowski sum: {A.rad+rad_adj+B.rad+rad_adj}")
                theta_BAort = asin((A.rad+rad_adj+B.rad+rad_adj)/dist_AB)
                # print(f"theta_BAort: {theta_BAort}")
                theta_RVO_left = theta_AB+theta_BAort
                # calculate right bound angle of RVO
                theta_RVO_right = theta_AB-theta_BAort
                # print(f"RVO: {[apex, degrees(theta_RVO_left), degrees(theta_RVO_right)]}")
                RVO.append([apex, theta_RVO_left, theta_RVO_right, dist_AB, A.rad+rad_adj+B.rad+rad_adj])

        # Process RVO for circular static objects
        for o in ws_model['circular_obstacles']:
            # o = [x, y, rad]
            pO = o[0:2]
            # using VO for static objects
            apex = A.pos
            dist_AB = Rob.euc_distance(A.pos, pO)
            theta_AB = atan2(pO[1]-A.pos[1], pO[0]-A.pos[0])
            # special case if two object collide
            if A.rad+o[2] > dist_AB:
                dist_AB = A.rad+o[2]
            # calculate left bound angle of RVO
            theta_BAort = asin((A.rad+o[2])/dist_AB)
            theta_RVO_left = theta_AB+theta_BAort
            # calculate right bound angle of RVO
            theta_RVO_right = theta_AB-theta_BAort
            RVO.append([apex, theta_RVO_left, theta_RVO_right, dist_AB, A.rad+o[2]])
        
        # Process RVO for rectangular objects
        # TODO: sample for point to convex tangents

        # get optimal velocity
        vA_post = find_Vel(A, RVO, method)
        A.new_vel = vA_post


def test_intersect(A, new_vel, RVO_all, suitable_V, unsuitable_V):
    """ calculate intersection """
    suit = True
    for RVO in RVO_all:
        apex = RVO[0]
        left = RVO[1]
        right = RVO[2]
        # calculate VO range without displacement
        vo = [new_vel[0]+A.pos[0]-apex[0], new_vel[1]+A.pos[1]-apex[1]]
        theta_dif = atan2(vo[1], vo[0])
        theta_right = atan2(sin(right), cos(right))
        theta_left = atan2(sin(left), cos(left))
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_vel)
        return True
    else:
        unsuitable_V.append(new_vel)
        return False
     
def search_nn_v(A, RVO, suitable_V, unsuitable_V, method):
    """ search for closest velocity """
    norm_v = Rob.euc_distance(A.hypo_vel, [0,0])
    theta_v = atan2(A.hypo_vel[1], A.hypo_vel[0])
    print(f"hypothetical vel: {A.hypo_vel}")

    theta_sstep = 0.1
    vel_sstep = 5.0
    if method == "dir_pri":
        # prioritize direction
        for theta_step in np.arange(0, PI, theta_sstep):
            for cur_theta in [theta_v+theta_step, theta_v-theta_step]:
                for vel_step in np.arange(0, norm_v, norm_v/vel_sstep):
                    for cur_vel in [norm_v+vel_step, norm_v-vel_step]:
                        new_v = [cur_vel*cos(cur_theta), cur_vel*sin(cur_theta)]
                        if test_intersect(A, new_v, RVO, suitable_V, unsuitable_V):
                            print(f"nn_vel: {new_v}")
                            return new_v
    
    elif method == "vel_pri":
        # prioritize velocity
        for vel_step in np.arange(0, norm_v, norm_v/vel_sstep):
            for cur_vel in [norm_v+vel_step, norm_v-vel_step]:
                for theta_step in np.arange(0, PI, theta_sstep):
                    for cur_theta in [theta_v+theta_step, theta_v-theta_step]:
                        new_v = [cur_vel*cos(cur_theta), cur_vel*sin(cur_theta)]
                        if test_intersect(A, new_v, RVO, suitable_V, unsuitable_V):
                            print(f"nn_vel: {new_v}")
                            return new_v
    
    elif method == 'round_s':
        vel_step = 5.0
        for theta in np.arange(0, 2*PI, 0.1):
            for vel in np.arange(0.02, norm_v+0.02, norm_v/vel_step):
                new_v = [vel*cos(theta), vel*sin(theta)]
                test_intersect(A, new_v, RVO, suitable_V, unsuitable_V)
        test_intersect(A, A.hypo_vel, RVO, suitable_V, unsuitable_V)
        if suitable_V:
            # find the closest velocity
            return min(suitable_V, key = lambda v : Rob.euc_distance(v, A.hypo_vel))

    else:
        #TODO: other methods 
        pass
    
    return None

def find_Vel(A, RVO, method):
    """ calculate optimzied velocity for all object """
    suitable_V = []
    unsuitable_V = []
    # search for best velocity
    nn_vel = search_nn_v(A, RVO, suitable_V, unsuitable_V, method)

    #----------------------
    # if no best velocity is found
    if nn_vel is None:
        print("Suitable not found")
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO:
                apex = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                minkwoski = RVO_BA[4]
                # calculate VO range without displacement
                translatedV = [unsuit_v[0]+A.pos[0]-apex[0], unsuit_v[1]+A.pos[1]-apex[1]]
                theta_dif = atan2(translatedV[1], translatedV[0])
                theta_right = atan2(sin(right), cos(right))
                theta_left = atan2(sin(left), cos(left))
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= minkwoski:
                        minkwoski = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/minkwoski)
                    dist_tg = abs(dist*cos(small_theta))-abs(minkwoski*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/Rob.euc_distance(translatedV, [0,0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        nn_vel = min(unsuitable_V, key = lambda v: ((WT*tc_V[tuple(v)])+Rob.euc_distance(v, A.vel)))
        # nn_vel = min(unsuitable_V, key = lambda v: (Rob.euc_distance(v, A.hypo_vel)))

    print(f"nn_vel: {nn_vel}")
    return nn_vel

def in_between(theta_right, theta_dif, theta_left):
    """check if theta_dif is between theta_left and theta_right """
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
            