#!/usr/bin/env python

import sys
import os

import rospy
import time
import threading
import openravepy

import sys
from utilities import Utilities
import math
import trajoptpy
import trajoptpy.kin_utils as ku

from openravepy import ikfast

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import InitOpenRAVELogging
import geometry_msgs.msg
from geometry_msgs.msg import *

import transformations
import numpy as np
from utilities import Utilities

import TransformMatrix

import numpy as np
import math
import copy
import sys

MAX_VAL = 180

class CostRegion:
    dim = 7
    vertices = np.zeros(shape=(7,2))
    center = np.zeros(7)
    cost  = np.zeros(7)
    total_cost = 0

    def init(self, vertices):
        self.vertices = vertices

    def calc_center(self):
        for d in range(0, self.dim):
            self.center[d] = (self.vertices[d][1] + self.vertices[d][0])*0.5

    def get_dist_center(self, config):
        return np.linalg.norm(config - self.center)

    def get_dist_boundary(self, config):
        dist = 0;
        for c in range(0,self.dim):
            dist = min(min(np.fabs(self.vertices[c][0] - config[c]), np.fabs(self.vertices[c][1] - config[c])), dist)
        return dist

    def set_cost(self, c1, c2, c3, c4, c5, c6, c7):
        self.cost[0] = c1
        self.cost[1] = c2
        self.cost[2] = c3
        self.cost[3] = c4
        self.cost[4] = c5
        self.cost[5] = c6
        self.cost[6] = c7
        self.total_cost = np.sum(self.cost)

    def inside_region(self, config):
        inside = True
        for i in range(0, self.dim):
            inside = inside and config[i] >= self.vertices[i][0] and config[i] <= self.vertices[i][1]
        return inside

    def is_neighbor(self, cost_region):
        neighbor = True
        for i in range(0, self.dim):
            c_dist = np.fabs(self.center[i] - cost_region.center[i])
            extents_a = np.fabs(self.vertices[i][0] - self.vertices[i][1]) * 0.5
            extents_b = np.fabs(cost_region.vertices[i][0] - cost_region.vertices[i][1]) * 0.5
            neighbor = neighbor and np.fabs((extents_b + extents_a) - c_dist) > sys.float_info.epsilon
        return neighbor

class HumanTrajopt:
    cost_regions = []

    def generate_cost_regions(self):
        for i in range(0,4):
            c1_0 = 0
            c1_1 = 0
            c1_cost = 0
            if (i == 0):
                c1_0 = 0
                c1_1 = 10
                c1_cost = 1
            elif (i == 1):
                c1_0 = 10 + sys.float_info.epsilon
                c1_1 = 20
                c1_cost = 2
            elif (i == 2):
                c1_0 = 20 + sys.float_info.epsilon
                c1_1 = MAX_VAL
                c1_cost = 3
            else:
                c1_0 = -MAX_VAL
                c1_1 = -sys.float_info.epsilon
                c1_cost = 4
            for j in range(0,3):
                c2_0 = 0
                c2_1 = 0
                c2_cost = 0
                if (j == 0):
                    c2_0 = 0
                    c2_1 = 0
                    c2_cost = 0
                elif (j == 1):
                    c2_0 = sys.float_info.epsilon
                    c2_1 = MAX_VAL
                    c2_cost = 1
                elif (j == 2):
                    c2_0 =  -MAX_VAL
                    c2_1 = -sys.float_info.epsilon
                    c2_cost = 1
                for k in range(0,3):
                    c3_0 = 0
                    c3_1 = 0
                    c3_cost = 0
                    if (k == 0):
                        c3_0 = 0
                        c3_1 = 0
                        c3_cost = 0
                    elif (k == 1):
                        c3_0 = sys.float_info.epsilon
                        c3_1 = MAX_VAL
                        c3_cost = 1
                    elif (k == 2):
                        c3_0 =  -MAX_VAL
                        c3_1 = -sys.float_info.epsilon
                        c3_cost = 1
                    for l in range(0,5):
                        c4_0 = 0
                        c4_1 = 0
                        c4_cost = 0
                        if (l == 0):
                            c4_0 = 0
                            c4_1 = 20
                            c4_cost = 1
                        elif (l == 1):
                            c4_0 = 20 + sys.float_info.epsilon
                            c4_1 = 60
                            c4_cost = 2
                        elif (l == 2):
                            c4_0 = 60 + sys.float_info.epsilon
                            c4_1 = 120
                            c4_cost = 3
                        elif (l == 3):
                            c4_0 = 120
                            c4_1 = MAX_VAL
                            c4_cost = 4
                        else:
                            c4_0 = -MAX_VAL
                            c4_1 = -sys.float_info.epsilon
                            c4_cost = 4
                        for o in range(0,3):
                            c5_0 = 0
                            c5_1 = 0
                            c5_cost = 0
                            if (o == 0):
                                c5_0 = 0
                                c5_1 = 0
                                c5_cost = 0
                            elif (o == 1):
                                c5_0 = sys.float_info.epsilon
                                c5_1 = MAX_VAL
                                c5_cost = 1
                            elif (o == 2):
                                c5_0 =  -MAX_VAL
                                c5_1 = -sys.float_info.epsilon
                                c5_cost = 1
                            for p in range(0,3):
                                c6_0 = 0
                                c6_1 = 0
                                c6_cost = 0
                                if (p == 0):
                                    c6_0 = 0
                                    c6_1 = 0
                                    c6_cost = 0
                                elif (p == 1):
                                    c6_0 = sys.float_info.epsilon
                                    c6_1 = MAX_VAL
                                    c6_cost = 1
                                elif (p == 2):
                                    c6_0 =  -MAX_VAL
                                    c6_1 = -sys.float_info.epsilon
                                    c6_cost = 1
                                for u in range(0,4):
                                    c7_0 = 0
                                    c7_1 = 0
                                    c7_cost = 0
                                    if (u == 0):
                                        c7_0 = 0
                                        c7_1 = 30
                                        c7_cost = 0
                                    elif (u == 1):
                                        c7_0 = 30 + sys.float_info.epsilon
                                        c7_1 = 60
                                        c7_cost = 1
                                    elif (l == 2):
                                        c7_0 = 60 + sys.float_info.epsilon
                                        c4_1 = 120
                                        c7_cost = 1
                                    else:
                                        c7_0 = -MAX_VAL
                                        c7_1 = -sys.float_info.epsilon
                                        c7_cost = 2
                                    cr = CostRegion()
                                    vertices = np.zeros(shape=(7,2))
                                    vertices[0][0] = c1_0
                                    vertices[0][1] = c1_1
                                    vertices[1][0] = c2_0
                                    vertices[1][1] = c2_1
                                    vertices[2][0] = c3_0
                                    vertices[2][1] = c3_1
                                    vertices[3][0] = c4_0
                                    vertices[3][1] = c4_1
                                    vertices[4][0] = c5_0
                                    vertices[4][1] = c5_1
                                    vertices[5][0] = c6_0
                                    vertices[5][1] = c6_1
                                    vertices[6][0] = c7_0
                                    vertices[6][1] = c7_1
                                    cr.vertices = np.copy(vertices)
                                    cr.set_cost(c1_cost, c2_cost, c3_cost, c4_cost, c5_cost, c6_cost, c7_cost)
                                    cr.calc_center()
                                    self.cost_regions.append(copy.deepcopy(cr))

    def get_region_inside(self, config):
        idx = 0
        for cost_region in self.cost_regions:
            if (cost_region.inside_region(config)):
                return idx
            idx += 1
        return -1

    def get_closest_cost_region(self, config):
        idx = 0
        closest_idx = 0
        inside_idx = self.get_region_inside(config)
        inside_cost = self.cost_regions[inside_idx].total_cost
        dist = sys.float_info.max
        for cost_region in self.cost_regions:
            if (cost_region.get_dist_boundary(config) < dist and idx != inside_idx and cost_region.total_cost < inside_cost):
                dist = cost_region.get_dist_boundary(config)
                closest_idx = idx
            idx += 1
        return self.cost_regions[closest_idx]

    def get_lowest_cost_region(self, config):
        idx = 0
        closest_idx = 0
        inside_idx = self.get_region_inside(config)
        inside_cost = self.cost_regions[inside_idx].total_cost
        inside_region = self.cost_regions[inside_idx]
        for cost_region in self.cost_regions:
            if (cost_region.is_neighbor(inside_region) and idx != inside_idx and cost_region.total_cost < inside_cost):
                inside_cost = cost_region.total_cost
                closest_idx = idx
            idx += 1
        return self.cost_regions[closest_idx]

    def get_gradient_cost(self, config):
        new_config = None
        if (type(config) == np.ndarray or type(config) == np.array):
            new_config = config
        else:
            new_config = np.zeros(len(config))
            for i in range(0, len(config)):
                new_config[i] = config[i]
        new_config *= (180/np.pi)

        lowest_cost_region = self.get_lowest_cost_region(new_config)
        return lowest_cost_region.get_dist_boundary(new_config)