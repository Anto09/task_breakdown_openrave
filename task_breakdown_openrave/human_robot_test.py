#!/usr/bin/env python

import sys
import os

import rospy
from baxter_moveit_test.srv import *

import time
import threading
import openravepy

#### YOUR IMPORTS GO HERE ####
import sys
from taskbreakdown_python import *
from utilities import Utilities
import math
import trajoptpy
import trajoptpy.kin_utils as ku
#### END OF YOUR IMPORTS ####

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
from human_robot import HumanRobot
from costwrapper import *

import TransformMatrix

import numpy as np
import math
import copy
import sys

from human_trajopt import HumanTrajopt

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

def str2num(string):
    return array([float(s) for s in string.split()])

def generate_ik_solver(robotfile, filename):
    # for generating ik solver
    env = Environment()
    kinbody = env.ReadRobotXMLFile(robotfile)
    env.Add(kinbody)
    solver = ikfast.IKFastSolver(kinbody=kinbody)
    chaintree = solver.generateIkSolver(baselink=0,eelink=16,freeindices=[5],solvefn=ikfast.IKFastSolver.solveFullIK_6D)
    code = solver.writeIkSolver(chaintree)
    open(filename,'w').write(code)

def make_fullbody_request(end_t, n_steps, manip_name, end_joint_target):
        coll_coeff = 20
        dist_pen = .05
        d = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : manip_name,
                "start_fixed" : True
            },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]}
                },            
                {
                    "name" : "cont_coll",
                    "type" : "collision",
                    "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":True}
                },
                {
                    "name": "disc_coll",
                    "type" : "collision",
                    "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":False}
                }            
            ],
            "constraints" : [
              {
                "type" : "pose", 
                "params" : {"xyz" : end_t[0:3,3].tolist(), 
                            "wxyz" : transformations.quaternion_from_matrix(end_t[0:3,0:3]).tolist(), 
                            "link": "Head",
                            "timestep" : n_steps-1
                            }
                             
              }
            ],
            "init_info" : {
                "type" : "straight_line",
                "endpoint" : end_joint_target.tolist()
            }
        }
        
        return d

def ExtendTrajoptRequest(request, waypoints):
    idx = 1
    for waypoint in waypoints:
        print 'waypoint rot target', transformations.quaternion_from_matrix(waypoint[0:3,0:3]).tolist()
        request["constraints"].extend([
            {
                "type":"pose",
                "name":"path_pose_waypoint",
                "params":{
                    "xyz": waypoint[0:3,3].tolist(),
                    "wxyz": transformations.quaternion_from_matrix(waypoint[0:3,0:3]).tolist(),
                    "link": "Head",
                    "timestep": idx
                }
            }
            ])
        idx += 1

    return request

def CalcKneeAnkleAngles(self, hip_trans, knee_trans, ankle_trans):
    l1 = np.linalg.norm(knee_trans[0:3,3] - ankle_trans[0:3,3])
    l2 = np.linalg.norm(hip_trans[0:3,3] - knee_trans[0:3,3])
    p2x = hip_trans[0,3]
    p2y = hip_trans[1,3]

    #q2 calculation
    c2 = (p2x**2 + p2y**2 - l1**2 - l2**2)/(2*l1*l2)
    s2_1 = np.sqrt(1-c2**2)
    s2_2 = -np.sqrt(1-c2**2)
    s2 = s2_1
    q2 = np.arctan2(s2_1, c2)
    if (q2 < 0):
        s2 = s2_2
        q2 = np.arctan2(s2_2, c2)

    #q1 calculation
    det = (l1**2 + l2**2 + (2*l1*l2*c2))
    s1 = (p2y*(l1+l2*c2) - p2x*l2*s2)/det
    c1 = (p2x*(l1+l2*c2) + p2y*l2*s2)/det
    q1 = np.arctan2(s1,c1)

    return q1,q2

if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    env.Load("/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/kinbodies_robots_envs/human_test.env.xml")
    time.sleep(0.1)

    utils = Utilities()
    ht = HumanTrajopt()
    ht.generate_cost_regions()

    support_path = [np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.43935400e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   1.90673274e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.43966000e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.44859184e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -3.64438239e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.43918493e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.45782661e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -8.68824824e-18],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.44161176e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.46704921e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   5.18953957e-18],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.44693812e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.47625052e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   5.18953957e-18],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.45515873e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.48597668e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   3.29451152e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.27259951e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.49390442e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -3.64438239e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   6.08676014e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50000290e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -1.05832763e-16],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.89934116e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50425743e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -5.03216117e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.71206335e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50666961e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -7.80771873e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.52665244e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50725727e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   3.29451152e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.34482372e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50605433e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -5.03216117e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.16826688e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50311047e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   5.18953957e-18],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.99863093e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.49849071e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   1.90673274e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.83750959e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.49227478e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   3.29451152e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.68642708e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.50995946e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   5.18953957e-18],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.73983830e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.52727387e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -8.68824824e-18],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.80424842e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.54414968e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -2.25660360e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.87940325e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.56052030e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   1.90673274e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   4.96500620e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.57632110e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,   3.29451152e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.06071941e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.59789082e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -7.80771873e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.33132057e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.61643961e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -1.61343914e-16],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.62000174e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]),
                np.array([[ -1.00000000e+00,  -1.22464680e-16,   0.00000000e+00,   1.63178582e+00],
                 [  1.22464680e-16,  -1.00000000e+00,   0.00000000e+00,  -6.41993995e-17],
                 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   5.92300731e-01],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])]


    InitOpenRAVELogging() 

    robot = env.GetRobots()[0]
    head = robot.GetLink('Head')
    print 'head_transform'
    print head.GetTransform()
    manip = robot.SetActiveManipulator("torso")
    robot.SetActiveDOFs(manip.GetArmIndices())
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    print ('done torso manip')

    print 'MANIP DOFS', robot.GetActiveDOFIndices()

    # manip = robot.SetActiveManipulator("base")
    # robot.SetActiveDOFs(manip.GetArmIndices())
    # ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
    # if not ikmodel.load():
    #     ikmodel.autogenerate()
    # print ('done base manip')  

    # manip = robot.SetActiveManipulator("knee")
    # robot.SetActiveDOFs(manip.GetArmIndices())
    # ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
    # if not ikmodel.load():
    #     ikmodel.autogenerate()
    # print ('done knee manip')

    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,'Human1')

    serialized_transform = TransformMatrix.SerializeTransform(support_path[len(support_path)-1])
    raw_input("Press enter to continue...")

    handles = []
    for sp in support_path:
        sp[0:3,3] += np.array([0.034094, 0.004925, 0.088688])
        sp[0:3,3] += np.array([-0.026786, 0, 0])
        handles.append(env.plot3(points=sp[0:3,3],
                                 pointsize=5.0,
                                 colors=array(((0,1,0)))))
    raw_input("Press enter to continue...")

    # for i in range(0, len(support_path)):
    #     pt = np.copy(support_path[i])
    #     pt[0:3,3] -= np.array([0.034094, 0.004925, 0.088688])
    #     pt[0:3,3] -= np.array([-0.026786, 0, 0])
    #     serialized_transform = TransformMatrix.SerializeTransform(pt)
    #     with env:
    #         startik = str2num(probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 0 %s'%serialized_transform))
    #         print ('ik solution \n', startik)
    #     raw_input("Press enter to continue...")
    

    # gr = GogglesRobot()
    # goggles  = env.GetRobots()[1]
    # gr.Init(env, goggles)
    # goggles.SetActiveDOFValues([0])
    # gr.Collapse()

    head_transform = manip.GetEndEffector().GetTransform()
    new_head_transform = np.copy(head_transform)
    new_head_transform[0,3] += 0.2
    new_head_transform[2,3] -= 0.1

    baxter = env.GetRobots()[1]
    b_manip = baxter.SetActiveManipulator("rightarm")
    b_sol = []

    goggles_trans = np.array([[ -6.12323400e-17,  -7.49879891e-33,  1.00000000e+00,  1.15732000e+00],
                              [  1.22464680e-16,  -1.00000000e+00,  0.00000000e+00,   0.00000000e+00],
                              [  1.00000000e+00,   1.22464680e-16,  6.12323400e-17,  7.32410000e-01],
                              [  0.00000000e+00,   0.00000000e+00,  0.00000000e+00,   1.00000000e+00]]
                            )
    with env:
        b_sol = ku.ik_for_link(goggles_trans, b_manip, b_manip.GetEndEffector().GetName(), 
                             filter_options = IkFilterOptions.IgnoreEndEffectorCollisions, 
                             return_all_solns = True)
    if (len(b_sol) > 0):
        baxter.SetActiveDOFs(b_manip.GetArmIndices())
        baxter.SetActiveDOFValues(b_sol[0])


    # sol = []
    # with env:
    #     sol = ku.ik_for_link(new_head_transform, manip, manip.GetEndEffector().GetName(), 
    #                          filter_options = IkFilterOptions.IgnoreSelfCollisions | IkFilterOptions.CheckEnvCollisions, 
    #                          return_all_solns = True)
    #     print "solutions",sol
    # for s in sol:
    #     with env:
    #         robot.SetActiveDOFs(manip.GetArmIndices())
    #         robot.SetActiveDOFValues(s)
    #     raw_input("Press enter to continue...")

    # hr = HumanRobot()
    # hr.Init(env, robot)
    # hr.TestTrajopt()

    # generate_ik_solver('/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/human_bio_two_arms_mod.xml', 
    #                    '/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/human_head_ik.cpp', )

    raw_input("Press enter to continue...")


    manip = robot.SetActiveManipulator("torso")
    target = support_path[len(support_path)-1]
    target[0:3,3] -= np.array([0.034094, 0.004925, 0.088688])
    target[0:3,3] -= np.array([-0.026786, 0, 0])
    sol = ku.ik_for_link(target, manip, manip.GetEndEffector().GetName(), 
                         filter_options = IkFilterOptions.IgnoreSelfCollisions | IkFilterOptions.CheckEnvCollisions, 
                         return_all_solns = False)
    request = make_fullbody_request(support_path[len(support_path)-1], len(support_path), "torso", sol)
    c_waypoints = []
    # for i in range(0, len(support_path)-1):
    #     waypoint = np.copy(support_path[i])
    #     waypoint[0:3,3] -= np.array([0.034094, 0.004925, 0.088688])
    #     waypoint[0:3,3] -= np.array([-0.026786, 0, 0])
    #     c_waypoints.append(waypoint)

    # request = ExtendTrajoptRequest(request, c_waypoints)
    print request
                
    s = json.dumps(request) # convert dictionary into json-formatted string


    cost_handles = []
    with env:  
        prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
        waypoints = []
        for i in range(0, len(support_path)-1):
            waypoint = np.copy(support_path[i])
            waypoint[0:3,3] -= np.array([0.034094, 0.004925, 0.088688])
            waypoint[0:3,3] -= np.array([-0.026786, 0, 0])
            waypoints.append(waypoint)
            co = CostObject()
            co.Init(waypoint, None, env, robot, utils, manip, None, None)
            if (i > 0):
                co.parent_node = cost_handles[i-1]
                co.parent_node.child_node = co
            cost_handles.append(co)
            prob.AddCost(co.TaskDeviationCost, [(i,j) for j in xrange(7)], "ABS")#, "up%i"%t)
            prob.AddCost(ht.get_gradient_cost, [(i,j) for j in xrange(7)], "ABS")

    traj = None
    with env:  
        result = trajoptpy.OptimizeProblem(prob) # do optimization
        traj = result.GetTraj()
        print traj

    robot.SetActiveDOFs(manip.GetArmIndices())
    for t in traj:
        with env:
            robot.SetActiveDOFValues(t)
        pt = manip.GetEndEffector().GetTransform()
        pt[0:3,3] += np.array([0.034094, 0.004925, 0.088688])
        pt[0:3,3] += np.array([-0.026786, 0, 0])
        handles.append(env.plot3(points=pt[0:3,3],
                                 pointsize=5.0,
                                 colors=array(((0,0,1)))))
        raw_input("Press enter to continue...")
    # for sp_trans in support_path:
    #     sp = np.copy(sp_trans)
    #     sp[0:3,3] -= np.array([0.034094, 0.004925, 0.088688])
    #     sp[0:3,3] -= np.array([-0.026786, 0, 0])
    #     # sp[0:3,3] -= np.array([0.034094, 0.004925, 0.088688])

    #     manip = robot.SetActiveManipulator("torso")
    #     sol = []
    #     with env:
    #         sol = ku.ik_for_link(sp, manip, manip.GetEndEffector().GetName(), 
    #                              filter_options = IkFilterOptions.IgnoreSelfCollisions | IkFilterOptions.CheckEnvCollisions, 
    #                              return_all_solns = False)
    #     if (len(sol) > 0):
    #         with env:
    #             robot.SetActiveDOFs(manip.GetArmIndices())
    #             robot.SetActiveDOFValues(sol)
    #     else:
    #         manip = robot.SetActiveManipulator("base")
    #         with env:
    #             sol = ku.ik_for_link(sp, manip, manip.GetEndEffector().GetName(), 
    #                                  filter_options = IkFilterOptions.IgnoreSelfCollisions | IkFilterOptions.CheckEnvCollisions, 
    #                                  return_all_solns = False)
    #         if (len(sol) > 0):
    #             with env:
    #                 robot.SetActiveDOFs(manip.GetArmIndices())
    #                 robot.SetActiveDOFValues(sol)

    #     pt = manip.GetEndEffector().GetTransform()
    #     pt[0:3,3] += np.array([0.034094, 0.004925, 0.088688])
    #     pt[0:3,3] += np.array([-0.026786, 0, 0])
    #     handles.append(env.plot3(points=pt[0:3,3],
    #                              pointsize=5.0,
    #                              colors=array(((0,0,1)))))

    #     raw_input("Press enter to continue...")

    raw_input("Press enter to exit...")