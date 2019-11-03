#!/usr/bin/env python

import sys
import os

import rospy
from baxter_moveit_test.srv import *

import time
import threading
import openravepy
import trajoptpy
import trajoptpy.kin_utils as ku

#### YOUR IMPORTS GO HERE ####
import sys
from taskbreakdown_python import *
from utilities import Utilities
import math
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
from apron_robot import ApronRobot

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',
                      'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50,
                                   0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]);         
        #robot.GetController().SetDesired(robot.GetDOFValues());
    #waitrobot(robot)

    raw_input("continue...")

def initialize():
    path = ObtainTransferPath()

def move_human_to_ready(tb, handles, env, apron, ppe_loc, apron_robot):
    right_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    left_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    torso_angles = [0, 0, 0, 0, 0, 0, np.pi, 0, 0]

    tb.MoveHumanUpperSingle(right_angles, left_angles)

    shoulder_angle_inc = np.pi * 0.03
    elbow_angle_inc = -np.pi * 0.03
    traj_len = 6

    rhand = tb.human.GetLink('rHand')
    lhand = tb.human.GetLink('lHand')

    prev_rtt = rhand.GetTransform()
    prev_rt1 = np.dot(prev_rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

    prev_ltt = lhand.GetTransform()
    prev_lt1 = np.dot(prev_ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
    prev_ppet = (prev_rt1+prev_lt1)*0.5

    prev_apron_trans = apron.GetTransform()
    prev_apron_loc = prev_ppet
    prev_apron_trans[0:3,3] = prev_apron_loc
    apron.SetTransform(prev_apron_trans)
    # apron_robot.RayCollisionCheckWithMove(prev_apron_trans)

    ppe_loc.append(prev_ppet)
    apron_trans = apron.GetTransform()
    rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                         [np.sin(np.pi),  np.cos(np.pi), 0],
                         [0, 0, 1]])
    apron_trans[0:3, 0:3] = np.dot(apron_trans[0:3, 0:3], rot_mat)

    ppe_trans.append(apron_trans)

    tb.neutral_ppe_trans = apron.GetTransform()

def test_motion(tb, handles, env, apron, ppe_loc, ppe_trans, apron_robot):

    shoulder_z_idx = 1
    elbow_z_idx = 3
    wrist_x_idx = 4

    right_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    left_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    torso_angles = [0, 0, 0, 0, 0, 0, np.pi, 0, 0]

    #pull forward
    traj_len = 4
    # traj_len = 3
    shoulder_angle_inc = 0
    elbow_angle_inc = -np.pi * 0.01
    rhand = tb.human.GetLink('rHand')
    lhand = tb.human.GetLink('lHand')

    prev_rtt = rhand.GetTransform()
    prev_rt1 = np.dot(prev_rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

    prev_ltt = lhand.GetTransform()
    prev_lt1 = np.dot(prev_ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
    prev_ppet = (prev_rt1+prev_lt1)*0.5

    prev_apron_trans = apron.GetTransform()
    prev_apron_loc = prev_ppet
    prev_apron_trans[0:3,3] = prev_apron_loc
    # apron.SetTransform(prev_apron_trans)
    apron_robot.RayCollisionCheckWithMove(prev_apron_trans)

    prev_state = apron_robot.out_of_collision
    index = 0

    for i in range(0, traj_len):
        index += 1
        right_angles[shoulder_z_idx] += shoulder_angle_inc
        right_angles[elbow_z_idx] += elbow_angle_inc

        left_angles[shoulder_z_idx] += shoulder_angle_inc
        left_angles[elbow_z_idx] += elbow_angle_inc

        tb.MoveHumanUpperSingle(right_angles, left_angles)

        rt = rhand.GetTransform()[0:3,3]
        rtt = rhand.GetTransform()
        rt1 = np.dot(rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

        lt = lhand.GetTransform()[0:3,3]
        ltt = lhand.GetTransform()
        lt1 = np.dot(ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
        ppet = (rt1+lt1)*0.5
        d_ppet = ppet - prev_ppet
        prev_ppet = ppet

        apron_trans = apron.GetTransform()
        #apron_loc = apron_trans[0:3,3] + d_ppet
        apron_loc = ppet
        apron_trans[0:3,3] = apron_loc

        # apron.SetTransform(apron_trans)
        apron_robot.RayCollisionCheckWithMove(apron_trans)
        if (apron_robot.out_of_collision != prev_state):
            tb.state_change_index = index
        prev_state = apron_robot.out_of_collision

        ppe_loc.append(ppet)
        apron_trans = apron.GetTransform()
        rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                             [np.sin(np.pi),  np.cos(np.pi), 0],
                             [0, 0, 1]])
        apron_trans[0:3, 0:3] = np.dot(apron_trans[0:3, 0:3], rot_mat)

        ppe_trans.append(apron_trans)

        #ppet = apron.GetTransform()[0:3,3]

        if (i == 0):
            handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                       pointsize=5.0,
                                       colors=array(((1,0.5,0)))))
        else:
            handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                       pointsize=5.0,
                                       colors=array(((1,0,0)))))

    #pull up
    traj_len = 10
    shoulder_angle_inc = np.pi * 0.03
    elbow_angle_inc = -np.pi * 0.035

    rhand = tb.human.GetLink('rHand')
    lhand = tb.human.GetLink('lHand')

    prev_rtt = rhand.GetTransform()
    prev_rt1 = np.dot(prev_rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

    prev_ltt = lhand.GetTransform()
    prev_lt1 = np.dot(prev_ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
    prev_ppet = (prev_rt1+prev_lt1)*0.5

    prev_apron_trans = apron.GetTransform()
    prev_apron_loc = prev_ppet
    prev_apron_trans[0:3,3] = prev_apron_loc

    apron.SetTransform(prev_apron_trans)
    if (apron_robot.out_of_collision != prev_state):
        tb.state_change_index = i
    prev_state = apron_robot.out_of_collision
    for i in range(0, traj_len):
        index += 1
        right_angles[shoulder_z_idx] += shoulder_angle_inc
        right_angles[elbow_z_idx] += elbow_angle_inc

        left_angles[shoulder_z_idx] += shoulder_angle_inc
        left_angles[elbow_z_idx] += elbow_angle_inc

        tb.MoveHumanUpperSingle(right_angles, left_angles)

        rt = rhand.GetTransform()[0:3,3]
        rtt = rhand.GetTransform()
        rt1 = np.dot(rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

        lt = lhand.GetTransform()[0:3,3]
        ltt = lhand.GetTransform()
        lt1 = np.dot(ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
        ppet = (rt1+lt1)*0.5
        d_ppet = ppet - prev_ppet
        prev_ppet = ppet

        apron_trans = apron.GetTransform()
        #apron_loc = apron_trans[0:3,3] + d_ppet
        apron_loc = ppet
        apron_trans[0:3,3] = apron_loc
        # apron.SetTransform(apron_trans)
        apron_robot.RayCollisionCheckWithMove(apron_trans)
        if (apron_robot.out_of_collision != prev_state):
            tb.state_change_index = index
        prev_state = apron_robot.out_of_collision

        ppe_loc.append(ppet)

        apron_trans = apron.GetTransform()
        rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                             [np.sin(np.pi),  np.cos(np.pi), 0],
                             [0, 0, 1]])
        apron_trans[0:3, 0:3] = np.dot(apron_trans[0:3, 0:3], rot_mat)

        ppe_trans.append(apron_trans)

        #ppet = apron.GetTransform()[0:3,3]
        handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))

    #pull down
    traj_len = 8
    shoulder_angle_inc = 0
    elbow_angle_inc = -np.pi * 0.02

    prev_rtt = rhand.GetTransform()
    prev_rt1 = np.dot(prev_rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

    prev_ltt = lhand.GetTransform()
    prev_lt1 = np.dot(prev_ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
    prev_ppet = (prev_rt1+prev_lt1)*0.5

    prev_apron_trans = apron.GetTransform()
    prev_apron_loc = prev_ppet
    prev_apron_trans[0:3,3] = prev_apron_loc
    # apron.SetTransform(prev_apron_trans)
    apron_robot.RayCollisionCheckWithMove(prev_apron_trans)

    for i in range(0, traj_len):
        index += i
        right_angles[shoulder_z_idx] += shoulder_angle_inc
        right_angles[elbow_z_idx] += elbow_angle_inc
        left_angles[shoulder_z_idx] += shoulder_angle_inc
        left_angles[elbow_z_idx] += elbow_angle_inc

        tb.MoveHumanUpperSingle(right_angles, left_angles)


        rt = rhand.GetTransform()[0:3,3]
        rtt = rhand.GetTransform()
        rt1 = np.dot(rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

        lt = lhand.GetTransform()[0:3,3]
        ltt = lhand.GetTransform()
        lt1 = np.dot(ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
        ppet = (rt1+lt1)*0.5
        d_ppet = ppet - prev_ppet
        prev_ppet = ppet

        apron_trans = apron.GetTransform()
        #apron_loc = apron_trans[0:3,3] + d_ppet
        apron_loc = ppet
        apron_trans[0:3,3] = apron_loc
        # apron.SetTransform(apron_trans)
        apron_robot.RayCollisionCheckWithMove(apron_trans)
        if (apron_robot.out_of_collision != prev_state):
            tb.state_change_index = index
        prev_state = apron_robot.out_of_collision

        ppe_loc.append(ppet)
        apron_trans = apron.GetTransform()
        rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                             [np.sin(np.pi),  np.cos(np.pi), 0],
                             [0, 0, 1]])
        apron_trans[0:3, 0:3] = np.dot(apron_trans[0:3, 0:3], rot_mat)

        ppe_trans.append(apron_trans)
        #ppet = apron.GetTransform()[0:3,3]
        handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))

        if (i > 3):
            elbow_angle_inc = np.pi * 0.03
            shoulder_angle_inc = -np.pi * 0.04

    direction = [0, 1, 0]
    angle = np.pi*0.5
    rot_mat = transformations.rotation_matrix(angle, direction)

    idx = 0
    for ppe_trans_point in ppe_trans:
        ppe_trans_point[0:3, 0:3] = np.dot(rot_mat[0:3, 0:3], ppe_trans_point[0:3, 0:3])
        ppe_trans[idx] = ppe_trans_point

        from_vec = ppe_trans_point[0:3,3]

        to_vec_1 = from_vec + 0.05*(ppe_trans_point[0:3,0])
        to_vec_2 = from_vec + 0.05*(ppe_trans_point[0:3,1])
        to_vec_3 = from_vec + 0.05*(ppe_trans_point[0:3,2])

        idx += 1

    handles.append(env.plot3(points=ppe_trans[tb.state_change_index][0:3,3],
                               pointsize=5.0,
                               colors=array(((0,1,0)))))

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    env.Load("/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/empty.env.xml")
    time.sleep(0.1)

    InitOpenRAVELogging() 
    trajoptpy.SetInteractive(False) # pause every iteration, until you press 'p'. Press escape to disable further plotting
    
    # raw_input("Press enter to exit...")

    robot    = env.GetRobots()[0]
    human    = env.GetRobots()[1]
    apron  = env.GetRobots()[2]
    forehead = env.GetKinBody('forehead')

    tb = TaskBreakdown()
    gr = ApronRobot()
    gr.Init(env, apron)
    handles = []
    tb.Init(robot, env, human, apron, gr, forehead, handles)

    tuckarms(env, robot)

    ppe_loc = []
    ppe_trans = []

    move_human_to_ready(tb, handles, env, apron, ppe_loc, gr)
    tb.CacheBodyTransforms()

    test_motion(tb, handles, env, apron, ppe_loc, ppe_trans, gr)

    tb.ObtainClosestBodyTransform(ppe_loc)
    tb.PurifyIntoTransfer(ppe_trans, env)
    tb.PurifyIntoSupport(ppe_trans, env)

    tb.FindBestDisplacement()
    raw_input("Press enter to generate reaching motions for right arm...")

    tb.GenerateReachingForTransfer(tb.r_manip)
    raw_input("Press enter to trajopt transfer path...")

    tb.TrajoptTransferPath()
    raw_input("Press enter to exit...")