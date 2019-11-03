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
from goggles_robot import GogglesRobot

class PlotSpinner(threading.Thread):
    def __init__(self,handle):
        threading.Thread.__init__(self)
        self.starttime = time.time()
        self.handle=handle
        self.ok = True
    def run(self):
        while self.ok:
            self.handle.SetTransform(matrixFromAxisAngle([0,mod(time.time()-self.starttime,2*pi),0]))
            self.handle.SetShow(bool(mod(time.time()-self.starttime,2.0) < 1.0))

def addPoint(p1, p2):
    p3 = Point()
    p3.x = p1.x + p2.x
    p3.y = p1.y + p2.y
    p3.z = p1.z + p2.z

    return p3

def init(robot_pos):
    print "initializing"

def test_client(x):
    print "waiting for service"
    rospy.wait_for_service('test')
    print "done waiting for service"
    
    try:
        print_int = rospy.ServiceProxy('test', test)
        
        print "Requesting %s"%(x)
        
        resp1 = print_int(x)

        print "Got",resp1.test_int
        # formal style
        # resp2 = test_srv.call(testRequest(x))

        if not resp1.test_int == x:
            raise Exception("test failure, returned test_int was %s"%resp1.test_int)
        # if not resp2.test_int == x:
        #   raise Exception("test failure, returned test_int was %s"%resp2.test_int)
        return resp1.test_int
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

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

def handle_for_openrave_ik(req):
    print "doing ik"

def generate_ik_solver(robotfile, filename):
    # for generating ik solver
    env = Environment()
    kinbody = env.ReadRobotXMLFile(robotfile)
    env.Add(kinbody)
    solver = ikfast.IKFastSolver(kinbody=kinbody)
    chaintree = solver.generateIkSolver(baselink=1,eelink=9,freeindices=[5],solvefn=ikfast.IKFastSolver.solveFullIK_6D)
    code = solver.writeIkSolver(chaintree)
    open(filename,'w').write(code)

def openrave_ik_server():
    rospy.init_node('openrave_ik_server')
    s = rospy.Service('openrave_ik', OpenRaveIK, handle_for_openrave_ik)
    print "ready to perform IK"
    rospy.spin()

def initialize():
    path = ObtainTransferPath()

def plot_point():
    spinner = None
    handles = []
    try:
        handles = []
        # handles.append(env.plot3(points=array(((0.325,-0.325, 0.725))),
        #                            pointsize=35.0,
        #                            colors=array(((1,0,0)))))
        handles.append(env.plot3(points=array(((0.325,-0.65, 0.725))),
                                   pointsize=35.0,
                                   colors=array(((1,0,0)))))
        # vpos = numpy.array([0.325, -0.325, 0.725])
        # extents = numpy.array([0.05, 0.05, 0.05])
        # handles.append(env.drawbox(vpos, extents))
       # spin one of the plots in another thread
        spinner = PlotSpinner(handles[-1])
        Tcamera = eye(4)
        Tcamera[0:3,3] = [-0.37, 0.26, 3.3]
        raw_input('Enter any key to quit. ')
        handles = None
    finally:
        if spinner is not None:
            spinner.ok = False

def move_human_to_ready(tb, handles, env, goggles, ppe_loc, goggles_robot):
    right_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    left_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    torso_angles = [0, 0, 0, 0, 0, 0, np.pi, 0, 0]

    # right_angles = [0, np.pi * 0.4, 0, (np.pi * 0.6) - (np.pi * 0.01), np.pi * 0.3, 0, 0]
    # left_angles  = [0, np.pi * 0.4, 0, (np.pi * 0.6) - (np.pi * 0.01), np.pi * 0.3, 0, 0]
    # torso_angles = [0, 0, 0, 0, 0, 0, np.pi, 0, 0]

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

    prev_goggles_trans = goggles.GetTransform()
    prev_goggles_loc = prev_ppet
    prev_goggles_trans[0:3,3] = prev_goggles_loc
    goggles.SetTransform(prev_goggles_trans)
    # goggles_robot.Simulate(prev_goggles_trans)

    ppe_loc.append(prev_ppet)
    goggles_trans = goggles.GetTransform()
    rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                         [np.sin(np.pi),  np.cos(np.pi), 0],
                         [0, 0, 1]])
    goggles_trans[0:3, 0:3] = np.dot(goggles_trans[0:3, 0:3], rot_mat)

    ppe_trans.append(goggles_trans)

    tb.neutral_ppe_trans = goggles.GetTransform()

def test_motion(tb, handles, env, goggles, ppe_loc, ppe_trans, goggles_robot):

    shoulder_z_idx = 1
    elbow_z_idx = 3
    wrist_x_idx = 4

    right_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    left_angles = [0, np.pi * 0.4, 0, np.pi * 0.6, np.pi * 0.3, 0, 0]
    torso_angles = [0, 0, 0, 0, 0, 0, np.pi, 0, 0]

    # right_angles = [0, np.pi * 0.4, 0, (np.pi * 0.6) - (np.pi * 0.01), np.pi * 0.3, 0, 0]
    # left_angles  = [0, np.pi * 0.4, 0, (np.pi * 0.6) - (np.pi * 0.01), np.pi * 0.3, 0, 0]
    # torso_angles = [0, 0, 0, 0, 0, 0, np.pi, 0, 0]

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

    prev_goggles_trans = goggles.GetTransform()
    prev_goggles_loc = prev_ppet
    prev_goggles_trans[0:3,3] = prev_goggles_loc
    # goggles.SetTransform(prev_goggles_trans)
    goggles_robot.Simulate(prev_goggles_trans)

    prev_state = goggles_robot.out_of_collision
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

        goggles_trans = goggles.GetTransform()
        #goggles_loc = goggles_trans[0:3,3] + d_ppet
        goggles_loc = ppet
        goggles_trans[0:3,3] = goggles_loc

        # goggles.SetTransform(goggles_trans)
        goggles_robot.Simulate(goggles_trans)
        if (goggles_robot.out_of_collision != prev_state):
            tb.state_change_index = index
        prev_state = goggles_robot.out_of_collision

        ppe_loc.append(ppet)
        goggles_trans = goggles.GetTransform()
        rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                             [np.sin(np.pi),  np.cos(np.pi), 0],
                             [0, 0, 1]])
        goggles_trans[0:3, 0:3] = np.dot(goggles_trans[0:3, 0:3], rot_mat)

        ppe_trans.append(goggles_trans)

        #ppet = goggles.GetTransform()[0:3,3]

        if (i == 0):
            handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                       pointsize=5.0,
                                       colors=array(((1,0.5,0)))))
        else:
            handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                       pointsize=5.0,
                                       colors=array(((1,0,0)))))

        # print 'human in collision with goggles?', env.CheckCollision(tb.human, goggles)
        # raw_input('press enter to continue...')


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

    prev_goggles_trans = goggles.GetTransform()
    prev_goggles_loc = prev_ppet
    prev_goggles_trans[0:3,3] = prev_goggles_loc

    goggles.SetTransform(prev_goggles_trans)
    if (goggles_robot.out_of_collision != prev_state):
        tb.state_change_index = i
    prev_state = goggles_robot.out_of_collision
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
        # print 'rt', rt1
        # print 'lt', lt1
        # print 'ppet', ppet
        # print 'd_ppet', d_ppet
        prev_ppet = ppet

        goggles_trans = goggles.GetTransform()
        #goggles_loc = goggles_trans[0:3,3] + d_ppet
        goggles_loc = ppet
        goggles_trans[0:3,3] = goggles_loc
        # goggles.SetTransform(goggles_trans)
        goggles_robot.Simulate(goggles_trans)
        if (goggles_robot.out_of_collision != prev_state):
            tb.state_change_index = index
        prev_state = goggles_robot.out_of_collision

        ppe_loc.append(ppet)

        goggles_trans = goggles.GetTransform()
        rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                             [np.sin(np.pi),  np.cos(np.pi), 0],
                             [0, 0, 1]])
        goggles_trans[0:3, 0:3] = np.dot(goggles_trans[0:3, 0:3], rot_mat)

        ppe_trans.append(goggles_trans)

        #ppet = goggles.GetTransform()[0:3,3]
        handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))


        # print 'human in collision with goggles?', env.CheckCollision(tb.human, goggles)
        # raw_input('press enter to continue...')

    #pull down
    traj_len = 8
    shoulder_angle_inc = 0
    elbow_angle_inc = -np.pi * 0.02

    prev_rtt = rhand.GetTransform()
    prev_rt1 = np.dot(prev_rtt,rhand.GetGeometries()[0].GetTransform())[0:3,3]

    prev_ltt = lhand.GetTransform()
    prev_lt1 = np.dot(prev_ltt,lhand.GetGeometries()[0].GetTransform())[0:3,3]
    prev_ppet = (prev_rt1+prev_lt1)*0.5

    prev_goggles_trans = goggles.GetTransform()
    prev_goggles_loc = prev_ppet
    prev_goggles_trans[0:3,3] = prev_goggles_loc
    # goggles.SetTransform(prev_goggles_trans)
    goggles_robot.Simulate(prev_goggles_trans)

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
        # print 'rt', rt1
        # print 'lt', lt1
        # print 'ppet', ppet
        # print 'd_ppet', d_ppet
        prev_ppet = ppet

        goggles_trans = goggles.GetTransform()
        #goggles_loc = goggles_trans[0:3,3] + d_ppet
        goggles_loc = ppet
        goggles_trans[0:3,3] = goggles_loc
        # goggles.SetTransform(goggles_trans)
        goggles_robot.Simulate(goggles_trans)
        if (goggles_robot.out_of_collision != prev_state):
            tb.state_change_index = index
        prev_state = goggles_robot.out_of_collision

        ppe_loc.append(ppet)
        goggles_trans = goggles.GetTransform()
        rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                             [np.sin(np.pi),  np.cos(np.pi), 0],
                             [0, 0, 1]])
        goggles_trans[0:3, 0:3] = np.dot(goggles_trans[0:3, 0:3], rot_mat)

        ppe_trans.append(goggles_trans)
        #ppet = goggles.GetTransform()[0:3,3]
        handles.append(env.plot3(points=array(((ppet[0],ppet[1], ppet[2]))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))

        if (i > 3):
            elbow_angle_inc = np.pi * 0.03
            shoulder_angle_inc = -np.pi * 0.04

        # print 'human in collision with goggles?', env.CheckCollision(tb.human, goggles)
        # raw_input('press enter to continue...')

    direction = [0, 1, 0]
    angle = np.pi*0.5
    rot_mat = transformations.rotation_matrix(angle, direction)

    # print 'rotating ppe path transformations'

    idx = 0
    for ppe_trans_point in ppe_trans:
        ppe_trans_point[0:3, 0:3] = np.dot(rot_mat[0:3, 0:3], ppe_trans_point[0:3, 0:3])
        ppe_trans[idx] = ppe_trans_point

        from_vec = ppe_trans_point[0:3,3]

        # print ('vectors')
        # print (ppe_trans_point[0:3,0])
        # print (ppe_trans_point[0:3,1])
        # print (ppe_trans_point[0:3,2])

        to_vec_1 = from_vec + 0.05*(ppe_trans_point[0:3,0])
        to_vec_2 = from_vec + 0.05*(ppe_trans_point[0:3,1])
        to_vec_3 = from_vec + 0.05*(ppe_trans_point[0:3,2])

        # handles.append(env.drawarrow(from_vec, to_vec_1, 0.005, [1, 0, 0]))
        # handles.append(env.drawarrow(from_vec, to_vec_2, 0.005, [0, 1, 0]))
        # handles.append(env.drawarrow(from_vec, to_vec_3, 0.005, [0, 0, 1]))
        idx += 1
        # print 'rot_mat \n',rot_mat
        # print ppe_trans_point

    # print 'reprinting ppe path transformations'
    # for ppe_trans_point in ppe_trans:
    #     print ppe_trans_point

    handles.append(env.plot3(points=ppe_trans[tb.state_change_index][0:3,3],
                               pointsize=5.0,
                               colors=array(((0,1,0)))))


def move_arm_test(env, robot):
    time.sleep(1)
    jointnames = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    if (sol != None and len(sol) > 0):
        for s in sol:
            with env:
                robot.SetActiveDOFValues(s);         
                robot.GetController().SetDesired(robot.GetDOFValues());
            waitrobot(robot)
            # time.sleep(5)

    right_joint_names = ['rShoulderX', 'rShoulderZ', 'rShoulderY', 'rElbowZ', 'rWristX', 'rWristY', 'rWristZ']
    left_joint_names = ['lShoulderX', 'lShoulderZ', 'lShoulderY', 'lElbowZ', 'lWristX', 'lWristY', 'lWristZ']
    torso_joint_names = ['PelvisRotX', 'PelvisRotY', 'PelvisRotZ', 'TorsoX', 'TorsoZ', 'TorsoY', 'HeadZ', 'HeadY', 'HeadX']
    right_angles = [[0.1, 0.1, 0, 0.1, 0, 0, 0.1],
                    [0.3, 0.3, 0, 0.3, 0, 0, 0.3],
                    [0.5, 0.5, 0, 0.5, 0, 0, 0.5],
                    [0.7, 0.7, 0, 0.7, 0, 0, 0.7]]
    left_angles = [[0.1, 0.1, 0, 0.1, 0, 0, 0.1],
                   [0.3, 0.3, 0, 0.3, 0, 0, 0.3],
                   [0.5, 0.5, 0, 0.5, 0, 0, 0.5],
                   [0.7, 0.7, 0, 0.7, 0, 0, 0.7]]

    raw_input("Press enter to exit...")

def move_test(env, robot):
    r_manip = robot.SetActiveManipulator('rightarm')
    r_joint_target = [0, 0, 0, 0, 0, 0, 0]
    print r_manip.GetArmIndices()

    print robot
    with env:
        print 'moving'
        robot.SetActiveDOFs(r_manip.GetArmIndices())
        #jointnames = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        #robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues(r_joint_target)
        #robot.GetController().SetDesired(robot.GetDOFValues());
        print 'done moving'
    waitrobot(robot)

def client_test():
    x = 8
    print test_client(x)

def quat_mult_test():
    axis = numpy.array([0, 0, 1])
    angle = numpy.pi * 0.25
    quat = transformations.quaternion_about_axis(angle, axis)

    axis2 = numpy.array([0, 0, 1])
    angle2 = numpy.pi * 0.25
    quat2 = transformations.quaternion_about_axis(angle2, axis2)

    quat3 = transformations.quaternion_multiply(quat, quat2)
    
    tup = transformations.quaternion_to_axis(quat3)
    print tup
    print tup[0]

    t_mat_rot = transformations.quaternion_matrix(quat)
    print t_mat_rot
    t_mat_trans = transformations.translation_matrix(numpy.array([0.70710678,0.70710678,0]))
    t_mat_trans = transformations.translation_matrix(numpy.array([0,1,0]))
    print t_mat_trans
    t_mat = numpy.dot(t_mat_rot,t_mat_trans)
    print t_mat

def quaternion_fromto_test(tb):
    q1 = tb.utils.ArrayToQuat([0,0,0,1])
    q2 = tb.utils.ArrayToQuat([0,0,0,1])
    q3 = tb.utils.QuaternionFromTo(q1,q2)

    q4 = tb.utils.MultiplyQuaternion(q1,q3)

    print q3
    print q4

    print transformations.quaternion_multiply(tb.utils.QuatToArray(q2),tb.utils.QuatToArray(q3))

    raw_input("Press enter to cont...")

def calc_slope_test(tb):
    p1 = [0,0,0]
    p2 = [1,1,1]
    p3 = [2,2,2]

    print tb.CalcSlope(p1,p2,p3)

def eval_slope_test(tb):
    p1 = [0,0,0]
    p2 = [1,1,1]
    p3 = [2,2,2]

    print tb.EvalSlopeDir(p1,p2,p3)

def robot_state_test(robot):
    manip = robot.SetActiveManipulator('rightarm')
    ee_trans = manip.GetEndEffector().GetTransform()
    print ee_trans
    raw_input("Press enter to continue...")

    #with RobotStateSaver(robot):
    r_state = RobotStateSaver(robot)
    with r_state:
        tuckarms(env, robot)
        ee_trans = manip.GetEndEffector().GetTransform()
        print ee_trans
        print robot.GetActiveDOFValues()
        raw_input("Press enter to continue...")

    r_state.Restore()

    print robot.GetActiveDOFValues()
    with env:
        robot.SetActiveDOFValues(robot.GetActiveDOFValues())

    Tgoal = numpy.array([[1,0,0,0.325],[0,1,0,-0.65],[0,0,1,0.725],[0,0,0,1]])

def grab_test(tb, robot, env, human, goggles, forehead):

    h = human.GetTransform()
    h[0,3] -= 0.5
    t = goggles.GetTransform()
    t[0,3] -= 0.5
    f = forehead.GetTransform()
    f[0,3] -= 0.5

    with env:
        human.SetTransform(h)
        goggles.SetTransform(t)
        forehead.SetTransform(f)

    t = goggles.GetTransform()
    t[0,3] -= 0.1
    handles.append(env.plot3(points=t[0:3,3],
                           pointsize=5.0,
                           colors=array(((0,0,1)))))
    rot_mat  = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                         [np.sin(np.pi),  np.cos(np.pi), 0],
                         [0, 0, 1]])
    t[0:3, 0:3] = np.dot(t[0:3, 0:3], rot_mat)
    direction = [0, 1, 0]
    angle = np.pi*0.5
    rot_mat = transformations.rotation_matrix(angle, direction)
    t[0:3, 0:3] = np.dot(rot_mat[0:3, 0:3], t[0:3, 0:3])
    ikSolutions = ku.ik_for_link(t, tb.r_manip, tb.r_manip.GetEndEffector().GetName(), filter_options = openravepy.IkFilterOptions.CheckEnvCollisions, return_all_solns = True)
    
    from_vec = t[0:3,3]

    to_vec_1 = from_vec + 0.05*(t[0:3,0])
    to_vec_2 = from_vec + 0.05*(t[0:3,1])
    to_vec_3 = from_vec + 0.05*(t[0:3,2])

    handles.append(env.drawarrow(from_vec, to_vec_1, 0.005, [1, 0, 0]))
    handles.append(env.drawarrow(from_vec, to_vec_2, 0.005, [0, 1, 0]))
    handles.append(env.drawarrow(from_vec, to_vec_3, 0.005, [0, 0, 1]))

    raw_input("Press enter to move to ready...")

    print len(ikSolutions)

    with env:
        robot.SetActiveManipulator(tb.r_manip)
        robot.SetActiveDOFs(tb.r_manip.GetArmIndices())
        robot.SetActiveDOFValues(ikSolutions[0])


    raw_input("Press enter to grab...")

    with env:
        robot.Grab(env.GetKinBody('goggles'))
    
    # # tb.TestInterp()
    raw_input("Press enter to continue...")

    tuckarms(env, robot)

    raw_input("Press enter to continue...")

def modify_goggles_x_extents(goggles, x_len):
    #extents arent modifiable
    goggle_links = goggles.GetLinks()

    idx = 0
    for g_link in goggle_links:
        print g_link.GetGeometries()[0].GetBoxExtents()
        if (idx == 1 or idx == 2):
            #this wont work
            g_link.GetGeometries()[0].GetBoxExtents[0] = x_len
            print g_link.GetGeometries()[0].GetBoxExtents()
        idx += 1

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
    goggles  = env.GetRobots()[2]
    forehead = env.GetKinBody('forehead')


    # generate_ik_solver('/home/anto/Desktop/baxter_custom_ikfast/baxter_left_arm.robot.xml', 
    #                    '/home/anto/Desktop/baxter_custom_ikfast/baxter_ik_left.cpp', )
    # generate_ik_solver('/home/anto/Desktop/baxter_custom_ikfast/baxter_right_arm.robot.xml', 
                       # '/home/anto/Desktop/baxter_custom_ikfast/baxter_ik_right.cpp', )
    # generate_ik_solver('/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/baxter_left_arm.robot.xml', 
    #                    '/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/baxter_ik_left_5.cpp', )
    # generate_ik_solver('/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/baxter_right_arm.robot.xml', 
    #                    '/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/baxter_ik_right_5.cpp', )
    # generate_ik_solver('baxter_right_arm.robot.xml', 'baxter_ik_right_2.cpp', )

    tb = TaskBreakdown()
    gr = GogglesRobot()
    gr.Init(env, goggles)
    handles = []
    tb.Init(robot, env, human, goggles, gr, forehead, handles)

    tuckarms(env, robot)

    # # print goggles.GetTransform()

    # # sol = tb.GetIKSolutions(Tgoal, manip)

    ppe_loc = []
    ppe_trans = []

    move_human_to_ready(tb, handles, env, goggles, ppe_loc, gr)
    tb.CacheBodyTransforms()

    test_motion(tb, handles, env, goggles, ppe_loc, ppe_trans, gr)

    tb.ObtainClosestBodyTransform(ppe_loc)
    tb.PurifyIntoTransfer(ppe_trans, env)
    tb.PurifyIntoSupport(ppe_trans, env)
    # tb.InterpolateTrajectories()

    # # tb.RelevantPathWaypointsTest()
    # raw_input("Press enter to continue...")

    tb.FindBestDisplacement()
    raw_input("Press enter to generate reaching motions for right arm...")

    # tb.GenerateReachingForTransfer(tb.r_manip)
    # raw_input("Press enter to trajopt transfer path...")

    tb.TrajoptTransferPath()
    raw_input("Press enter to exit...")