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

def move_robot(robot):
    dof_val = 0.0
    for i in range(0,20):
        dof_val += 0.1
        with env:
            robot.SetActiveDOFValues([dof_val])
        raw_input("Press enter to exit...")

if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    env.Load("/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/strap_test_env.env.xml")
    time.sleep(0.1)

    InitOpenRAVELogging() 
    
    robot = env.GetRobots()[0]
    gr = GogglesRobot()
    gr.Init(env, robot)
    gr.CheckCollisionTest()

    # with env:
    #     robot.SetActiveDOFs([0])

    # print robot.GetActiveDOFValues()

    # with env:
    #     report = CollisionReport()
    #     inlier1 = env.CheckCollision(robot,report)
    #     print 'mindist: ',report.minDistance
    #     contacts = report.contacts
    #     handles = [env.drawlinestrip(points=array((c.pos,c.pos-c.depth*c.norm)), linewidth=3.0, colors=array((1,0,0,1))) for c in contacts]
        
    #     print 'collision',env.CheckCollision(robot,report)

    #     print 'plink1',report.plink1
    #     print 'plink2',report.plink2
    #     print 'numCols',report.numCols

    #     print 'numWithinTol',report.numWithinTol

    #     print 'num contacts',len(contacts)
    #     print 'contacts'
    #     for c in contacts: 
    #         print c

    #     for link in robot.GetLinks():
    #         print link

    #     backstrap = robot.GetLink('backstrap')
    #     report1 = CollisionReport()
    #     inlier2 = env.CheckCollision(backstrap, report1)
    #     print 'mindist: ',report1.minDistance
    #     contacts = report1.contacts
    #     handles = [env.drawlinestrip(points=array((c.pos,c.pos-c.depth*c.norm)), linewidth=3.0, colors=array((1,0,0,1))) for c in contacts]
        
    #     print 'collision',env.CheckCollision(backstrap,report1)

    #     print 'plink1',report1.plink1
    #     print 'plink2',report1.plink2
    #     print 'collision data plink1',report1.plink1.GetCollisionData()
    #     for vertex in report1.plink1.GetCollisionData().vertices:
    #         print vertex
    #         handles.append(env.plot3(vertex,
    #                        pointsize=5.0,
    #                        colors=array(((0.5,0.5,0)))))

    #     print 'plink geometry'
    #     for geom in report1.plink2.GetGeometries():
    #         print geom
    #         gtype =  geom.GetType()
    #         print gtype
    #         print geom.IsModifiable()

    #         if (gtype == 'Sphere'):
    #             print geom.GetSphereRadius()
    #         elif (gtype == 'Box'):
    #             print geom.GetBoxExtents()
    #         elif (gtype == 'Cylinder'):
    #             print geom.GetCylinderRadius()
    #             print geom.GetCylinderHeight()

    #     print 'numCols',report1.numCols

    #     print 'numWithinTol',report1.numWithinTol

    #     print 'num contacts',len(contacts)
    #     print 'contacts'
    #     for c in contacts: 
    #         print c

    #     print backstrap.GetTransform()[0:3,3] - backstrap.GetTransform()[0:3,3]
    #     time.sleep(0.1)


    # move_robot(robot)

    raw_input("Press enter to exit...")