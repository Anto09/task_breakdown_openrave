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
    env.Load("/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/kinbodies_robots_envs/apron_robot_env.env.xml")
    time.sleep(0.1)

    InitOpenRAVELogging() 
    
    robot = env.GetRobots()[0]
    raw_input("Press enter to exit...")