#!/usr/bin/env python
# Jim Mainprice, ARC
# September 2013
# Worcester Polytechnic Institute
#
# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
from openravepy.misc import OpenRAVEGlobalArguments
from random import *

## SYSTEM - FILE OPS ##
import sys
import os
from datetime import datetime
import time
import shutil

class Pr2TrajectoryReader():

     #--------------------------------------------------------------------------
     # Loads the PR2 and wheel model
     #--------------------------------------------------------------------------
    def __init__(self):

        current_path = os.path.dirname(os.path.realpath(__file__)) + "/"
        print current_path
        environment = current_path + '../models/heres_how_env_empty.xml'
        # environment = 'robots/pr2-beta-static.zae'

        self.env = Environment()
        #self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.SetDebugLevel(DebugLevel.Info)  # set output level to debug
        self.env.Load(environment)
        self.robot = self.env.GetRobots()[0]

        self.ViewerStarted = False

        self.right_arm_idx = 5
        self.left_arm_idx = 7

        self.r_arm_indices = self.robot.GetManipulators()[self.right_arm_idx].GetArmIndices()
        self.l_arm_indices = self.robot.GetManipulators()[self.left_arm_idx].GetArmIndices()

        print self.r_arm_indices
        print self.l_arm_indices

        self.arm_indices = concatenate((self.r_arm_indices, self.l_arm_indices), axis=0)

        self.suplementary_dofs = []
        # self.suplementary_dofs.append( self.jointDict["torso_lift_joint"] )

        self.manip_indices = concatenate((self.arm_indices, array(self.suplementary_dofs)), axis=0)

        print self.manip_indices

        self.robot.SetActiveDOFs(self.manip_indices)

        self.plan_all_DOF_ik = False

        self.alldofs = self.manip_indices

        self.dt = .1
        self.execute_in_loop = True

        self.drawingHandles = None

        return

    def LaunchViewer(self):

        # Start the Viewer and draws the world frame
        if not self.ViewerStarted:

            self.env.SetViewer('qtcoin')

            T_cam = ( [[ 0.79892075,  0.38440252, -0.46255846,  3.10823035], \
                       [ 0.59796664, -0.42518478,  0.6794511,  -4.20137262], \
                       [ 0.0645099,  -0.81942212, -0.56954883,  3.56726289], \
                       [ 0.,          0.,          0.,          1.,        ]])

            # self.env.GetViewer().GetCameraTransform()
            # self.env.GetViewer().SetCamera(T_cam)
            self.env.GetViewer().EnvironmentSync()

    # return [left, right]
    # left  : [ [q0, 0.0], [q1, t1] , ... , [qN, tN] ]
    # right : [ [q0, 0.0], [q1, t1] , ... , [qN, tN] ]
    def load_traj(self, filename):

        traj = RaveCreateTrajectory(self.robot.GetEnv(), '')
        traj.Init(self.robot.GetActiveConfigurationSpecification())
        f = open(filename, 'r')
        traj.deserialize(f.read())
        f.close()

        print "traj.GetDuration() : ", traj.GetDuration()
        left = []
        right = []
        scaling = 5
        for t in linspace(0.0, traj.GetDuration(), 60):
            q = traj.Sample(t)
            left.append([q[self.l_arm_indices].tolist(), t*scaling])
            right.append([q[self.r_arm_indices].tolist(), t*scaling])

        return [left, right]

    def execute_left_right(self, filename):

        [left, right] = self.load_traj(filename)

        while True:
            for i in range(len(left)):
                self.robot.SetDOFValues(left[i][0], self.l_arm_indices)
                self.robot.SetDOFValues(right[i][0], self.r_arm_indices)
                if i > 0:
                    time.sleep(left[i][1]-left[i-1][1])
                # print "set config ", i

    # Plays the trajectory in openrave
    def execute(self, filename):

        traj = RaveCreateTrajectory(self.robot.GetEnv(), '')
        traj.Init(self.robot.GetActiveConfigurationSpecification())

        f = open(filename, 'r')
        traj.deserialize(f.read())
        f.close()

        print "Play Back Trajectory!!!!"

        if self.drawingHandles is not None:
            del self.drawingHandles[:]

        print self.robot.GetActiveConfigurationSpecification()
        print traj.GetConfigurationSpecification()

        data = traj.GetWaypoint(0)
        q = traj.GetConfigurationSpecification().ExtractJointValues(data, self.robot, self.robot.GetActiveDOFIndices())
        self.robot.SetActiveDOFValues(q)
        print "Press return to exit."
        sys.stdin.readline()

        data = traj.GetWaypoint(traj.GetNumWaypoints()-1)
        q = traj.GetConfigurationSpecification().ExtractJointValues(data, self.robot, self.robot.GetActiveDOFIndices())
        self.robot.SetActiveDOFValues(q)
        print "Press return to exit."
        sys.stdin.readline()

        # Draw trace
        newrobots = []
        for t in linspace(0, traj.GetDuration(), 10):
            newrobots.append(RaveCreateRobot(self.env, self.robot.GetXMLId()))
            newrobots[-1].Clone(self.robot, 0)
            for link in newrobots[-1].GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(0.3)
            self.env.Add(newrobots[-1], True)
            q = traj.Sample(t)  # get configuration
            newrobots[-1].SetDOFValues(q[0:newrobots[-1].GetDOF()])
            self.robot.SetDOFValues(q[0:newrobots[-1].GetDOF()])

        print "Press return to exit."
        sys.stdin.readline()

        for r in newrobots:
            self.env.Remove(r)

        while True:

            for i in range(traj.GetNumWaypoints()):
                # get the waypoint values, this holds velocites, time stamps, etc
                data = traj.GetWaypoint(i)
                # extract the robot joint values only
                with self.env: # have to lock environment since accessing robot
                    q = traj.GetConfigurationSpecification().ExtractJointValues(data, self.robot, self.robot.GetActiveDOFIndices())
                    self.robot.SetActiveDOFValues(q)
                time.sleep(self.dt)

            if not self.execute_in_loop:
                break
        return


def main():

    filename = None

    if len(sys.argv) >= 2:
        for index in range(1, len(sys.argv)):
            if sys.argv[index] == "-f" and index+1 < len(sys.argv):
                filename = str(sys.argv[index+1])

    if filename is None:
        return

    player = Pr2TrajectoryReader()
    player.LaunchViewer()

    player.execute_left_right(filename)
    player.KillOpenrave()

    return

if __name__ == "__main__":
    main()
