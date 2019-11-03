#!/usr/bin/env python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Baxter RSDK Joint Trajectory Example: file playback
"""

import argparse
import operator
import sys

from bisect import bisect
from copy import copy
from os import path

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self):
        #create our action server clients
        self._left_client = actionlib.SimpleActionClient(
            'robot/limb/left/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._right_client = actionlib.SimpleActionClient(
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        #verify joint trajectory action servers are available
        l_server_up = self._left_client.wait_for_server(rospy.Duration(1.0))
        r_server_up = self._right_client.wait_for_server(rospy.Duration(1.0))
        if not l_server_up or not r_server_up:
            msg = ("Action server not available."
                   " Verify action server availability.")
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        #create our goal request
        self._l_goal = FollowJointTrajectoryGoal()
        self._r_goal = FollowJointTrajectoryGoal()

        #limb interface - current angles needed for start move
        self._l_arm = baxter_interface.Limb('left')
        self._r_arm = baxter_interface.Limb('right')

        #gripper interface - for gripper command playback
        self._l_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self._r_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        # Verify Grippers Have No Errors and are Calibrated
        if self._l_gripper.error():
            self._l_gripper.reset()
        if self._r_gripper.error():
            self._r_gripper.reset()
        if (not self._l_gripper.calibrated() and
            self._l_gripper.type() != 'custom'):
            self._l_gripper.calibrate()
        if (not self._r_gripper.calibrated() and
            self._r_gripper.type() != 'custom'):
            self._r_gripper.calibrate()

        #gripper goal trajectories
        self._l_grip = FollowJointTrajectoryGoal()
        self._r_grip = FollowJointTrajectoryGoal()

        #param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        #gripper control rate
        self._gripper_rate = 20.0  # Hz

        #scale and min for trajectory execution time
        self.exec_time_scaling = 0.7
        self.min_joint_exec_time = 0.2

        #figure out trajectory execution time
        # left_exec_time = self.exec_time_scaling * self.weighted_euclidean_distance(working_left_arm_config, left_target_config)
        # if left_exec_time < self.min_joint_exec_time:
        #     left_exec_time = self.min_joint_exec_time
        # right_exec_time = self.exec_time_scaling * self.weighted_euclidean_distance(working_right_arm_config, right_target_config)
        # if right_exec_time < self.min_joint_exec_time:
        #     right_exec_time = self.min_joint_exec_time

        self.left_joint_names = ["left_s0","left_s1","left_e0","left_e1","left_w0","left_w1","left_w2","left_gripper"]
        self.right_joint_names = ["right_s0",'right_s1',"right_e0","right_e1","right_w0","right_w1","right_w2","right_gripper"]

        self.joint_names = ["left_s0","left_s1","left_e0","left_e1","left_w0","left_w1","left_w2","left_gripper","right_s0",'right_s1',"right_e0","right_e1","right_w0","right_w1","right_w2","right_gripper"]


    def _execute_gripper_commands(self):
        start_time = rospy.get_time()
        r_cmd = self._r_grip.trajectory.points
        l_cmd = self._l_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        now_from_start = rospy.get_time() - start_time
        while(now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown()):
            idx = bisect(pnt_times, now_from_start) - 1
            if self._r_gripper.type() != 'custom':
                self._r_gripper.command_position(r_cmd[idx].positions[0])
            if self._l_gripper.type() != 'custom':
                self._l_gripper.command_position(l_cmd[idx].positions[0])
            rate.sleep()
            now_from_start = rospy.get_time() - start_time

    def _add_point(self, positions, side, time):
        #creates a point in trajectory with time_from_start and positions
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'left':
            self._l_goal.trajectory.points.append(point)
        elif side == 'right':
            self._r_goal.trajectory.points.append(point)
        elif side == 'left_gripper':
            self._l_grip.trajectory.points.append(point)
        elif side == 'right_gripper':
            self._r_grip.trajectory.points.append(point)

    def generate_executable_trajectory(self, r_traj, l_traj):
        for i in range(0, len(self.left_joint_names)-1):
            self._l_goal.trajectory.joint_names.append(self.left_joint_names[i])
            self._r_goal.trajectory.joint_names.append(self.right_joint_names[i])

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in self.joint_names:
                if 'left' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._l_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
                elif 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            #determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        start_offset = 0
        for i in range(0, len(r_traj)): #r_traj and l_traj must have the same length
            if i == 0:
                cmd = {}
                for j in range(0, len(self.left_joint_names) - 1):
                    cmd[self.left_joint_names[j]] = l_traj[i][j]
                for j in range(0, len(self.right_joint_names) - 1):
                    cmd[self.right_joint_names[j]] = r_traj[i][j]
                start_offset = find_start_offset(cmd)
            #add a point for this set of commands with recorded time
            cur_r_traj = []
            cur_l_traj = []
            for j in range(1, len(l_traj[i])-1):
                cur_r_traj.append(r_traj[i][j])
                cur_l_traj.append(l_traj[i][j])

            self._add_point(cur_l_traj, 'left', l_traj[i][0]+ start_offset)
            self._add_point(cur_r_traj, 'right', r_traj[i][0] + start_offset)
            self._add_point([l_traj[i][8]], 'left_gripper', l_traj[i][0] + start_offset)
            self._add_point([r_traj[i][8]], 'right_gripper', r_traj[i][0] + start_offset)

        raw_input("press enter to continue")

    def start(self):
        # print 'l_goal', self._l_goal
        # print '_r_goal', self._r_goal

        self._left_client.send_goal(self._l_goal)
        self._right_client.send_goal(self._r_goal)
        self._execute_gripper_commands()

    def stop(self):
        if (self._left_client.gh is not None and
            self._left_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._left_client.cancel_goal()

        if (self._right_client.gh is not None and
            self._right_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._right_client.cancel_goal()

        #delay to allow for terminating handshake
        rospy.sleep(0.1)

    def wait(self):
        #create a timeout for our trajectory execution
        #total time trajectory expected for trajectory execution plus a buffer
        last_time = self._r_goal.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(last_time + time_buffer)

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)
        l_result = (self._left_client.get_result().error_code == 0)
        r_result = (self._right_client.get_result().error_code == 0)

        #verify result
        if all([l_finish, r_finish, l_result, r_result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False

def execute_traj(r_manip_traj, l_manip_traj):
    print("Initializing node... ")
    rospy.init_node("baxter_openrave_trajectory_execution")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = Trajectory()
    #for safe interrupt handling

    traj.generate_executable_trajectory(r_manip_traj, l_manip_traj)
    rospy.on_shutdown(traj.stop)
    result = True
    loop_cnt = 1
    loopstr = "1"
    while (result == True and loop_cnt <= 1
           and not rospy.is_shutdown()):
        print("Playback loop %d of %s" % (loop_cnt, loopstr,))
        traj.start()
        result = traj.wait()
        loop_cnt = loop_cnt + 1
    print("Exiting - Trajectory Execution Complete")