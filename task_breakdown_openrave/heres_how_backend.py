#!/usr/bin/env python
# Ben Suay, RAIL
# Jim Mainprice, ARC
# July 2013, October 2013
# Worcester Polytechnic Institute

import rospy
import actionlib
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from heres_how_play_trajs import *
import lane_builder


class PR2Execution():

    def __init__(self):
        # Make the node
        # rospy.init_node("pr2_execution")
        # Make an instance of the Here's How reader class
        # self.reader = Pr2TrajectoryReader()
        # Params
        self.exec_time_scaling = 0.7
        self.min_joint_exec_time = 0.2
        # Load arm actionlib clients
        self.left_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.left_arm_client.wait_for_server()
        self.right_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.right_arm_client.wait_for_server()
        print "Arm actionlib clients ready"
        # Load the arm config subscribers
        self.latest_left_arm_config = None
        self.left_arm_config_sub = rospy.Subscriber("l_arm_controller/state", JointTrajectoryControllerState, self.left_arm_config_cb)
        self.latest_right_arm_config = None
        self.right_arm_config_sub = rospy.Subscriber("r_arm_controller/state", JointTrajectoryControllerState, self.right_arm_config_cb)
        print "Arm config subs loaded"
        print "Waiting for arm configs..."
        while (self.latest_left_arm_config is None) or (self.latest_right_arm_config is None):
            pass
        print "Got arm configs"

    def left_arm_config_cb(self, msg):
        new_config = []
        new_config.append(msg.actual.positions[msg.joint_names.index("l_shoulder_pan_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("l_shoulder_lift_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("l_upper_arm_roll_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("l_elbow_flex_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("l_forearm_roll_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("l_wrist_flex_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("l_wrist_roll_joint")])
        self.latest_left_arm_config = new_config

    def right_arm_config_cb(self, msg):
        new_config = []
        new_config.append(msg.actual.positions[msg.joint_names.index("r_shoulder_pan_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("r_shoulder_lift_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("r_upper_arm_roll_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("r_elbow_flex_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("r_forearm_roll_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("r_wrist_flex_joint")])
        new_config.append(msg.actual.positions[msg.joint_names.index("r_wrist_roll_joint")])
        self.latest_right_arm_config = new_config

    def weighted_euclidean_distance(self, state1, state2, weights=None, dimensions=None):
        if weights is None:
            assert(len(state1) == len(state2))
            if dimensions is None:
                total = 0.0
                for index in range(len(state1)):
                    temp = (state1[index] - state2[index]) ** 2
                    total += temp
                return math.sqrt(total)
            else:
                assert(dimensions <= len(state1))
                total = 0.0
                for index in range(dimensions):
                    temp = (state1[index] - state2[index]) ** 2
                    total += temp
                return math.sqrt(total)
        else:
            assert(len(state1) == len(state2) == len(weights))
            if dimensions is None:
                total = 0.0
                for index in range(len(state1)):
                    temp = ((state1[index] - state2[index]) ** 2) * weights[index]
                    total += temp
                return math.sqrt(total)
            else:
                assert(dimensions <= len(state1))
                total = 0.0
                for index in range(dimensions):
                    temp = ((state1[index] - state2[index]) ** 2) * weights[index]
                    total += temp
                return math.sqrt(total)

    def execute_to_joint_target(self, left_target_config, right_target_config):
        # First, get the current arm configs
        working_left_arm_config = self.latest_left_arm_config
        working_right_arm_config = self.latest_right_arm_config
        # Second, figure out how long to execute the trajectories for
        left_exec_time = self.exec_time_scaling * self.weighted_euclidean_distance(working_left_arm_config, left_target_config)
        if left_exec_time < self.min_joint_exec_time:
            left_exec_time = self.min_joint_exec_time
        right_exec_time = self.exec_time_scaling * self.weighted_euclidean_distance(working_right_arm_config, right_target_config)
        if right_exec_time < self.min_joint_exec_time:
            right_exec_time = self.min_joint_exec_time
        # Third, assemble trajectory points for the targets
        left_target_point = JointTrajectoryPoint()
        left_target_point.positions = left_target_config
        left_target_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        left_target_point.time_from_start = rospy.Duration(left_exec_time)
        right_target_point = JointTrajectoryPoint()
        right_target_point.positions = right_target_config
        right_target_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        right_target_point.time_from_start = rospy.Duration(right_exec_time)
        # Fourth, assemble trajectories
        left_traj = JointTrajectory()
        left_traj.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_traj.points = [left_target_point]
        right_traj = JointTrajectory()
        right_traj.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_traj.points = [right_target_point]
        # Fifth, assemble trajectory goals
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = left_traj
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = right_traj
        # Sixth, execute
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        # self.left_arm_client.wait_for_result()
        # self.right_arm_client.wait_for_result()

    def execute_joint_trajectories(self, left_trajectory, right_trajectory):
        # First, assemble joint trajectory points for both arms
        left_points = []
        for [q, t] in left_trajectory:
            left_target_point = JointTrajectoryPoint()
            left_target_point.positions = q
            left_target_point.velocities = []
            left_target_point.time_from_start = rospy.Duration(t)
            left_points.append(left_target_point)
        right_points = []
        for [q, t] in right_trajectory:
            right_target_point = JointTrajectoryPoint()
            right_target_point.positions = q
            right_target_point.velocities = []
            right_target_point.time_from_start = rospy.Duration(t)
            right_points.append(right_target_point)
        # Second, assemble trajectories
        left_traj = JointTrajectory()
        left_traj.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_traj.points = left_points
        right_traj = JointTrajectory()
        right_traj.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_traj.points = right_points
        # Third, assemble trajectory goals
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = left_traj
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = right_traj
        # Fourth, execute
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        # self.left_arm_client.wait_for_result()
        # self.right_arm_client.wait_for_result()


if __name__ == '__main__':
    execution = PR2Execution()
    execution.execute()


