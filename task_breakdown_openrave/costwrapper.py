#!/usr/bin/env python

import argparse

import sys
import os
import math
import copy
import json

import rospy
from baxter_moveit_test.srv import *

import time
import threading
import openravepy
import trajoptpy
import trajoptpy.kin_utils as ku

#### YOUR IMPORTS GO HERE ####
import sys
#### END OF YOUR IMPORTS ####

from openravepy import ikfast

if not __openravepy_build_doc__:
	from openravepy import *
	from numpy import *

from openravepy.misc import InitOpenRAVELogging

from geometry_msgs.msg import *
import transformations
import numpy as np
from human_trajopt import HumanTrajopt

class CostWrapper:

	robot 			= None
	human 			= None
	goggles 		= None
	env 			= None
	r_manip 		= None
	l_manip 		= None
	a_manip			= None
	utils 			= None
	goggles_robot	= None

	start_t_target 	= []
	end_t_target 	= []
	pose_targets	= []
	env_locked		= False

	joint_vel_coeff = 250
	
	handles = []

	grid = []
	k =[]

	wp_idx = -1

	def Init(self, robot, env, human, goggles, utils, r_manip, l_manip):
		self.robot 		= robot
		self.env 		= env
		self.human 		= human
		self.goggles 	= goggles
		self.utils 		= utils
		self.r_manip 	= r_manip
		self.l_manip 	= l_manip

		k = np.ndarray(shape = (3,3), dtype = float, order = 'F')
		k[0] = [1,1,1]
		k[1] = [1,1,1]
		k[2] = [1,1,1]

	def JointVelCost(self, x):
		print 'calling joint vel cost'
		cost = 0
		dof_count = len(self.a_manip.GetArmIndices())
		x_new = np.zeros(shape = (x.shape[0]/dof_count, dof_count))

		idx = 0
		for i in range(0, x.shape[0]):
			x_new[i,:] = np.copy(x[idx:idx+dof_count])
			idx += dof_count
			if (idx >= x.shape[0]):
				break
		diffAxis0 = np.copy(x_new[0:x_new.shape[0]-1, :])
		diffAxis1 = np.copy(x_new[1:x_new.shape[0], :])
		diffAxis = diffAxis1 - diffAxis0

		diag_mat = np.eye(x_new.shape[1]).dot(self.joint_vel_coeff)

		# print 'calculated joint vel cost is', (np.square(diffAxis).dot(diag_mat)).sum()
		return (np.square(diffAxis).dot(diag_mat)).sum()
		# test for maximizing joint velocity
		# print 'calculated joint vel cost is', (10.0 * self.joint_vel_coeff)/((np.square(diffAxis).dot(diag_mat)).sum())
		# return (10.0 * self.joint_vel_coeff)/((np.square(diffAxis).dot(diag_mat)).sum())

	def ManipulabilityCost(self, x):
		jac = self.a_manip.CalculateJacobian()
		return np.sqrt(np.linalg.det(jac.dot(jac.transpose())))



class CostObject:
	pose_target  	   = []
	constraint_target  = []
	current_pose 	   = []

	env 	 	  = None
	robot 	 	  = None
	utils 	 	  = None
	a_manip	 	  = None
	r_manip	 	  = None
	l_manip	 	  = None
	goggles  	  = None
	g_pose   	  = None
	g_quat 	 	  = None
	gradient 	  = None
	goggles_robot = None

	task_dev_coeff 	= 1000
	cart_dev_frac 	= 0.75
	orient_dev_frac = 0.25

	parent_node = None
	child_node 	= None

	handles = []

	human_trajopt = None
	time_step = 0
	total_steps = 0

	# variables for partial trajectory evaluation
	start_idx 	= -1
	end_idx 	= -1
	eval_idx 	= -1
	vgr_q = 9999999.99
	vgr_s = 9999999.99
	vgr_q_task = 9999999.99
	vgr_s_task = 9999999.99

	def Init(self, pose_target, constraint_target, env, robot, utils, a_manip, goggles, goggles_robot, parent = None, child = None, r_manip = None, l_manip = None):
		self.pose_target 		= pose_target
		self.current_pose		= pose_target
		self.constraint_target	= constraint_target
		self.env 				= env
		self.robot 				= robot
		self.utils 				= utils
		self.a_manip 			= a_manip
		self.parent_node		= parent
		self.child_node			= child
		self.r_manip			= r_manip
		self.l_manip			= l_manip

		if (goggles != None):
			self.goggles 			= goggles
			self.g_pose 			= self.goggles.GetTransform()
			self.g_quat 			= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(self.g_pose[0:3, 0:3]))
			self.goggles_robot		= goggles_robot

	def CartDeviationCost(self, x, two_arms = False):
		r_state = self.robot.CreateRobotStateSaver()

		if (two_arms):
			tool_pose_right = []
			tool_pose_left  = []

			ee_transform_r 	= self.r_manip.GetEndEffector().GetTransform()
			ee_quat_r 		= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(ee_transform_r[0:3, 0:3]))
			with r_state:
				#right
				self.robot.SetActiveDOFs(self.r_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x[0:8])
				tool_pose_right = self.r_manip.GetEndEffector().GetTransform() 

				#left
				self.robot.SetActiveDOFs(self.l_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x[8:14])
				tool_pose_right = self.r_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()

			return 0

		else:
			tool_pose = []

			ee_transform 	= self.a_manip.GetEndEffector().GetTransform()
			ee_quat 		= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(ee_transform[0:3, 0:3]))
			with r_state:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x)
				tool_pose = self.a_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()

			pos_diff 	= self.utils.SubVec(self.pose_target[0:3,3], tool_pose[0:3,3])
			tool_quat	= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(tool_pose[0:3, 0:3]))
	 
			cart_dev = np.linalg.norm(pos_diff)

			return cart_dev

	def TaskDeviationCost(self, x, two_arms = False):
		r_state = self.robot.CreateRobotStateSaver()

		if (two_arms):
			tool_pose_right = self.r_manip.GetEndEffector().GetTransform()
			tool_pose_left  = self.l_manip.GetEndEffector().GetTransform()

			return 0
		else:
			t0_ee 	= self.a_manip.GetEndEffector().GetTransform()

			with r_state:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x)
				t0_ee = self.a_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()

			tpose_ee = np.dot(np.linalg.inv(self.pose_target), t0_ee)

			delta = np.array([tpose_ee[0,3],
							  tpose_ee[1,3],
							  tpose_ee[2,3],
							  np.arctan2(tpose_ee[2,1], tpose_ee[2,2]),
							  -np.arcsin(tpose_ee[2,0]),
							  np.arctan2(tpose_ee[1,0], tpose_ee[0,0])])

			delta_pos = delta[0:3]
			delta_rot = delta[3:6]

			# return np.linalg.norm(delta)
			# self.goggles_robot.RayCollisionCheckDiff(new_g_pose)
			return (self.cart_dev_frac*(np.linalg.norm(delta_pos)) + self.orient_dev_frac*(np.linalg.norm(delta_rot))) * self.task_dev_coeff

	def SlopeCostWithMemory(self, x, two_arms = False):
		# start = eval - 1, end = eval + 1
		dof_count = len(self.a_manip.GetArmIndices())
		x_new = np.zeros(shape = (x.shape[0]/dof_count, dof_count))

		idx = 0
		for i in range(0, x.shape[0]):
			x_new[i,:] = np.copy(x[idx:idx+dof_count])
			idx += dof_count
			if (idx >= x.shape[0]):
				break

		r_state 	= self.robot.CreateRobotStateSaver()

		if (two_arms):
			t0_ee_r_prev 	= self.r_manip.GetEndEffector().GetTransform()
			t0_ee_r_curr 	= self.r_manip.GetEndEffector().GetTransform()

			t0_ee_l_prev 	= self.l_manip.GetEndEffector().GetTransform()
			t0_ee_l_curr 	= self.l_manip.GetEndEffector().GetTransform()

			return 0

		else:
			t0_ee_prev 	= self.a_manip.GetEndEffector().GetTransform()
			t0_ee_curr 	= self.a_manip.GetEndEffector().GetTransform()
			# t0_ee_next 	= self.a_manip.GetEndEffector().GetTransform()

			with r_state:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x_new[self.eval_idx, :])
				t0_ee_curr = self.a_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()

			if (self.parent_node != None):
				with r_state:
					self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
					self.robot.SetActiveDOFValues(x_new[self.start_idx, :])
					t0_ee_prev = self.a_manip.GetEndEffector().GetTransform() 
				r_state.Restore()
				r_state.Release()

			# if (self.child_node != None):
			# 	with r_state:
			# 		self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
			# 		self.robot.SetActiveDOFValues(x_new[self.end_idx, :])
			# 		t0_ee_next = self.a_manip.GetEndEffector().GetTransform() 
			# 	r_state.Restore()
			# 	r_state.Release()

			cost_val = 0
			#use dot product to calculate difference
			if (self.parent_node != None):
				prev_orig = np.linalg.norm(self.pose_target[0:3,3] - self.parent_node.pose_target[0:3,3])
				prev_curr = np.linalg.norm(t0_ee_curr[0:3,3] - t0_ee_prev[0:3,3])
				cost_val += (1-np.dot(prev_curr, prev_orig)) #(0-1)

			# if (self.child_node != None):
			# 	next_orig = np.linalg.norm(self.child_node.pose_target[0:3,3] - self.pose_target[0:3,3])
			# 	next_curr = np.linalg.norm(t0_ee_next[0:3,3] - t0_ee_curr[0:3,3])
			# 	cost_val += (np.dot(next_curr, next_orig))

			return cost_val * 50

	def OptimalTrajectory (self, x):
		self.vgr_q = min(self.SumSquaredVel(x, self.time_step, self.total_steps), self.vgr_q)
		self.vgr_s = min(self.SumSquaredVel(x, 0, self.total_steps), self.vgr_s)

	def StartNomalizer(self, x):
		dof_count = len(self.a_manip.GetArmIndices())
		x_new = np.zeros(shape = (x.shape[0]/dof_count, dof_count))

		idx = 0
		for i in range(0, x.shape[0]):
			x_new[i,:] = np.copy(x[idx:idx+dof_count])
			idx += dof_count
			if (idx >= x.shape[0]):
				break

		return np.linalg.norm(x_new[0]) ** 2

	def SumSquaredVel(self, x, start_idx, end_idx):
		dof_count = len(self.a_manip.GetArmIndices())
		x_new = np.zeros(shape = ((x.shape[0]/dof_count), dof_count))

		idx = 0
		for i in range(0, x.shape[0]):
			x_new[i,:] = np.copy(x[idx:idx+dof_count])
			idx += dof_count
			if (idx >= x.shape[0]):
				break

		x_new = x_new[start_idx:end_idx]
		if (end_idx - start_idx == 0):
			return 0

		diffAxis0 = np.copy(x_new[0:x_new.shape[0]-1, :])
		diffAxis1 = np.copy(x_new[1:x_new.shape[0], :])
		diffAxis = diffAxis1 - diffAxis0
		diag_mat = np.eye(x_new.shape[1])

		return (np.square(diffAxis).dot(diag_mat)).sum()

	# can't pass function here (due to TrajOpt format)
	def PredictabilityCost(self, x):
		return np.exp(self.SumSquaredVel(x))

	# can't pass function here (due to TrajOpt format)
	def LegibilityCost(self, x): #assume single hand (what about double)
		# calculate sum squared of velocities
		cost = self.SumSquaredVel(x, 0, self.time_step)
		self.OptimalTrajectory(x)

		# legibility = (1.0/self.total_steps) * np.exp(-cost - self.vgr_q) / np.exp(-self.vgr_s) 
		legibility = np.exp(-cost - self.vgr_q) / np.exp(-self.vgr_s) 
		legibility = legibility * (self.total_steps - self.time_step) / (self.total_steps**2 - self.total_steps)

		print 'sum squared vel cost', cost
		print 'self.vgr_q', self.vgr_q
		print 'self.vgr_s', self.vgr_s
		print 'cost for', self.time_step, 'is', (1.0/legibility)
		return 100.0/legibility 
		# print 'cost for', self.time_step, 'is', (legibility)
		# return legibility 

	def SumSquaredVelTask(self, x, start_idx, end_idx):
		dof_count = len(self.a_manip.GetArmIndices())
		x_new = np.zeros(shape = ((x.shape[0]/dof_count), dof_count))

		idx = 0
		for i in range(0, x.shape[0]):
			x_new[i,:] = np.copy(x[idx:idx+dof_count])
			idx += dof_count
			if (idx >= x.shape[0]):
				break

		x_new = x_new[start_idx:end_idx]
		if (end_idx - start_idx == 0):
			return 0

		idx = 0
		t_new = np.zeros(shape = ((x.shape[0]/dof_count), 3))
		for config in x_new:
			t = []
			r_state = self.robot.CreateRobotStateSaver()
			with r_state:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(config)
				t = self.a_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()
			t_new[idx:,] = t[0:3,3]
			idx += 1

		diffAxis0 = np.copy(t_new[0:t_new.shape[0]-1, :])
		diffAxis1 = np.copy(t_new[1:t_new.shape[0], :])
		diffAxis = diffAxis1 - diffAxis0
		diag_mat = np.eye(t_new.shape[1])

		return (np.square(diffAxis).dot(diag_mat)).sum()

	def OptimalTrajectoryTask(self, x):
		self.vgr_q_task = min(self.SumSquaredVelTask(x, self.time_step, self.total_steps), self.vgr_q_task)
		self.vgr_s_task = min(self.SumSquaredVelTask(x, 0, self.total_steps), self.vgr_s_task)

	def LegibilityCostTask(self, x):
		cost = self.SumSquaredVelTask(x, 0, self.time_step)
		self.OptimalTrajectoryTask(x)

		legibility = np.exp(-cost - self.vgr_q_task) / np.exp(-self.vgr_s_task) 
		legibility = legibility * (self.total_steps - self.time_step) / (self.total_steps**2 - self.total_steps)

		print 'cost for', self.time_step, 'is', (1.0/legibility)
		return 100.0/legibility 
