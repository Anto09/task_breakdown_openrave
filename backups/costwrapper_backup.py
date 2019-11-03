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

	joint_vel_coeff = 200
	
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

	def CartesianDeviationCost(self, x):
		self.wp_idx += 1
		self.wp_idx = self.wp_idx % 23
		r_state = RobotStateSaver(self.robot)
		tool_pose = []
		with r_state:
			self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
			self.robot.SetActiveDOFValues(x)
			tool_pose = self.a_manip.GetEndEffector().GetTransform() #do forward kinematics
		r_state.Restore()
		r_state.Release()

		return np.linalg.norm(self.utils.SubVec(waypoint[0:3,3], tool_pose[0:3,3]))

	def RotationDeviationCost(x, n, waypoint, manip):
		tool_pose = manip.GetEndEffector().GetTransform()

		way_point_quat 	= transformations.quaternion_from_matrix(waypoint[0:3, 0:3])
		quat 			= transformations.quaternion_from_matrix(tool_pose[0:3, 0:3])
		adjustment_quat = self.utils.QuaternionFromTo(way_point_quat, quat)
		axis_diff_mag 	= np.fabs(sum(self.utils.SubVec(way_point_quat[1:4], adjustment_quat[1:4]))) 

		return (0.25 * axis_mag + 0.75 * adjustment_quat[0]) #25% of cost from different in axis

	def TestCost(self, x):
		#print 'args are',x,self.end_t_target

		self.wp_idx += 1
		self.wp_idx = self.wp_idx % 23


		r_state = RobotStateSaver(self.robot)
		tool_pose = []
		with self.env:
			with RobotStateSaver(self.robot):
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x)
				tool_pose = self.a_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()

		self.handles.append(self.env.plot3(tool_pose[0:3,3],
										   pointsize=5.0,
										   colors=array(((0,1,0)))))
		# return np.linalg.norm(self.utils.SubVec(self.pose_targets[self.wp_idx][0:3,3], tool_pose[0:3,3]))
		# print self.wp_idx, len(self.pose_targets)
		# print 'pose_target', self.pose_targets[self.wp_idx][0:3,3]
		# print 'joint_var', x

		if (self.wp_idx == 22):
			time.sleep(0.5)
			self.handles = []

		return 0

	def GogglesFKConstraint(self, x):
		r_state 		= RobotStateSaver(self.robot)
		g_pose 			= self.goggles.GetTransform()
		# g_quat 			= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(g_pose[0:3, 0:3]))
		ee_transform 	= self.a_manip.GetEndEffector().GetTransform()
		# ee_quat 		= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(ee_transform[0:3, 0:3]))
		cost 			= 0

		with r_state:
			if (self.env_locked == False):
				with self.env:
					self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
					self.robot.SetActiveDOFValues(x)

				tool_pose 			= self.a_manip.GetEndEffector().GetTransform() #do forward kinematics
				tool_quat			= self.utils.ArrayToQuaT(transformations.quaternion_from_matrix(tool_pose[0:3, 0:3]))
				pos_diff 			= self.utils.SubVec(tool_pose[0:3,3], ee_transform[0:3,3])
				new_g_pose 			= np.copy(g_pose)
				new_g_pose[0:3,3] 	= self.utils.AddVec(new_g_pose[0:3,3], pos_diff)
				# new_g_quat			= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(new_g_pose[0:3, 0:3]))
				# adjustment_quat		= self.utils.QuaternionFromTo(ee_quat, tool_quat)

	   # 			mult_quat =  transformations.quaternion_multiply(self.utils.QuatToArray(g_quat),
	   # 															 self.utils.QuatToArray(adjustment_quat))
	   # 			mult_quat = self.utils.QuatToArray(mult_quat)

	   # 			if (self.utils.QuatMag(mult_quat) > 1e-3):
	   # 				cost += 500
	   # 			else:
				# with self.env:
				# 	self.goggles.SetTransform(new_g_pose)
		  #  			if self.env.CheckCollision(self.goggles):
		  #  				cost += 500
				self.goggles_robot.MoveLenses(new_g_pose, self.env_locked)

	   			# if self.env.CheckCollision(self.goggles):
	   			if (self.env.CheckCollision(self.goggles_robot.lens_body)):
	   				print 'collision'
	   				cost = 20
			else:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x)

				tool_pose 			= self.a_manip.GetEndEffector().GetTransform() #do forward kinematics
				# tool_quat			= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(tool_pose[0:3, 0:3]))
				pos_diff 			= self.utils.SubVec(tool_pose[0:3,3], ee_transform[0:3,3])
				new_g_pose 			= np.copy(g_pose)
				new_g_pose[0:3,3] 	= self.utils.AddVec(new_g_pose[0:3,3], pos_diff)
				# new_g_quat			= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(new_g_pose[0:3, 0:3]))
				# adjustment_quat		= self.utils.QuaternionFromTo(ee_quat, tool_quat)

	   # 			mult_quat =  transformations.quaternion_multiply(self.utils.QuatToArray(g_quat),
	   # 															 self.utils.QuatToArray(adjustment_quat))
	   # 			mult_quat = self.utils.QuatToArray(mult_quat)

	   # 			if (self.utils.QuatMag(mult_quat) > 1e-3):
	   # 				cost += 500
	   # 			else:
				# self.goggles.SetTransform(new_g_pose)

	   # 			if self.env.CheckCollision(self.goggles):
	   # 				cost += 500
				self.goggles_robot.MoveLenses(new_g_pose, self.env_locked)

	   			# if self.env.CheckCollision(self.goggles):
	   			if (self.env.CheckCollision(self.goggles_robot.lens_body)):
	   				print 'collision'
	   				cost = 20

		r_state.Restore()
		r_state.Release()
		# self.goggles.SetTransform(g_pose)
		self.goggles_robot.MoveLenses(g_pose, self.env_locked)

		return cost

	def ModGogglesFKConstraint(self, x):
		g_pose 			= self.goggles.GetTransform()
		ee_transform 	= self.a_manip.GetEndEffector().GetTransform()

		cost = 0
		if (self.env_locked == False):
			with self.env:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x)
		else:
			self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
			self.robot.SetActiveDOFValues(x)

		tool_pose 			= self.a_manip.GetEndEffector().GetTransform() #do forward kinematics
		pos_diff 			= self.utils.SubVec(tool_pose[0:3,3], ee_transform[0:3,3])
		new_g_pose 			= np.copy(g_pose)
		new_g_pose[0:3,3] 	= self.utils.AddVec(new_g_pose[0:3,3], pos_diff)
		self.goggles_robot.MoveLenses(new_g_pose, self.env_locked)
		if (self.env.CheckCollision(self.goggles_robot.lens_body, self.human)):
			print 'collision'
			cost = 20
		self.goggles_robot.MoveLenses(g_pose, self.env_locked)

		return cost

	def JointVelCost(self, x):
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

		print 'value', (np.square(diffAxis).dot(diag_mat)).sum()

		return (np.square(diffAxis).dot(diag_mat)).sum()

	def ManipulabilityCost(self, x):
		jac = self.a_manip.CalculateJacobian()
		return np.sqrt(np.linalg.det(jac.dot(jac.transpose())))

	def PredictabilityCost(self, x):
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

		return np.exp(np.sum(diffAxis))

	def LegibilityCost(self, x):
		#whole trajectory legibility cost
		return 0



class CostObject:
	pose_target  	   = []
	constraint_target  = []
	current_pose 	   = []

	env 	 	  = None
	robot 	 	  = None
	utils 	 	  = None
	a_manip	 	  = None
	goggles  	  = None
	g_pose   	  = None
	g_quat 	 	  = None
	gradient 	  = None
	goggles_robot = None

	task_dev_coeff 	= 100
	cart_dev_frac 	= 0.75
	orient_dev_frac = 0.25

	parent_node = None
	child_node 	= None

	handles = []

	human_trajopt = None
	time_step = 0

	# variables for partial trajectory evaluation
	start_idx 	= -1
	end_idx 	= -1
	eval_idx 	= -1

	def Init(self, pose_target, constraint_target, env, robot, utils, a_manip, goggles, goggles_robot, parent = None, child = None):
		self.pose_target 		= pose_target
		self.current_pose		= pose_target
		self.constraint_target	= constraint_target
		self.env 				= env
		self.robot 				= robot
		self.utils 				= utils
		self.a_manip 			= a_manip
		self.parent_node		= parent
		self.child_node			= child

		if (goggles != None):
			self.goggles 			= goggles
			self.g_pose 			= self.goggles.GetTransform()
			self.g_quat 			= self.utils.ArrayToQuat(transformations.quaternion_from_matrix(self.g_pose[0:3, 0:3]))
			self.goggles_robot		= goggles_robot


	def CartDeviationCost(self, x):
		# r_state = RobotStateSaver(self.robot)
		r_state = self.robot.CreateRobotStateSaver()
		tool_pose = []
		# with self.env: #env already locked when optimizing the problem (double env locks safe??)

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

	def TaskDeviationCost(self, x):
		r_state = self.robot.CreateRobotStateSaver()
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

	def TaskDeviationCostWithObj(self, x):
		r_state = self.robot.CreateRobotStateSaver()
		t0_ee 	= self.a_manip.GetEndEffector().GetTransform()
		t0_obj 	= self.goggles.GetTransform()
		tee_obj = np.dot(np.linalg.inv(t0_ee), t0_obj)

		with r_state:
			self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
			self.robot.SetActiveDOFValues(x)
			t0_ee = self.a_manip.GetEndEffector().GetTransform() 
		r_state.Restore()
		r_state.Release()

		t0_obj = np.dot(t0_ee, tee_obj)
		tpose_obj = np.dot(np.linalg.inv(self.pose_target), t0_obj)

		delta = np.array([tpose_obj[0,3],
						  tpose_obj[1,3],
						  tpose_obj[2,3],
						  np.arctan2(tpose_obj[2,1], tpose_obj[2,2]),
						  -np.arcsin(tpose_obj[2,0]),
						  np.arctan2(tpose_obj[1,0], tpose_obj[0,0])])

		return np.linalg.norm(delta)

	def SlopeCost(self, x):
		if (self.parent_node == None):
			return 0

		r_state = self.robot.CreateRobotStateSaver()
		t0_ee 	= self.a_manip.GetEndEffector().GetTransform()
		with r_state:
			self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
			self.robot.SetActiveDOFValues(x)
			t0_ee = self.a_manip.GetEndEffector().GetTransform() 
		r_state.Restore()
		r_state.Release()


		orig_slope = self.utils.CalcSlope(self.parent_node.pose_target[0:3,3], self.pose_target[0:3,3])
		curr_slope = self.utils.CalcSlope(self.parent_node.pose_target[0:3,3], t0_ee[0:3,3])	
		# orig_slope = self.utils.CalcSlope(self.parent_node.pose_target[0:3,3], self.pose_target[0:3,3])
		# curr_slope = self.utils.CalcSlope(self.parent_node.current_pose[0:3,3], t0_ee[0:3,3])

		diff_slope = 0
		if (np.sign(orig_slope[0]) != np.sign(curr_slope[0]) and np.fabs(orig_slope[0] - curr_slope[0]) > 1e-4):
			diff_slope += 1
		elif (np.sign(orig_slope[1]) != np.sign(curr_slope[1]) and np.fabs(orig_slope[1] - curr_slope[1]) > 1e-4):
			diff_slope += 1
		elif (np.sign(orig_slope[2]) != np.sign(curr_slope[2]) and np.fabs(orig_slope[2] - curr_slope[2]) > 1e-4):
			diff_slope += 1

		self.current_pose = t0_ee
		return diff_slope * 0.1

		# o_dir = (self.pose_target[0:3,3] - self.parent_node.pose_target[0:3,3])/np.linalg.norm(self.pose_target[0:3,3] - self.parent_node.pose_target[0:3,3])
		# n_dir = (t0_ee[0:3,3] - self.parent_node.pose_target[0:3,3])/np.linalg.norm(t0_ee[0:3,3] - self.parent_node.pose_target[0:3,3])		
		# cost = 0

		# print 'o_dir', o_dir
		# print 'n_dir', n_dir
		# for i in range(0,3):
		# 	cost += np.fabs(o_dir[i] - n_dir[i])*0.5 * (2 - np.abs(np.sign(o_dir[i]) + np.sign(n_dir[i])))
		# print 'cost', cost

		# return 0.1 * cost

	def SlopeCostWithMemory(self, x):
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
		t0_ee_prev 	= self.a_manip.GetEndEffector().GetTransform()
		t0_ee_curr 	= self.a_manip.GetEndEffector().GetTransform()
		t0_ee_next 	= self.a_manip.GetEndEffector().GetTransform()

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

		if (self.child_node != None):
			with r_state:
				self.robot.SetActiveDOFs(self.a_manip.GetArmIndices())
				self.robot.SetActiveDOFValues(x_new[self.end_idx, :])
				t0_ee_next = self.a_manip.GetEndEffector().GetTransform() 
			r_state.Restore()
			r_state.Release()

		cost_val = 0
		#use dot product to calculate difference
		if (self.parent_node != None):
			prev_orig = np.linalg.norm(self.pose_target[0:3,3] - self.parent_node.pose_target[0:3,3])
			prev_curr = np.linalg.norm(t0_ee_curr[0:3,3] - t0_ee_prev[0:3,3])
			cost_val += np.dot(prev_curr, prev_orig)

		if (self.child_node != None):
			next_orig = np.linalg.norm(self.child_node.pose_target[0:3,3] - self.pose_target[0:3,3])
			next_curr = np.linalg.norm(t0_ee_next[0:3,3] - t0_ee_curr[0:3,3])
			cost_val += np.dot(next_curr, next_orig)

		return cost_val * 10

		