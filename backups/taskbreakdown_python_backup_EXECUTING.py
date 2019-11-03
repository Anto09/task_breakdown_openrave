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

from utilities import Utilities
from costwrapper import *
from goggles_robot import GogglesRobot
from baxter_openrave_traj_execution import *

def waitrobot(robot):
	"""busy wait for robot completion"""
	while not robot.GetController().IsDone():
		time.sleep(0.01)
			
class TaskBreakdown:

	robot 			= None
	human 			= None
	goggles 		= None
	forehead 		= None
	env 			= None
	r_manip 		= None
	l_manip 		= None
	utils 			= None
	costwrapper 	= None
	goggles_robot	= None

	original_path 			= []
	gridPositions 			= []
	cachedBodyLocations 	= []
	motionBodyLocations 	= []
	sortedBodyLocations 	= []
	cachedBodyTransforms	= []
	motionBodyTransforms 	= []
	sortedBodyTransforms 	= []
	ppePathLocations		= []
	ppePathTransforms 		= []
	neutral_ppe_trans		= []

	#transforms
	orig_goggles_transform 	= []
	orig_forehead_transform = []

	#geometry_msgs/Pose
	two_arm_planning		= False
	path_start_point 		= None
	transfer_path			= None
	transfer_path_right		= None
	transfer_path_left		= None
	support_path			= None
	valid_transfer_path		= None
	valid_support_path		= None

	#lists of geometry_msgs/Pose
	valid_transfer_starts_right 		= []
	valid_transfer_changepoints_right	= []
	valid_transfer_ends_right			= []
	#list of lists of numpy matrices
	valid_transfer_waypoints_right 	= []
	valid_human_transforms_right	= []
	valid_goggles_transforms_right	= []
	valid_forehead_transforms_right	= []

	valid_transfer_starts_left			= []
	valid_transfer_changepoints_left	= []
	valid_transfer_ends_left			= []
	#list of lists of numpy matrices
	valid_transfer_waypoints_left 	= []
	valid_human_transforms_left		= []
	valid_goggles_transforms_left	= []
	valid_forehead_transforms_left	= []

	#for drawing stuff with OpenRAVE
	draw_handles = []

	#grid
	cube_size = 0.05 #length of grid cube

	#collapse index
	state_change_index = -1

	#right goggles links
	back_strap_right 	= None
	lens_body_right  	= None
	left_strap1_right	= None
	left_strap2_right 	= None
	right_strap1_right 	= None
	right_strap2_right 	= None

	#left goggles links
	back_strap_left 	= None
	lens_body_left  	= None
	left_strap1_left	= None
	left_strap2_left 	= None
	right_strap1_left 	= None
	right_strap2_left 	= None

	#goggle joint vals
	i_val = -0.22
	max_val = -0.32

	def Init(self, robot, env, human, goggles, goggles_robot, forehead, handles = None):
		self.robot 			= robot
		self.env 			= env
		self.human 			= human
		self.goggles 		= goggles
		self.forehead 		= forehead
		self.utils 			= Utilities()
		self.r_manip 		= self.robot.SetActiveManipulator('rightarm')
		self.l_manip 		= self.robot.SetActiveManipulator('leftarm')
		self.goggles_robot 	= goggles_robot

		if (handles != None):
			self.draw_handles = handles

		self.costwrapper = CostWrapper()
		self.costwrapper.Init(self.robot, self.env, self.human, self.goggles, self.utils, self.r_manip, self.l_manip)

		self.orig_goggles_transform  = self.goggles.GetTransform()
		self.orig_forehead_transform = self.forehead.GetTransform()

		self.back_strap_right 	 = self.robot.GetLink('backstrap')
		self.lens_body_right  	 = self.robot.GetLink('lenses')
		# self.left_strap1_right	 = self.robot.GetLink('leftstrap1')
		# self.right_strap1_right 	 = self.robot.GetLink('rightstrap1')
		self.left_strap2_right  	 = self.robot.GetLink('leftstrap2')
		self.right_strap2_right 	 = self.robot.GetLink('rightstrap2')

		self.back_strap_left 	 = self.robot.GetLink('backstrap2')
		self.lens_body_left  	 = self.robot.GetLink('lenses2')
		# self.left_strap1_left	 = self.robot.GetLink('leftstrap12')
		# self.right_strap1_left 	 = self.robot.GetLink('rightstrap12')
		self.left_strap2_left  	 = self.robot.GetLink('leftstrap22')
		self.right_strap2_left 	 = self.robot.GetLink('rightstrap22')

	def GetIKSolutions(self, Tgoal, manip = None, getAll = True): #manip is OpenRAVE manip
		sol = None
		if (self.env == None or self.robot == None or manip == None):
			print('\n Error: Undefined env, robot, or manip \n')
			return sol

		with self.env:
			#sol = manip.FindIKSolutions(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
			if (getAll):
				sol = ku.ik_for_link(Tgoal, manip, manip.GetEndEffector().GetName(), filter_options = openravepy.IkFilterOptions.CheckEnvCollisions, return_all_solns = True)
			else:
				sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) 

		return sol

	def InterpolateTrajectories(self):	    
		num_points = 3
		new_transfer_traj = self.utils.RediscretizeTrajectory(self.transfer_path, num_points)
		for traj_point in new_transfer_traj:
			self.DrawLocation(traj_point, [0, 0, 1])
			self.DrawOrientation(traj_point)
		self.transfer_path = new_transfer_traj

		self.state_change_index *= (num_points + 1)

		new_support_traj = self.utils.RediscretizeTrajectory(self.support_path, num_points)
		for traj_point in new_support_traj:
			self.DrawLocation(traj_point, [0, 1, 0])
			self.DrawOrientation(traj_point)
		self.support_path = new_support_traj

	def ObtainClosestBodyTransform(self, ppe_path):
		if (len(ppe_path) < 1):
			print ('\n Error: Invalid path length')
			return

		orig_ppe_loc = ppe_path[0]

		###cachedBodyLocations to be used as "original" configuration transforms of the human###
		orig_ppe_distances = []
		ppe_distances = []
		print_ppe_distances = []

		for tup in self.cachedBodyLocations:
			if (len(tup) != 3):
				continue
			orig_dist = np.linalg.norm(orig_ppe_loc - tup[2])

			orig_ppe_distances.append(tuple([tup[0], tup[1], orig_dist]))
			ppe_distances.append(tuple([tup[0], tup[1], 0]))


		mot_idx = 0
		for motion_locations in self.motionBodyLocations:
			idx = 0
			for tup in motion_locations:
				new_dist = ppe_distances[idx][2]
				#negative = got nearer, positive = got further
				new_dist += (np.linalg.norm(tup[2] - ppe_path[mot_idx]) - orig_ppe_distances[idx][2])
				ppe_distances[idx] = tuple([tup[0], tup[1], new_dist])
				idx += 1
			mot_idx += 1

		print ppe_distances[0]

		ppe_distances.sort(key=lambda tup : tup[2])
		for tup in ppe_distances:

			name1 = tup[0]
			name1 = name1.replace('Dummy', '')
			name1 = name1.replace('X', '')
			name1 = name1.replace('Y', '')
			name1 = name1.replace('Z', '')
			name1 = name1.replace('Trans', '')
			name1 = name1.replace('Rot', '')

			name2 = tup[1]
			name2 = name2.replace('Dummy', '')
			name2 = name2.replace('X', '')
			name2 = name2.replace('Y', '')
			name2 = name2.replace('Z', '')
			name2 = name2.replace('Trans', '')
			name2 = name2.replace('Rot', '')

			print_ppe_distances.append(tuple([name1, name2, tup[2]]))

			#print tuple([name1, name2, tup[2]])
			print tup

		ppe_distances.reverse()
		self.sortedBodyLocations = ppe_distances

		print 'most significant body part is', self.sortedBodyLocations[0]
		return ppe_distances

	def PurifyIntoTransfer(self, ppe_transform_path, env):
		print "length ppe_transform_path", len(ppe_transform_path)
		print "length self.motionBodyTransforms", len(self.motionBodyTransforms)

		if (self.env == None):
			self.env = env
		self.ppePathTransforms = ppe_transform_path[:]

		transfer_transforms = []

		mot_idx = 0
		sig_name = self.sortedBodyLocations[0][1] #name of most significant transform
		for ppe_t in ppe_transform_path:
			ppe_trans = ppe_t[:]
			motion_transforms = self.motionBodyTransforms[mot_idx][:]
			body_trans = None
			neutral_body_trans = None
			#find most significant transform
			for trans in motion_transforms:
				if (trans[1] == sig_name):
					body_trans = trans[:]
					break
			for trans in self.cachedBodyTransforms:
				if (trans[1] == sig_name):
					neutral_body_trans = trans[:]
					break
			
			body_to_world = np.linalg.inv(body_trans[2])
			body_to_ppe = np.dot(body_to_world, ppe_trans)
			transfer_trans = np.dot(neutral_body_trans[2], body_to_ppe)
			transfer_trans[0,3] -= (0.1 + 0.035)
			transfer_transforms.append(transfer_trans[:])
			self.DrawLocation(transfer_trans[0:3,3], [0,0,1])
			self.DrawOrientation(transfer_trans)
			mot_idx += 1
		self.transfer_path = transfer_transforms[:]

	def PurifyIntoSupport(self, ppe_transform_path, env):
		if (self.env == None):
			self.env = env
		if (len(self.ppePathTransforms) == 0):
			self.ppePathTransforms = ppe_transform_path[:]

		support_transforms = []

		if (len(self.neutral_ppe_trans) == 0):
			self.neutral_ppe_trans = ppe_transform_path[0]

		mot_idx = 0
		sig_name = self.sortedBodyLocations[0][1]
		for ppe_trans in ppe_transform_path:
			motion_transforms = self.motionBodyTransforms[mot_idx][:]
			body_trans = None
			#find most significant transform
			for trans in motion_transforms:
				if (trans[1] == sig_name):
					body_trans = trans[:]
					break

			new_ppe_trans = np.copy(ppe_trans)
			new_ppe_trans[0:3, 0:3] = np.identity(3)
			ppe_to_world = np.linalg.inv(new_ppe_trans)
			ppe_to_body = np.dot(ppe_to_world, body_trans[2])
			support_trans = np.dot(self.neutral_ppe_trans, ppe_to_body)
			support_transforms.append(support_trans)

			self.draw_handles.append(self.env.plot3(points=support_trans[0:3,3],
									   pointsize=5.0,
									   colors=array(((0,1,0)))))

			mot_idx += 1
		self.support_path = support_transforms

	def AppendValidStartAndEnd(self, displacement_vec, adjustment_quat, human_transform, goggles_transform, forehead_transform):
		T_target_start = self.utils.ModifyTransform(self.transfer_path[0], displacement_vec, adjustment_quat)
		T_target_end = self.utils.ModifyTransform(self.transfer_path[len(self.transfer_path)-1], displacement_vec, adjustment_quat)

		new_path_point_start = self.utils.ModifyTransformToPoint(self.transfer_path[0], displacement_vec, adjustment_quat)
		new_path_point_cp = self.utils.ModifyTransformToPoint(self.transfer_path[self.state_change_index], displacement_vec, adjustment_quat)
		new_path_point_end = self.utils.ModifyTransformToPoint(self.transfer_path[len(self.transfer_path)-1], displacement_vec, adjustment_quat)

		r_start_sol = self.GetIKSolutions(T_target_start, self.r_manip)
		r_end_sol 	= self.GetIKSolutions(T_target_end, self.r_manip)

		l_start_sol = self.GetIKSolutions(T_target_start, self.l_manip)
		l_end_sol 	= self.GetIKSolutions(T_target_end, self.l_manip)

		#for cutting testing, return indices with valid right arm trajectories
		hasSol = False
		#valid starts and ends will always be the same length
		if (r_start_sol != None and r_end_sol != None and len(r_start_sol) > 0 and len(r_end_sol) > 0):
			print ('Append Valid Start And End For Right')
			self.valid_transfer_starts_right.append(new_path_point_start)
			self.valid_transfer_changepoints_right.append(new_path_point_cp)
			self.valid_transfer_ends_right.append(new_path_point_end)
			
			valid_transfer_waypoints = []
			for ppeTrans in self.transfer_path:	
				T_target = self.utils.ModifyTransform(ppeTrans, displacement_vec, adjustment_quat)
				valid_transfer_waypoints.append(T_target)
			self.valid_transfer_waypoints_right.append(valid_transfer_waypoints)
			self.valid_human_transforms_right.append(human_transform)
			self.valid_goggles_transforms_right.append(goggles_transform)
			self.valid_forehead_transforms_right.append(forehead_transform)
			hasSol = True

		# if (l_start_sol != None and l_end_sol != None and len(l_start_sol) > 0 and len(l_end_sol) > 0):
		# 	print ('Append Valid Start And End For Left')
		# 	self.valid_transfer_starts_left.append(new_path_point_start)
		# 	self.valid_transfer_ends_left.append(new_path_point_end)

		# 	valid_transfer_waypoints = []
		# 	# for ppeTrans in self.ppePathTransforms:		
		# 	for ppeTrans in self.transfer_path:		
		# 		T_target = self.utils.ModifyTransform(ppeTrans, displacement_vec, adjustment_quat)
		# 		valid_transfer_waypoints.append(T_target)
		# 	self.valid_transfer_waypoints_left.append(valid_transfer_waypoints)
		# 	self.valid_human_transforms_left.append(human_transform)
		# 	self.valid_goggles_transforms_left.append(goggles_transform)
		# 	self.valid_forehead_transforms_left.append(forehead_transform)
		# 	hasSol = True

		return hasSol

	def TransformWorker(self, h_trans, f_trans):
		with self.env:
			self.human.SetTransform(h_trans)
			self.forehead.SetTransform(f_trans)

	# modify to only consider points near baxter(set by radius parameter)

	def FindBestDisplacementTwoArms(self):
		person_axis 	= np.array([0,0,1])
		person_angle 	= np.pi
		person_quat 	= transformations.quaternion_about_axis(person_angle, person_axis)

		#return values
		best_displacement 	= None
		best_path 			= None
		best_sols 			= None
		best_arm 			= None #for single-arm manipulation

		#generate grid positions
		self.gridPositions = []
		x_idx = self.robot.GetTransform()[0,3]
		y_idx = self.robot.GetTransform()[1,3]
		for i in range(-20, 20):
			for j in range(-20, 20):
				grid_point = Point()
				grid_point.x = float(i) * 0.05
				grid_point.y = float(j) * 0.05
				grid_point.z = path_start_pos.z
				self.gridPositions.append(grid_point)

		orig_human_transform = self.human.GetTransform()

		angle = 0
		r_traj = []
		l_traj = []

		#PPE specific function calls

		for a in range(0, 1): #angle discretization
			angle = float(a) * numpy.pi * 0.25

			#reorient person 


	def FindBestDisplacement(self):
		#assume person is in neutral orientation, i.e. facing the robot with parallel x-axes
		disp = None
		if (self.env == None or self.robot == None or len(self.transfer_path) <= 0):
			print('\n Undefined env or robot, length of path or grid positions must be greater than 0 \n')
			return disp

		self.original_path = self.transfer_path

		path_start_pos 		= self.utils.VecToPoint(self.transfer_path[0][0:3,3])
		person_axis 		= np.array([0,0,1])
		person_quat_array 	= transformations.quaternion_about_axis(0, person_axis)
		person_quat 		= self.utils.ArrayToQuat(person_quat_array)

		#return values
		best_displacement 	= None
		best_path 			= None
		best_sols 			= None
		best_arm 			= None #for single-arm manipulation

		#generate grid positions
		self.gridPositions = []
		x_idx = self.robot.GetTransform()[0,3]
		y_idx = self.robot.GetTransform()[1,3]
		for i in range(-20, 20):
			for j in range(-20, 20):
				grid_point = Point()
				grid_point.x = float(i) * 0.05
				grid_point.y = float(j) * 0.05
				grid_point.z = path_start_pos.z
				self.gridPositions.append(grid_point)

		orig_human_transform = self.human.GetTransform()

		angle = 0
		r_traj = []
		l_traj = []

		#PPE specific function calls
		self.CollapseGoggles()

		for a in range(0, 1):
			angle = float(a) * np.pi * 0.25

			#person orientation
			axis 					= numpy.array([0, 0, 1]) #always rotate about z axis
			quat_array 				= transformations.quaternion_about_axis(angle, axis) #np_array
			adjustment_quat 		= self.utils.ArrayToQuat(quat_array) #geometry_msgs/Quaternion
			new_person_quat 		= self.utils.MultiplyQuaternion(person_quat, adjustment_quat) #geometry_msgs/Quaternion
			new_person_quat_array 	= self.utils.QuatToArray(new_person_quat) #np_array

			angle_transform = transformations.quaternion_matrix(new_person_quat_array) #4x4 numpy array

			self.move_human_to_neutral()
			self.goggles_robot.Reset()

			for i in range(0, len(self.gridPositions)):
				r_traj = []
				l_traj = []
				grid_pos = self.gridPositions[i] 
				displacement_vec = self.utils.SubPoint(grid_pos, path_start_pos) #geometry_msgs/Point
				new_displacement_vec = self.utils.RotateVector(adjustment_quat, displacement_vec)

				r_valid_count = 0
				l_valid_count = 0
				success = False

				human_transform = np.copy(orig_human_transform)
				human_transform[0:3, 0:3] = angle_transform[0:3, 0:3]

				self.utils.DisplaceTransform(human_transform, displacement_vec)

				forehead_transform = np.copy(self.orig_forehead_transform)
				forehead_transform[0:3, 0:3] = angle_transform[0:3, 0:3]
				self.utils.DisplaceTransform(forehead_transform, displacement_vec)

				transfer_path_start = self.utils.ModifyTransform(np.copy(self.transfer_path[0]), displacement_vec, adjustment_quat)
				if (transfer_path_start[0,3] <= self.robot.GetTransform()[0,3] or 
					np.fabs(transfer_path_start[0,3] - self.robot.GetTransform()[0,3]) > 0.6 or
					np.fabs(transfer_path_start[1,3] - self.robot.GetTransform()[1,3]) > 0.3 or
					np.linalg.norm(transfer_path_start[0:3,3] - self.robot.GetTransform()[0:3,3]) > 1):
					print("invalid due to location")
					continue


				self.TransformWorker(human_transform, forehead_transform)

				if (self.env.CheckCollision(self.robot)):
					print("invalid due to collision")
					continue

				new_path = []  #array of geometry_msgs/Pose

				if (self.AppendValidStartAndEnd(new_displacement_vec, adjustment_quat, human_transform, goggles_transform, forehead_transform)):
					print 'displacement', new_displacement_vec, 'and rotation', adjustment_quat, 'has a valid start and end'

				pidx = 0
				for path_transform in self.transfer_path:
					T_target = self.utils.ModifyTransform(np.copy(path_transform), displacement_vec, adjustment_quat)

					r_sols = self.GetIKSolutions(T_target, self.r_manip)
					l_sols = self.GetIKSolutions(T_target, self.l_manip)

					has_right_sol = False
					has_left_sol = False
					has_sol = False
					if (r_sols != None and len(r_sols) > 0):
						r_valid_count += 1
						has_sol = True
						has_right_sol = True
						r_traj.append(r_sols[0])
					# 	# self.MoveManip(self.r_manip, r_sols[0])
					# 	# raw_input('press enter to continue...')

					# if (l_sols != None and len(l_sols) > 0):
					# 	l_valid_count += 1
					# 	has_sol = True
					# 	has_left_sol = True
					# 	l_traj.append(l_sols[0])
						# self.MoveManip(self.l_manip, l_sols[0])
						# raw_input('press enter to continue...')

					# if (has_right_sol == False):
					# 	self.DrawLocation(T_target, [1, 0, 0])
					# else:
					# 	self.DrawLocation(T_target, [0, 1, 0])
						# self.DrawOrientation(T_target)
						# from_vec = T_target[0:3,3]
						# break # for testing only

					# if (has_left_sol == False):
					# 	self.DrawLocation(T_target, [1, 0.5, 0.2])
					# else:
					# 	self.DrawLocation(T_target, [0, 0, 1])
						# self.DrawOrientation(T_target)
						# from_vec = T_target[0:3,3]
						# break # for testing only

					if (has_sol == False):
						break

					pidx += 1

				if (r_valid_count == len(self.transfer_path)): #add scoring here to pick right over left or vice versa
					best_displacement 	= displacement_vec
					best_path 			= new_path
					best_sols 			= r_sols
					best_arm 			= 'rightarm'

					self.valid_transfer_path = []
					for path_point in new_path:
						T_rot = transformations.quaternion_matrix(self.utils.QuatToArray(path_point.orientation))

						# self.DrawLocation(path_point, [0, 0, 1])
						# self.DrawOrientation(T_rot, path_point)

						T_mat = np.copy(T_rot)
						T_mat[0:3,3] = [path_point.position.x, path_point.position.y, path_point.position.z]
						self.valid_transfer_path.append(T_mat)

					print ('for right, best displacement \n', displacement_vec)
					self.TransformWorker(orig_human_transform, self.orig_forehead_transform)

					# return #pick first solution only for testing

				# if (l_valid_count == len(self.transfer_path)): #add scoring here to pick right over left or vice versa
				# 	best_displacement 	= displacement_vec
				# 	best_path 			= new_path
				# 	best_sols 			= l_sols
				# 	best_arm 			= 'leftarm'

				# 	self.valid_transfer_path = []
				# 	for path_point in new_path:
				# 		T_rot = transformations.quaternion_matrix(self.utils.QuatToArray(path_point.orientation))

				# 		# self.DrawLocation(path_point, [0, 0, 1])
				# 		# self.DrawOrientation(T_rot, path_point)

				# 		T_mat = np.copy(T_rot)
				# 		T_mat[0:3,3] = [path_point.position.x, path_point.position.y, path_point.position.z]
				# 		self.valid_transfer_path.append(T_mat)

				# 	print ('for left, best displacement \n', displacement_vec)
				# 	print i
				# 	self.TransformWorker(orig_human_transform, self.orig_forehead_transform)

		print ('Returning best displacement')
		return tuple([best_displacement, best_path, best_sols, best_arm])

	def GenerateTraj(self, poses, manip, interp = False):
		traj = []
		idx = -1
		prev_sol = None
		no_sol_count = 0.0

		#how to pick the best IK solution
		for pose in poses:
			sol = self.GetIKSolutions(pose, manip)
			if (interp == True):
				if (prev_sol == None):				
					if (len(sol) > 0):
						traj.append(sol[0].tolist())
						idx+=1
					else:
						prev_sol = traj[idx]
						no_sol_count += 1.0
				else:			
					if (len(sol) > 0):
						#print 'interpolated!'
						sol_0 = prev_sol[:]
						sol_n = sol[0][:]
						sol_diff = self.utils.SubArr(sol_n, sol_0)
						x_n = no_sol_count+1
						x_0 = 0.0
						x = 1.0
						while (x < x_n):
							traj.append(self.utils.InterpolateConfig(sol_0, sol_n, sol_diff, x_0, x_n, x))
							x += 1.0
							idx+=1
						traj.append(sol[0].tolist())
						idx+=1
						prev_sol = None
						no_sol_count = 0
					else:
						no_sol_count += 1.0
			else:
				if (len(sol) > 0):
					traj.append(sol[0].tolist())
					idx+=1
					prev_sol = sol[0][:]
				else:
					traj.append(prev_sol[:].tolist())

		return traj

	def GenerateTrajFromPoseWaypoints(self, start_config, end_config, pose_waypoints, manip, n_steps):
		traj = []
		traj.append(start_config.tolist())
		current_pose = None
		idx = 0
		prev_sol = start_config

		for i in range(1, n_steps-1): #don't include start and end
			if (idx < len(pose_waypoints) and i == pose_waypoints[idx][1]): #update the c-space vector to be inserted into the traj
				# self.DrawLocation(pose_waypoints[idx][0][0:3,3], [0.7, 0.5, 0.0])
				sol = []
				with self.env:
					sol = self.GetIKSolutions(pose_waypoints[idx][0], manip)
				if (len(sol) > 0): #if there is no solution to the pose waypoint, stay stationary
					prev_sol = sol[0]
				idx+=1
			traj.append(prev_sol.tolist())
		traj.append(end_config.tolist())

		return traj

	def GetRelevantPathWaypoints(self, pose_waypoints): #retrieve points where slope signs or concavity change
		relevant_waypoints = []

		if (len(pose_waypoints) < 3):
			return relevant_waypoints

		for i in range(1, len(pose_waypoints) - 2): #don't include start and end points in calculation curvature at a given point
			before 	= pose_waypoints[i-1][:]
			current = pose_waypoints[i][:]
			after 	= pose_waypoints[i+1][:]

			if (self.utils.EvalSlopeDir(before, current, after) == True):
				relevant_waypoints.append(tuple([current, i]))
				# self.DrawLocation(current[0:3,3], [0.7,0.3,0.2])

		return relevant_waypoints

	def CalcDiscreteCurvature(self, p1, p2, p3):
		p1_arr = []
		p2_arr = []
		p3_arr = []

		if (type(p1) == Point):
			p1_arr = self.utils.PointToArray(p1)
		elif (len(p1) > 3): #transformation matrix
			p1_arr = p1[0:3,3]
		else:
			p1_arr = p1[:]

		if (type(p2) == Point):
			p2_arr = self.utils.PointToArray(p2)
		elif (len(p2) > 3): #transformation matrix
			p2_arr = p2[0:3,3]
		else:
			p2_arr = p2[:]

		if (type(p3) == Point):
			p3_arr = self.utils.PointToArray(p3)
		elif (len(p3) > 3): #transformation matrix
			p3_arr = p3[0:3,3]
		else:
			p3_arr = p3[:]

		p1p2 = self.utils.SubVec(p2_arr, p1_arr)
		p2p3 = self.utils.SubVec(p2_arr, p3_arr)

		#angle of vectors FROM p2
		p1p2mag = self.utils.VecMag(p1p2)
		p2p3mag = self.utils.VecMag(p2p3)

		print("denom:", p1p2mag * p2p3mag, "p1p2mag:", p1p2mag, "p2p3mag:", p2p3mag)
		o_angle = np.arccos(np.dot(p1p2, p2p3)/(p1p2mag * p2p3mag))
		angle = np.arctan2(np.sin(o_angle), np.cos(o_angle))

		m_angle = np.arctan2(np.linalg.norm(np.cross(p1p2,p2p3)), np.dot(p1p2, p2p3))

		return m_angle * 180 / np.pi

	def GetPathWaypointsCurvature(self, pose_waypoints, discrete = True): #retrieve points where path curvature is at maximum or concavity changes
		relevant_waypoints = []

		if (len(pose_waypoints) < 3):
			return relevant_waypoints

		for i in range(1, len(pose_waypoints) - 2): #don't include start and end points in calculation curvature at a given point
			before 	= pose_waypoints[i-1]
			current = pose_waypoints[i]
			after 	= pose_waypoints[i+1]

			angle = self.CalcDiscreteCurvature(before, current, after)

			if (180 - np.fabs(angle) > 15):
				relevant_waypoints.append(tuple([current, i]))

		return relevant_waypoints

	def GetTotalWaypoints(self, traj):
		total_waypoints = self.GetRelevantPathWaypoints(traj)
		for curve_waypoint in self.GetPathWaypointsCurvature(traj):
			contains = False
			for t_waypoint in total_waypoints:
				if (curve_waypoint[1] == t_waypoint[1]):
					contains = True
					break
			if (contains == False):
				total_waypoints.append(curve_waypoint)
		return total_waypoints

	def GetReachableWaypoints(self, pose_waypoints):
		r_waypoints = []
		for p_waypoint in pose_waypoints:
			w_sol = self.GetIKSolutions(p_waypoint[0], self.r_manip)
			# self.DrawLocation(p_waypoint[0],[0.3,0.5,0.7])
			print ('Is waypoint', p_waypoint[0][0:3, 3], 'reachable?', len(w_sol) > 0)
			if (len(w_sol) > 0):
				r_waypoints.append(p_waypoint)
		return r_waypoints

	def GenerateTrajoptRequest(self, start_xyz_target, start_rot_target, end_xyz_target, end_rot_target, n_steps, manip_name, init_joint_target, end_joint_target, init_traj = None):
		request = {
		  "basic_info" : {
			"n_steps" : n_steps, #num points of init traj
			"manip" : manip_name, # see below for valid values
			"start_fixed" : True #True # i.e., DOF values at first timestep are fixed based on current robot state
		  },
		  "costs" : [
		  {
			"type" : "joint_vel", # joint-space velocity cost
			"params": {"coeffs" : [250]} # a list of length one is automatically expanded to a list of length n_dofs
		  },
		  {
			"type" : "collision",
			"name" :"cont_coll",
			"params" : {
			  "continuous" : True,
			  "coeffs" : [500], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.0025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			}, 
		  }#,   
		 #  {
			# "type" : "collision",
			# "name" :"dis_coll",
			# "params" : {
			#   "continuous" : False,
			#   "coeffs" : [500], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			#   "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			# },
		 #  }
		  ],
		  "constraints" : [
		  {
			"type" : "pose", 
			"params" : {"xyz" : start_xyz_target.tolist(), 
						"wxyz" : start_rot_target, 
						"link": "right_hand",
						"timestep" : 0 #-1 of n steps
						}
						 
		  },
		  {
			"type" : "pose", 
			"params" : {"xyz" : end_xyz_target.tolist(), 
						"wxyz" : end_rot_target, 
						"link": "right_hand",
						"timestep" : n_steps-1 #-1 of n steps
						}
						 
		  },
		  {
			"type" : "cart_vel",
			"name" : "cart_vel",
			"params" : {
				"max_displacement" : 0.5,
				"first_step" : 0,
				"last_step" : n_steps-1, #inclusive
				"link" : "right_hand"
			},
		  }
		  ],
		  "init_info" : {
			  # "type" : "given_traj",
			  # "data" : init_traj
			  "type" : "straight_line",
			  "endpoint" : end_joint_target.tolist()
			  # "type" : "stationary"
		  }
		}

		if (init_traj != None):
			request["init_info"] = {
				"type" : "given_traj",
				"data" : init_traj.tolist()
				}
		
		return request

	def GenerateTrajoptRequestNoPose(self, n_steps, manip_name, init_joint_target, end_xyz_target, end_rot_target, end_joint_target, init_traj = None):

		request = {
		  "basic_info" : {
			"n_steps" : n_steps, #num points of init traj
			"manip" : manip_name, # see below for valid values
			"start_fixed" : True #True # i.e., DOF values at first timestep are fixed based on current robot state
		  },
		  "costs" : [
		  {
			"type" : "joint_vel", # joint-space velocity cost
			"params": {"coeffs" : [200]} # a list of length one is automatically expanded to a list of length n_dofs
		  },
		  {
			"type" : "collision",
			"name" :"cont_coll",
			"params" : {
			  "continuous" : True,
			  "coeffs" : [300], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			}, 
		  }#,   
		 #  {
			# "type" : "collision",
			# "name" :"dis_coll",
			# "params" : {
			#   "continuous" : False,
			#   "coeffs" : [300], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			#   "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			# },
		 #  }
		  ],
		  "constraints" : [
		  {
			"type" : "pose", 
			"params" : {"xyz" : end_xyz_target.tolist(), 
						"wxyz" : end_rot_target, 
						"link": "right_hand", #change later
						"timestep" : n_steps-1 #-1 of n steps
						}
						 
		  },
		  {
			"type" : "cart_vel",
			"name" : "cart_vel",
			"params" : {
				"max_displacement" : 0.5,
				"first_step" : 0,
				"last_step" : n_steps-1, #inclusive
				"link" : "right_hand"
			},
		  }
		  ],
		  "init_info" : {
			  "type" : "straight_line",
			  "endpoint" : end_joint_target.tolist()
		  }
		}
		
		return request

	def ExtendTrajoptRequest(self, request, waypoints, linkname):
		for waypoint in waypoints:
			request["constraints"].extend([
				{
					"type":"pose",
					"name":"path_pose_waypoint",
					"params":{
						"xyz": waypoint[0][0:3,3].tolist(),
						"wxyz": transformations.quaternion_from_matrix(waypoint[0][0:3,0:3]).tolist(),
						"link": "right_hand",
						"timestep": waypoint[1]
					}
				}
				])

			self.DrawLocation(waypoint[0], [0,0,1])
			self.DrawOrientation(waypoint[0])
				
		return request

	def SplitTraj(self, traj):
		traj_a = []
		traj_b = []
		for i in range(0, self.state_change_index):
			traj_a.append(traj[i])
		for i in range(self.state_change_index, len(traj)):
			traj_b.append(traj[i])

		return traj_a, traj_b

	def TrajoptTransferPath(self):
		if (self.transfer_path == None or len(self.transfer_path) == 0):
			print ('\n Error: Transfer path does not exist')
			return 

		print ('Trajopt-ing transfer path')
		#same number of transfer starts and ends (guaranteed)
		num_valid_right = len(self.valid_transfer_starts_right)
		num_valid_left 	= len(self.valid_transfer_starts_left)

		trajectories 	= []
		init_traj 		= []
		total_waypoints = []

		for i in range (0, num_valid_right): 
			print 'iteration:',i
			time.sleep(0.1)

			# part one
			trajectories = []
			self.draw_handles = []
			self.ExtendGoggles()
		
			start_xyz_target = self.utils.PointToArray(self.valid_transfer_starts_right[i].position)
			start_rot_target = self.utils.QuatToArray(self.valid_transfer_starts_right[i].orientation)

			end_xyz_target = self.utils.PointToArray(self.valid_transfer_changepoints_right[i].position)
			end_rot_target = self.utils.QuatToArray(self.valid_transfer_changepoints_right[i].orientation)

			start_t_target 	= openravepy.matrixFromPose(np.r_[start_rot_target, start_xyz_target])
			end_t_target 	= openravepy.matrixFromPose(np.r_[end_rot_target, end_xyz_target])

			total_targets = []
			for j in range(0, self.state_change_index+1):
				total_targets.append(self.valid_transfer_waypoints_right[i][j])

			self.TransformWorker(self.valid_human_transforms_right[i],,
								 self.valid_forehead_transforms_right[i])


			r_start_sol = self.GetIKSolutions(start_t_target, self.r_manip)
			r_end_sol = self.GetIKSolutions(end_t_target, self.r_manip)

			print "is start target reachable? ", len(r_start_sol) > 0
			print "is end target reachable? ", len(r_end_sol) > 0
			if (len(r_start_sol) == 0 or len(r_end_sol) == 0):
				print 'Start or end not reachable, continuing...'
				continue

			self.DrawLocation(start_xyz_target, [1,0,0])
			self.DrawLocation(end_xyz_target, [1,0,0])

			total_waypoints = self.GetTotalWaypoints(total_targets)
			time_val = 0.0;

			if (len(total_waypoints) > len(self.GetReachableWaypoints(total_waypoints))):
				print 'Not all waypoints are reachable, continuing...'
				continue

			if (len(r_start_sol) > 0):
				self.MoveManip(self.r_manip, r_start_sol[0]) 

				manip_name = self.r_manip.GetName()
				n_steps = self.state_change_index + 1 #len(self.valid_transfer_waypoints_right[i])
				init_traj = None # self.GenerateTrajFromPoseWaypoints(r_start_sol[0], r_end_sol[0], total_waypoints, self.r_manip, n_steps)

				for iters in range(0, 10):
					request = self.GenerateTrajoptRequest(start_xyz_target, start_rot_target, end_xyz_target, end_rot_target, n_steps, manip_name, r_start_sol[0], r_end_sol[0], init_traj)
					request = self.ExtendTrajoptRequest(request, total_waypoints, manip_name)

					s = json.dumps(request) # convert dictionary into json-formatted string
					prob = None 

					cost_handles = []
					trajectories = []
					self.costwrapper.a_manip = self.r_manip
					with self.env:	
						prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem

						idx = 0
						for t in range(0,n_steps):
							co = CostObject()
							co.Init(self.valid_transfer_waypoints_right[i][idx], None, 
									self.env, self.robot, self.utils, self.r_manip, self.goggles, None)
							co.time_step = t;
							if (t > 0):
								co.parent_node = cost_handles[t-1]
								co.parent_node.child_node = co
								co.start_idx = t-1
								co.eval_idx = t
							if (t < n_steps-1):
								co.end_idx = t+1
							cost_handles.append(co)

							prob.AddCost(co.TaskDeviationCost, [(t,j) for j in xrange(7)], "ABS")#, "up%i"%t)
							prob.AddCost(self.costwrapper.ManipulabilityCost, [(t,j) for j in xrange(7)], "ABS")
							prob.AddWholePathCost(co.SlopeCostWithMemory, [(t,j) for j in xrange(7)], "ABS", "right_hand")
							idx += 1

						t_start = time.time()

					raw_input("Press enter to continue...")
					try:
						with self.env:
							result 		= trajoptpy.OptimizeProblem(prob) # do optimization
							t_elapsed 	= time.time() - t_start
							trajectories.append(result.GetTraj())
							init_traj 	= result.GetTraj()
							print "optimization took %.3f seconds"%t_elapsed
							raw_input("Press enter to continue...")
					except:
						print "Unexpected error:", sys.exc_info()[0]

				for traj in trajectories:
					# traj = self.ShortcutSmoothing(traj, self.r_manip, total_waypoints)
					traj = self.utils.DiscretizeCspaceTrajectory(traj, 3)
					self.VisualizeTraj(traj, self.r_manip, self.valid_transfer_waypoints_right[i])
					time_inc = 5.0/len(traj)
					exec_traj = []
					neutral_traj = []
					for traj_point in traj:
						exec_traj_point = [time_val]
						for traj_val in traj_point: exec_traj_point.append(traj_val)
						exec_traj_point.append(0)
						exec_traj.append(exec_traj_point)
						neutral_traj.append([time_val, -0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50, 0])
						time_val += time_inc

					# execute_traj(exec_traj, neutral_traj)
					self.WriteTrajectoryToFile(exec_traj, neutral_traj, 'output_traj'+str(i), 'w')
					raw_input("Press enter to continue...")

			# part two
			trajectories = []
			# self.draw_handles = []
			self.CollapseGoggles()

			start_xyz_target = self.utils.PointToArray(self.valid_transfer_changepoints_right[i].position)
			start_rot_target = self.utils.QuatToArray(self.valid_transfer_changepoints_right[i].orientation)

			end_xyz_target = self.utils.PointToArray(self.valid_transfer_ends_right[i].position)
			end_rot_target = self.utils.QuatToArray(self.valid_transfer_ends_right[i].orientation)

			start_t_target 	= openravepy.matrixFromPose(np.r_[start_rot_target, start_xyz_target])
			end_t_target 	= openravepy.matrixFromPose(np.r_[end_rot_target, end_xyz_target])

			total_targets = []
			for j in range(self.state_change_index, len(self.valid_transfer_waypoints_right[i])):
				total_targets.append(self.valid_transfer_waypoints_right[i][j])

			self.TransformWorker(self.valid_human_transforms_right[i],
								 self.valid_forehead_transforms_right[i])


			r_start_sol = self.GetIKSolutions(start_t_target, self.r_manip)
			r_end_sol = self.GetIKSolutions(end_t_target, self.r_manip)

			print "is start target reachable? ", len(r_start_sol) > 0
			print "is end target reachable? ", len(r_end_sol) > 0
			if (len(r_start_sol) == 0 or len(r_end_sol) == 0):
				print 'Start or end not reachable, continuing...'
				continue

			self.DrawLocation(start_xyz_target, [1,0,0])
			self.DrawLocation(end_xyz_target, [1,0,0])

			total_waypoints = self.GetTotalWaypoints(total_targets)
			if (len(total_waypoints) > len(self.GetReachableWaypoints(total_waypoints))):
				print 'Not all waypoints are reachable, continuing...'
				continue

			# print '2nd waypoints'
			if (len(r_start_sol) > 0):
				self.MoveManip(self.r_manip, r_start_sol[0]) 

				manip_name = self.r_manip.GetName()
				n_steps = len(self.valid_transfer_waypoints_right[i]) - self.state_change_index
				init_traj = self.GenerateTrajFromPoseWaypoints(r_start_sol[0], r_end_sol[0], total_waypoints, self.r_manip, n_steps)

				for iters in range(0, 10):
					request = self.GenerateTrajoptRequest(start_xyz_target, start_rot_target, end_xyz_target, end_rot_target, n_steps, manip_name, r_start_sol[0], r_end_sol[0], None)
					request = self.ExtendTrajoptRequest(request, total_waypoints, manip_name)

					s = json.dumps(request) # convert dictionary into json-formatted string
					prob = None 

					cost_handles = []
					trajectories = []
					with self.env:	
						prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem

						idx = self.state_change_index
						for t in range(0,n_steps):
							co = CostObject()
							co.Init(self.valid_transfer_waypoints_right[i][idx], None, 
									self.env, self.robot, self.utils, self.r_manip, self.goggles, None)
							if (t > 0):
								co.parent_node = cost_handles[t-1]
								co.parent_node.child_node = co
								co.start_idx = t-1
								co.eval_idx = t
							if (t < n_steps-1):
								co.end_idx = t+1
							cost_handles.append(co)

							prob.AddCost(co.TaskDeviationCost, [(t,j) for j in xrange(7)], "ABS")#, "up%i"%t)
							prob.AddCost(self.costwrapper.ManipulabilityCost, [(t,j) for j in xrange(7)], "ABS")
							prob.AddWholePathCost(co.SlopeCostWithMemory, [(t,j) for j in xrange(7)], "ABS", "right_hand") #changes for two arm planning
							idx += 1

						t_start = time.time()

					try:
						with self.env:
							result = trajoptpy.OptimizeProblem(prob) # do optimization
							t_elapsed = time.time() - t_start
							trajectories.append(result.GetTraj())
							init_traj 	= result.GetTraj()
							print "optimization took %.3f seconds"%t_elapsed
							raw_input("Press enter to continue...")
					except:
						print "Unexpected error:", sys.exc_info()[0]

				for traj in trajectories:
					# traj = self.ShortcutSmoothing(traj, self.r_manip, total_waypoints)
					traj = self.utils.DiscretizeCspaceTrajectory(traj, 3)
					self.VisualizeTraj(traj, self.r_manip, self.valid_transfer_waypoints_right[i])
					# time_val = 0.0;
					time_inc = 10.0/len(traj)
					exec_traj = []
					neutral_traj = []
					for traj_point in traj:
						exec_traj_point = [time_val]
						for traj_val in traj_point: exec_traj_point.append(traj_val)
						exec_traj_point.append(0)
						exec_traj.append(exec_traj_point)
						neutral_traj.append([time_val, -0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50, 0])
						time_val += time_inc

					# execute_traj(exec_traj, neutral_traj)
					self.WriteTrajectoryToFile(exec_traj, neutral_traj, 'output_traj'+str(i), 'a')

					raw_input("Press enter to continue...")

	#visualization functions
	def ExecuteTrajectory(self, trajToExecute):
		print '---------------EXECUTING TRAJECTORY---------------'
		with self.env:
		  traj = RaveCreateTrajectory(self.env,'')
		  spec = IkParameterization.GetConfigurationSpecificationFromType(IkParameterizationType.Transform6D,'linear')

		  #print self.robot.GetActiveConfigurationSpecification()

		  traj.Init(self.robot.GetActiveConfigurationSpecification());

		  for traj_pts in trajToExecute:
			traj.Insert(traj.GetNumWaypoints(), traj_pts)

		#don't execute yet
		#   planningutils.RetimeActiveDOFTrajectory(traj, self.robot, hastimestamps=False, maxvelmult=0.01, maxaccelmult=0.01, plannername='LinearTrajectoryRetimer')
		
		#   self.robot.GetController().SetPath(traj)
		# self.robot.WaitForController(0)

	def MoveManip(self, manip, joint_target, Locked = False):
		if (Locked == False):
			with self.env:
				self.robot.SetActiveDOFs(manip.GetArmIndices())
				self.robot.SetActiveDOFValues(joint_target)
		else: 
			self.robot.SetActiveDOFs(manip.GetArmIndices())
			self.robot.SetActiveDOFValues(joint_target)

	def VisualizeTraj(self, trajectory, manip, orig_traj = None, isGreen = True):
		print "Visualizing trajectory", type(trajectory[0])
		g_pose 			= self.goggles.GetTransform()
		ee_transform 	= manip.GetEndEffectorTransform()
		self.robot.SetActiveManipulator(manip)
		prev_g_pose = np.copy(g_pose)
		idx = 0
		prev_ee_poses = []
		for joint_vals in trajectory:
			print joint_vals
			new_ee_trans = []
			with self.env:
				self.robot.SetActiveDOFs(manip.GetArmIndices())
				self.robot.SetActiveDOFValues(joint_vals)

			# new_ee_trans 		= manip.GetEndEffector().GetTransform()
			# pos_diff 			= self.utils.SubVec(new_ee_trans[0:3,3], ee_transform[0:3,3])
			# new_g_pose 			= np.copy(g_pose)
			# new_g_pose[0:3,3] 	= self.utils.AddVec(g_pose[0:3,3], pos_diff)

			# self.DrawLocation(new_ee_trans, np.array([0.0, 1.0, 0.0]))

			# if (idx > 0):
			# 	from_vec = prev_ee_poses[idx-1][0:3,3]
			# 	to_vec 	 = new_ee_trans[0:3,3]
			# 	self.draw_handles.append(self.env.drawarrow(from_vec, to_vec, 0.0005, [1, 0, 0]))

			# prev_ee_poses.append(new_ee_trans)
			# prev_g_pose = np.copy(self.goggles.GetTransform())
			# idx += 1
			raw_input("Press enter to continue...")

	#drawing functions
	def DrawPoint(self, point, rgb):
		self.draw_handles.append(self.env.plot3(points=array(((point.position.x, point.position.y, point.position.z))),
												pointsize=5.0,
												colors=array(((0,0,1)))))

	def DrawLocation(self, transform ,rgb):
		if (np.shape(transform) == (4,4)):
			self.draw_handles.append(self.env.plot3(transform[0:3,3],
											   pointsize=5.0,
											   colors=rgb))
		elif (np.shape(transform) == (4,)):
			self.draw_handles.append(self.env.plot3(transform[0:3],
											   pointsize=5.0,
											   colors=rgb))
		elif (np.shape(transform) == (3,)):
			self.draw_handles.append(self.env.plot3(transform,
											   pointsize=5.0,
											   colors=rgb))

	def DrawOrientation(self, transform, location = None):
		if (np.shape(transform) == (4,4) or np.shape(transform) == (3,3)):
			from_vec = []
			if (location == None):
				from_vec = transform[0:3,3]
			elif (type(location) == geometry_msgs.msg._Point.Point):
				from_vec = self.utils.PointToArray(location.position)

			to_vec_1 = from_vec + 0.05*(transform[0:3,0])
			to_vec_2 = from_vec + 0.05*(transform[0:3,1])
			to_vec_3 = from_vec + 0.05*(transform[0:3,2])

			self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_1, 0.002, [1, 0, 0]))
			self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_2, 0.002, [0, 1, 0]))
			self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_3, 0.002, [0, 0, 1]))

	#generating reaching motion
	def GenerateReachingMotion(self, pose_target, pose_xyz_target, pose_rot_target, manip, n_steps):
		print manip.GetName()
		with self.env:
			self.robot.SetActiveManipulator(manip)
			self.robot.SetActiveDOFs(manip.GetArmIndices())
			start_config = self.robot.GetActiveDOFValues()
		end_configs = self.GetIKSolutions(pose_target, manip)

		trajectories = []
		if (len(end_configs) > 0):
			
			manip_name = manip.GetName()

			print start_config
			print end_configs[0]
			raw_input("press enter to set to end")
			with self.env:
				self.robot.SetActiveDOFs(manip.GetArmIndices())
				self.robot.SetActiveDOFValues(end_configs[0])
			raw_input("press enter to set to start")
			with self.env:
				self.robot.SetActiveDOFs(manip.GetArmIndices())
				self.robot.SetActiveDOFValues(start_config)
			request = self.GenerateTrajoptRequestNoPose(n_steps, manip_name, start_config, pose_xyz_target, pose_rot_target, end_configs[0], None)
			self.DrawLocation(pose_target, [0.5, 0, 0.5])

			s = json.dumps(request) # convert dictionary into json-formatted string
			prob = None 

			cost_handles = []
			self.costwrapper.a_manip = manip
			with self.env:	
				prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem

				for t in range(0,n_steps):
					prob.AddCost(self.costwrapper.ManipulabilityCost, [(t,j) for j in xrange(7)], "ABS")#, "up%i"%t)
					prob.AddCost(self.costwrapper.LegibilityCost, [(t,j) for j in xrange(7)], "ABS")
					#add other costs here

				t_start = time.time()

			raw_input("press enter to solve")

			try:
				with self.env:
					result = trajoptpy.OptimizeProblem(prob) # do optimization
					t_elapsed = time.time() - t_start
					trajectories.append(result.GetTraj())
					print "optimization took %.3f seconds"%t_elapsed
					raw_input("Press enter to continue...")
			except:
				print "Unexpected error:", sys.exc_info()[0]

			for traj in trajectories:
				self.VisualizeTraj(traj, self.r_manip)
				raw_input("Press enter to continue...")

	def GenerateReachingForTransfer(self, manip):
		num_valid_right = len(self.valid_transfer_starts_right)
		num_valid_left 	= len(self.valid_transfer_starts_left)

		if (manip == self.r_manip):
			for i in range (0, num_valid_right): 
				start_xyz_target = self.utils.PointToArray(self.valid_transfer_starts_right[i].position)
				start_rot_target = self.utils.QuatToArray(self.valid_transfer_starts_right[i].orientation)
				start_t_target 	= openravepy.matrixFromPose(np.r_[start_rot_target, start_xyz_target])
				self.GenerateReachingMotion(start_t_target, start_xyz_target, start_rot_target, manip, 30)
				raw_input("Press enter to continue...")
		else:
			for i in range (0, num_valid_left): 
				start_xyz_target = self.utils.PointToArray(self.valid_transfer_starts_left[i].position)
				start_rot_target = self.utils.QuatToArray(self.valid_transfer_starts_left[i].orientation)
				start_t_target 	= openravepy.matrixFromPose(np.r_[start_rot_target, start_xyz_target])
				self.GenerateReachingMotion(start_t_target, start_xyz_target, start_rot_target, manip, 30)
				raw_input("Press enter to continue...")

	#body transform functions
	def CacheBodyTransforms(self):
		del self.cachedBodyLocations[:]
		links = self.human.GetLinks()
		for link in links:
			geoms = link.GetGeometries()
			self.cachedBodyTransforms.append(tuple([link.GetName(), link.GetName(), link.GetTransform()]))
			self.cachedBodyLocations.append(tuple([link.GetName(), link.GetName(), link.GetTransform()[0:3,3]]))
			idx = 0
			for geom in geoms:
				geom_trans = link.GetTransform() * geom.GetTransform()
				self.cachedBodyTransforms.append(tuple([link.GetName(), link.GetName()+'_'+str(idx), geom_trans]))
				self.cachedBodyLocations.append(tuple([link.GetName(), link.GetName()+'_'+str(idx), geom_trans[0:3,3]]))
				idx += 1

	def AppendBodyTransforms(self):
		body_transforms = []
		body_locations = []
		links = self.human.GetLinks()
		for link in links:
			geoms = link.GetGeometries()
			body_transforms.append(tuple([link.GetName(), link.GetName(), link.GetTransform()]))
			body_locations.append(tuple([link.GetName(), link.GetName(), link.GetTransform()[0:3,3]]))
			idx = 0
			for geom in geoms:
				geom_trans = link.GetTransform() * geom.GetTransform()
				body_transforms.append(tuple([link.GetName(), link.GetName()+'_'+str(idx), geom_trans]))
				body_locations.append(tuple([link.GetName(), link.GetName()+'_'+str(idx), geom_trans[0:3,3]]))
				idx += 1

		self.motionBodyTransforms.append(body_transforms)
		self.motionBodyLocations.append(body_locations)

	def MoveHumanUpperSingle(self, r_arm_angles = None, l_arm_angles = None, torso_angles = None):
		if (self.human == None or self.env == None or (r_arm_angles == None and l_arm_angles == None and torso_angles == None)):
			return
		right_joint_names = ['rShoulderX', 'rShoulderZ', 'rShoulderY', 'rElbowZ', 'rWristX', 'rWristY', 'rWristZ']
		left_joint_names = ['lShoulderX', 'lShoulderZ', 'lShoulderY', 'lElbowZ', 'lWristX', 'lWristY', 'lWristZ']
		torso_joint_names = ['PelvisRotX', 'PelvisRotY', 'PelvisRotZ', 'TorsoX', 'TorsoZ', 'TorsoY', 'HeadZ', 'HeadY', 'HeadX']

		if (r_arm_angles != None):
			with self.env:
				self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in right_joint_names])
				self.human.SetActiveDOFValues(r_arm_angles);             
			waitrobot(self.human)    
			#self.human.GetController().SetDesired(self.human.GetDOFValues());
		if (l_arm_angles != None):
			with self.env:
				self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in left_joint_names])
				self.human.SetActiveDOFValues(l_arm_angles);                 
			waitrobot(self.human)
			#self.human.GetController().SetDesired(self.human.GetDOFValues());
		if (torso_angles != None):
			with self.env:
				self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in torso_joint_names])
				self.human.SetActiveDOFValues(torso_angles);         
			waitrobot(self.human)
			#self.human.GetController().SetDesired(self.human.GetDOFValues());
		time.sleep(1)

		#len right_arm_angles arr = 7
		#len left_arm_angles arr = 7
		#len torso_arm_angles arr = 9
		self.AppendBodyTransforms()

	def move_human_to_neutral(self):
		right_angles = [0, 0, 0, 0, 0, 0, 0]
		left_angles  = [0, 0, 0, 0, 0, 0, 0]
		torso_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0]

		self.MoveHumanUpperSingle(right_angles, left_angles)

	#goggles functions
	def ExtendGoggles(self):
		if (self.back_strap_right != None):
			with self.env:
				# #left
				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-4, len(self.robot.GetJoints())-3,
										  len(self.robot.GetJoints())-2, len(self.robot.GetJoints())-1])
				self.robot.SetActiveDOFValues([0, 0, 0, 0])	


				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-5])
				self.robot.SetActiveDOFValues([self.max_val])	

				#right
				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-9, len(self.robot.GetJoints())-8,
										  len(self.robot.GetJoints())-7, len(self.robot.GetJoints())-6])
				self.robot.SetActiveDOFValues([0, 0, 0, 0])	


				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-10])
				self.robot.SetActiveDOFValues([self.max_val])	

	def CollapseGoggles(self):
		if (self.back_strap_right != None):
			with self.env:
				#left
				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-5])
				self.robot.SetActiveDOFValues([0])	
				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-4, len(self.robot.GetJoints())-3,
										  len(self.robot.GetJoints())-2, len(self.robot.GetJoints())-1])
				self.robot.SetActiveDOFValues([np.pi * 0.5, -np.pi * 0.5, -np.pi * 0.5, np.pi * 0.5])	

				#right
				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-10])
				self.robot.SetActiveDOFValues([0])	
				self.robot.SetActiveDOFs([len(self.robot.GetJoints())-9, len(self.robot.GetJoints())-8,
										  len(self.robot.GetJoints())-7, len(self.robot.GetJoints())-6])
				self.robot.SetActiveDOFValues([np.pi * 0.5, -np.pi * 0.5, -np.pi * 0.5, np.pi * 0.5])	

	#path refinemant functions
	def ShortcutSmoothing(self, traj, manip, total_waypoints):
		cur_traj = np.copy(traj)
		critical_indices = []
		for waypoint in total_waypoints:
			critical_indices.append(waypoint[1])

		for i in range(0, 200):
			# short-range shortcutting
			rand_start = np.random.randint(0, len(cur_traj) - 3)
			rand_end =  rand_start + 3#np.random.randint(0, len(cur_traj))

			# check if a critical waypoint falls in between
			invalid = False
			for index in critical_indices:
				if (rand_start < index and index < rand_end):
					invalid = True
					break

			if (invalid):
				continue

			# if (rand_start > rand_end):
			# 	temp = rand_start
			# 	rand_start = rand_end
			# 	rand_end = temp

			diff = cur_traj[rand_end] - cur_traj[rand_start]
			step = np.copy(diff)/5
			curr = np.copy(cur_traj[rand_start])
			shortcut = []
			for j in range(0, 5):
				curr += step;
				with self.env:
					self.robot.SetActiveDOFs(manip.GetArmIndices())
					self.robot.SetActiveDOFValues(curr)
				if (self.env.CheckCollision(self.robot)):
					break
				else:
					shortcut.append(np.copy(curr))
			new_traj = []
			if (np.linalg.norm(cur_traj[rand_end] - curr) < 1e-6):
				for j in range(0, rand_start):
					new_traj.append(np.copy(cur_traj[j]))
				new_traj.append(np.copy(cur_traj[rand_start]))
				for point in shortcut:
					new_traj.append(np.copy(point))
				for j in range(rand_end+1, len(cur_traj)):
					new_traj.append(np.copy(cur_traj[j]))
				cur_traj = new_traj
		
		with self.env:
			self.robot.SetActiveDOFs(manip.GetArmIndices())
			self.robot.SetActiveDOFValues(traj[0])

		return cur_traj

	def ShortcutSmoothingTaskSpace(self, traj):
		return

	#test functions
	def RelevantPathWaypointsTest(self):
		total_waypoints = self.GetRelevantPathWaypoints(self.transfer_path)
		for curve_waypoint in self.GetPathWaypointsCurvature(self.transfer_path):
			idx = curve_waypoint[1]
			appeared = False
			for t_waypoint in total_waypoints:
				if (t_waypoint[1] == idx):
					appeared = True
					break
			if (appeared == False):
				total_waypoints.append(curve_waypoint)

		print total_waypoints
	
	def TestInterp(self):
		traj = []
		sol_0 = [0.0, 0.0, 0.0, 0.0, 0.0]
		sol_n = [5.0, 5.0, 5.0, 5.0, 5.0]
		sol_diff = self.utils.SubArr(sol_n, sol_0)
		x_n = 5.0
		x_0 = 0.0
		x = 1.0
		while (x < x_n):
			scaler =  (x - x_0)/(x_n - x_0)
			print 'scaler',scaler
			print 'sol_diff',sol_diff
			print 'mult',self.utils.MultArr(sol_diff, scaler)
			new_sol = self.utils.AddArr(sol_0, self.utils.MultArr(sol_diff, scaler))
			x += 1.0
			traj.append(new_sol[:])

		print 'test'
		print traj

	#out-of-execution functions
	def WriteTrajectoryToFile(self, right_traj, left_traj, filename, mode):
		joint_names = ["left_s0","left_s1","left_e0","left_e1","left_w0","left_w1","left_w2","left_gripper",
					   "right_s0",'right_s1',"right_e0","right_e1","right_w0","right_w1","right_w2","right_gripper"]

		f = open(filename, mode)

		if (mode == 'w'):
			f.write('time,')
			for joint in joint_names:
				f.write(joint)
				f.write(',')
			f.write('\n')

		for i in range(0, len(right_traj)):
			for val in left_traj[i]:
				f.write(str(val))
				f.write(',')
			for j in range(1, len(right_traj[i])):
				f.write(str(right_traj[i][j]))
				if (j < len(right_traj[i])-1):
					f.write(',')
			f.write('\n')
		f.close()


