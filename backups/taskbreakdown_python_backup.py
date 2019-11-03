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

def waitrobot(robot):
	"""busy wait for robot completion"""
	while not robot.GetController().IsDone():
		time.sleep(0.01)
			
class TaskBreakdown:

	robot 		= None
	human 		= None
	env 		= None
	r_manip 	= None
	l_manip 	= None
	utils 		= None
	costwrapper = None

	original_path 			= []
	grid 					= []
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

	#geometry_msgs/Pose
	path_start_point 		= None
	transfer_path			= None
	support_path			= None
	valid_transfer_path		= None
	valid_support_path		= None

	#lists of geometry_msgs/Pose
	valid_transfer_starts_right 	= []
	valid_transfer_ends_right		= []
	#list of lists of numpy matrices
	valid_transfer_waypoints_right 	= []

	valid_transfer_starts_left		= []
	valid_transfer_ends_left		= []
	#list of lists of numpy matrices
	valid_transfer_waypoints_left 	= []

	#for drawing stuff with OpenRAVE
	draw_handles = []

	#grid
	cube_size = 0.05 #length of grid cube

	def Init(self, robot, env, human, handles = None):
		self.robot 		= robot
		self.env 		= env
		self.human 		= human
		self.utils 		= Utilities()
		self.r_manip 	= self.robot.SetActiveManipulator('rightarm')
		self.l_manip 	= self.robot.SetActiveManipulator('leftarm')

		if (handles != None):
			self.draw_handles = handles

		self.costwrapper = CostWrapper()
		self.costwrapper.Init(self.robot, self.env, self.human, self.utils, self.r_manip, self.l_manip)

	#service methods
	def ObtainGridPositionsService(self, start_point):
		print "waiting for grid positions service"
		rospy.wait_for_service('ObtainGridPositions')
		print "done waiting for grid positions service"

		try:
			grid_pos = rospy.ServiceProxy('ObtainGridPositions', ObtainGridPositions)
			
			#print "Requesting %s"%(start_point)
			resp1 = grid_pos(start_point)
			#print "Got",resp1.target_positions

			return resp1.target_positions

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def ObtainGridService(self):
		print "waiting for grid service"
		rospy.wait_for_service('ObtainGrid')
		print "done waiting for grid service"

		try:
			grid_pos = rospy.ServiceProxy('ObtainGrid', ObtainGrid)
			resp1 = grid_pos()

			return resp1.grid

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

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

	def MoveHumanUpper(self, r_arm_angles = None, l_arm_angles = None, torso_angles = None):
		if (self.human == None or self.env == None or (r_arm_angles == None and l_arm_angles == None and torso_angles == None)):
			return
		right_joint_names = ['rShoulderX', 'rShoulderZ', 'rShoulderY', 'rElbowZ', 'rWristX', 'rWristY', 'rWristZ']
		left_joint_names = ['lShoulderX', 'lShoulderZ', 'lShoulderY', 'lElbowZ', 'lWristX', 'lWristY', 'lWristZ']
		torso_joint_names = ['PelvisRotX', 'PelvisRotY', 'PelvisRotZ', 'TorsoX', 'TorsoZ', 'TorsoY', 'HeadZ', 'HeadY', 'HeadX']

		traj_len = 0;
		if (r_arm_angles != None):
			traj_len = len(r_arm_angles)
		elif (l_arm_angles != None):
			traj_len = len(l_arm_angles)
		elif (torso_angles != None):
			traj_len = len(torso_angles)

		for i in range(0, traj_len):
			if (r_arm_angles != None):
				with self.env:
					self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in right_joint_names])
					self.human.SetActiveDOFValues(r_arm_angles[i]);         
				#self.human.GetController().SetDesired(self.human.GetDOFValues());
			waitrobot(self.human)
			if (l_arm_angles != None):
				with self.env:
					self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in left_joint_names])
					self.human.SetActiveDOFValues(l_arm_angles[i]);         
				#self.human.GetController().SetDesired(self.human.GetDOFValues());
			waitrobot(self.human)
			if (torso_angles != None):
				with self.env:
					self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in torso_joint_names])
					self.human.SetActiveDOFValues(torso_angles[i]);         
				#self.human.GetController().SetDesired(self.human.GetDOFValues());
			waitrobot(self.human)
			time.sleep(1)

		#len right_arm_angles arr = 7
		#len left_arm_angles arr = 7
		#len torso_arm_angles arr = 9
		self.AppendBodyTransforms()

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

	def GetIKSolution(self, Tgoal, manip = None): #manip is OpenRAVE manip
		sol = None
		if (self.env == None or self.robot == None or manip == None):
			print('\n Error: Undefined env, robot, or manip \n')
			return sol

		with self.env:
			sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
			#print "solutions",sol
		return sol

	def GetIKSolutions(self, Tgoal, manip = None): #manip is OpenRAVE manip
		sol = None
		if (self.env == None or self.robot == None or manip == None):
			print('\n Error: Undefined env, robot, or manip \n')
			return sol

		with self.env:
			sol = manip.FindIKSolutions(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
			#print "solutions",sol
		return sol

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
		if (self.env == None):
			self.env = env
		self.ppePathTransforms = ppe_transform_path
		transfer_transforms = []

		mot_idx = 0
		sig_name = self.sortedBodyLocations[0][1] #name of most significant transform
		for ppe_trans in ppe_transform_path:
			motion_transforms = self.motionBodyTransforms[mot_idx]
			body_trans = None
			neutral_body_trans = None
			#find most significant transform
			for trans in motion_transforms:
				if (trans[1] == sig_name):
					body_trans = trans
					break
			for trans in self.cachedBodyTransforms:
				if (trans[1] == sig_name):
					neutral_body_trans = trans
					break
			
			body_to_world = np.linalg.inv(body_trans[2])
			body_to_ppe = np.dot(body_to_world, ppe_trans)
			transfer_trans = np.dot(neutral_body_trans[2], body_to_ppe)
			transfer_transforms.append(transfer_trans)

			self.draw_handles.append(env.plot3(points=transfer_trans[0:3,3],
									   pointsize=5.0,
									   colors=array(((0,0,1)))))

			from_vec = transfer_trans[0:3,3]

			to_vec_1 = from_vec + 0.05*(transfer_trans[0:3,0])
			to_vec_2 = from_vec + 0.05*(transfer_trans[0:3,1])
			to_vec_3 = from_vec + 0.05*(transfer_trans[0:3,2])

			# self.draw_handles.append(env.drawarrow(from_vec, to_vec_1, 0.005, [1, 0, 0]))
			# self.draw_handles.append(env.drawarrow(from_vec, to_vec_2, 0.005, [0, 1, 0]))
			# self.draw_handles.append(env.drawarrow(from_vec, to_vec_3, 0.005, [0, 0, 1]))

			mot_idx += 1
			self.transfer_path = transfer_transforms

	def PurifyIntoSupport(self, ppe_transform_path, env):
		if (self.env == None):
			self.env = env
		self.ppePathTransforms = ppe_transform_path
		support_transforms = []

		if (len(self.neutral_ppe_trans) == 0):
			self.neutral_ppe_trans = ppe_transform_path[0]

		mot_idx = 0
		sig_name = self.sortedBodyLocations[0][1]
		for ppe_trans in ppe_transform_path:
			motion_transforms = self.motionBodyTransforms[mot_idx]
			body_trans = None
			#find most significant transform
			for trans in motion_transforms:
				if (trans[1] == sig_name):
					body_trans = trans
					break

			new_ppe_trans = np.copy(ppe_trans)
			new_ppe_trans[0:3, 0:3] = np.identity(3)
			ppe_to_world = np.linalg.inv(new_ppe_trans)
			ppe_to_body = np.dot(ppe_to_world, body_trans[2])
			support_trans = np.dot(self.neutral_ppe_trans, ppe_to_body)
			support_transforms.append(support_trans)

			self.draw_handles.append(env.plot3(points=support_trans[0:3,3],
									   pointsize=5.0,
									   colors=array(((0,1,0)))))

			mot_idx += 1
			self.support_path = support_transforms

	def DistanceScore(self, p1, p2):
		p3 = self.utils.SubPoint(p1,p2)
		return 

	def AppendValidStartAndEnd(self, displacement_vec, adjustment_quat):
		path_transform_start = self.ppePathTransforms[0]
		path_transform_end	 = self.ppePathTransforms[len(self.ppePathTransforms)-1]

		new_path_point_start = Pose()
		new_path_point_start.position = self.utils.AddPoint(self.utils.VecToPoint(path_transform_start[0:3,3]), displacement_vec)
		new_path_point_start.orientation = self.utils.MultiplyQuaternion(self.utils.VecToQuat(transformations.quaternion_from_matrix(path_transform_start[0:3, 0:3])), adjustment_quat)
		T_target_start = transformations.quaternion_translation_matrix(self.utils.QuatToArray(new_path_point_start.orientation), self.utils.PointToArray(new_path_point_start.position))
		
		new_path_point_end = Pose()
		new_path_point_end.position = self.utils.AddPoint(self.utils.VecToPoint(path_transform_end[0:3,3]), displacement_vec)
		new_path_point_end.orientation = self.utils.MultiplyQuaternion(self.utils.VecToQuat(transformations.quaternion_from_matrix(path_transform_end[0:3, 0:3])), adjustment_quat)
		T_target_end = transformations.quaternion_translation_matrix(self.utils.QuatToArray(new_path_point_end.orientation), self.utils.PointToArray(new_path_point_end.position))
		
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
			self.valid_transfer_ends_right.append(new_path_point_end)

			self.draw_handles.append(self.env.plot3(T_target_start[0:3,3],
											   pointsize=15.0,
											   colors=array(((1,0,0)))))
			self.draw_handles.append(self.env.plot3(T_target_end[0:3,3],
											   pointsize=15.0,
											   colors=array(((1,0,0)))))

			valid_transfer_waypoints = []
			for ppeTrans in self.ppePathTransforms:		
				new_path_point = Pose()
				new_path_point.position = self.utils.AddPoint(self.utils.VecToPoint(ppeTrans[0:3,3]), displacement_vec)
				new_path_point.orientation = self.utils.MultiplyQuaternion(self.utils.VecToQuat(transformations.quaternion_from_matrix(ppeTrans[0:3, 0:3])), adjustment_quat)
				T_target = transformations.quaternion_translation_matrix(self.utils.QuatToArray(new_path_point.orientation), self.utils.PointToArray(new_path_point.position))
				valid_transfer_waypoints.append(T_target)
			self.valid_transfer_waypoints_right.append(valid_transfer_waypoints)
			hasSol = True

		if (l_start_sol != None and l_end_sol != None and len(l_start_sol) > 0 and len(l_end_sol) > 0):
			print ('Append Valid Start And End For Left')
			self.valid_transfer_starts_left.append(new_path_point_start)
			self.valid_transfer_ends_left.append(new_path_point_end)

			self.draw_handles.append(self.env.plot3(T_target_start[0:3,3],
											   pointsize=15.0,
											   colors=array(((1,0,0)))))
			self.draw_handles.append(self.env.plot3(T_target_end[0:3,3],
											   pointsize=15.0,
											   colors=array(((1,0,0)))))

			valid_transfer_waypoints = []
			for ppeTrans in self.ppePathTransforms:		
				new_path_point = Pose()
				new_path_point.position = self.utils.AddPoint(self.utils.VecToPoint(ppeTrans[0:3,3]), displacement_vec)
				new_path_point.orientation = self.utils.MultiplyQuaternion(self.utils.VecToQuat(transformations.quaternion_from_matrix(ppeTrans[0:3, 0:3])), adjustment_quat)
				T_target = transformations.quaternion_translation_matrix(self.utils.QuatToArray(new_path_point.orientation), self.utils.PointToArray(new_path_point.position))
				valid_transfer_waypoints.append(T_target)
			self.valid_transfer_waypoints_left.append(valid_transfer_waypoints)

		return hasSol

	def FindBestDisplacement(self):
		#path = vector of geometry_msgs/Pose
		#gridPositions = vector of geometry_msgs/Point
		#assume person is in neutral orientation, i.e. facing the robot with parallel x-axes
		disp = None
		if (self.env == None or self.robot == None or len(self.ppePathTransforms) <= 0 or len(self.gridPositions) <= 0):
			print('\n Undefined env or robot, length of path or grid positions must be greater than 0 \n')
			return disp

		self.original_path = self.ppePathTransforms
		self.gridPositions = self.gridPositions

		num_pos = len(self.gridPositions)
		num_path = len(self.ppePathTransforms)
		path_start_vec = self.ppePathTransforms[0][0:3,3]  
		path_start_rotmat = self.ppePathTransforms[0][0:3, 0:3]

		path_start_pos = self.utils.VecToPoint(path_start_vec)
		path_start_orient = self.utils.VecToQuat(transformations.quaternion_from_matrix(path_start_rotmat))

		person_axis = numpy.array([0,0,1])
		person_angle = numpy.pi
		person_quat_array = transformations.quaternion_about_axis(person_angle, person_axis)
		person_quat = self.utils.ArrayToQuat(person_quat_array)

		#return values
		best_displacement = None
		best_path = None
		best_sols = None
		best_arm = None

		print ('path_start_pos\n', path_start_pos)
		print ('path_start_orient\n', path_start_orient)

		orig_human_transform = self.human.GetTransform()

		angle = 0
		for a in range(0, 3):
			if (a == 1):
				angle = -numpy.pi * 0.25
			elif (a == 2):
				angle = numpy.pi * 0.25

			#person orientation
			axis = numpy.array([0, 0, 1]) #always rotate about z axis
			quat_array = transformations.quaternion_about_axis(angle, axis) #np_array
			adjustment_quat = self.utils.ArrayToQuat(quat_array) #geometry_msgs/Quaternion
			new_person_quat = self.utils.MultiplyQuaternion(person_quat, adjustment_quat) #geometry_msgs/Quaternion
			new_person_quat_array = self.utils.QuatToArray(new_person_quat) #np_array
			new_person_axis_angle = transformations.quaternion_to_axis(new_person_quat_array) #angle_axis tuple
			new_person_angle = new_person_axis_angle[0] #double

			#angle_transform = transformations.quaternion_matrix(quat_array) #4x4 numpy array
			angle_transform = transformations.quaternion_matrix(new_person_quat_array) #4x4 numpy array

			self.move_human_to_neutral()

			for i in range(0, num_pos):

				# if (i != 1873 or i != 1933):
				# 	continue

				grid_pos = self.gridPositions[i] 
				#print ('grid_pos \n', grid_pos)

				displacement_vec = self.utils.SubPoint(grid_pos, path_start_pos) #geometry_msgs/Point
				#print ('displacement_vec \n', displacement_vec)

				#adjust displacement according to person's angle -> person is facing in 180 about z by default
				#project displacement onto x-axis first, then rotate by the given person angle (orientation) -> this assumes path is in straight line from person
				#displacement_vec[0] = vector_norm(PointToArray(displacement_vec)) * np.cos(new_person_axis_angle)
				#displacement_vec[1] = vector_norm(PointToArray(displacement_vec)) * np.sin(new_person_axis_angle)
				#displacement_vec[2] = 0

				new_displacement_vec = self.utils.RotateVector(adjustment_quat, displacement_vec)

				r_valid_count = 0
				l_valid_count = 0
				success = False

				human_transform = np.copy(orig_human_transform)
				# human_transform[0,3] += new_displacement_vec.x
				# human_transform[1,3] += new_displacement_vec.y
				# human_transform[2,3] += new_displacement_vec.z
				human_transform[0:3, 0:3] = angle_transform[0:3, 0:3]
				human_transform[0,3] += displacement_vec.x
				human_transform[1,3] += displacement_vec.y
				human_transform[2,3] += displacement_vec.z

				if (human_transform[0,3] <= self.robot.GetTransform()[0,3]):
					continue

				with self.env:
					self.human.SetTransform(human_transform)

				if (self.env.CheckCollision(self.robot,self.human)):
					continue

				new_path = []  #array of geometry_msgs/Pose


				# for path_transform in self.ppePathTransforms:
				# 	path_point_loc = self.utils.VecToPoint(path_transform[0:3,3])
				# 	path_point_orient = transformations.quaternion_from_matrix(path_transform[0:3, 0:3])
				# 	path_point = Pose()
				# 	path_point.position = path_point_loc
				# 	path_point.orientation = self.utils.VecToQuat(path_point_orient)

				# 	new_path_point = path_point;
				# 	new_path_point.position = self.utils.AddPoint(new_path_point.position, displacement_vec)

				# 	new_path_point.orientation = self.utils.MultiplyQuaternion(new_path_point.orientation, adjustment_quat)

				# 	new_path.append(new_path_point)

				# 	T_target = transformations.quaternion_translation_matrix(self.utils.QuatToArray(new_path_point.orientation), self.utils.PointToArray(new_path_point.position))
					

				# 	self.draw_handles.append(self.env.plot3(T_target[0:3,3],
				# 									   pointsize=5.0,
				# 									   colors=array(((1,0,0)))))
				# 	from_vec = T_target[0:3,3]

				# 	to_vec_1 = from_vec + 0.05*(T_target[0:3,0])
				# 	to_vec_2 = from_vec + 0.05*(T_target[0:3,1])
				# 	to_vec_3 = from_vec + 0.05*(T_target[0:3,2])

				# 	self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_1, 0.005, [1, 0, 0]))
				# 	self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_2, 0.005, [0, 1, 0]))
				# 	self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_3, 0.005, [0, 0, 1]))

				if (self.AppendValidStartAndEnd(new_displacement_vec, adjustment_quat)):
					print i

				pidx = 0
				for path_transform in self.ppePathTransforms:
					# if (pidx > 0):
					# 	break
					path_point_loc = self.utils.VecToPoint(path_transform[0:3,3])
					path_point_orient = transformations.quaternion_from_matrix(path_transform[0:3, 0:3])
					new_path_point = Pose()
					new_path_point.position = self.utils.AddPoint(path_point_loc, displacement_vec)
					new_path_point.orientation = self.utils.MultiplyQuaternion(self.utils.VecToQuat(path_point_orient), adjustment_quat)
					new_path.append(new_path_point)

					T_target = transformations.quaternion_translation_matrix(self.utils.QuatToArray(new_path_point.orientation), self.utils.PointToArray(new_path_point.position))
					# print ('T_target is \n', T_target)

					r_sols = self.GetIKSolutions(T_target, self.r_manip)
					l_sols = self.GetIKSolutions(T_target, self.l_manip)

					has_sol = False
					if (r_sols != None and len(r_sols) > 0):
						r_valid_count += 1
						has_sol = True

					if (l_sols != None and len(l_sols) > 0):
						l_valid_count += 1
						has_sol = True

					if (has_sol == False):
						# self.draw_handles.append(self.env.plot3(T_target[0:3,3],
						# 								   pointsize=5.0,
						# 								   colors=array(((1,0,0)))))

						break
					else:
						# self.draw_handles.append(self.env.plot3(T_target[0:3,3],
						# 								   pointsize=5.0,
						# 								   colors=array(((0,1,0)))))
						from_vec = T_target[0:3,3]

						# to_vec_1 = from_vec + 0.05*(T_target[0:3,0])
						# to_vec_2 = from_vec + 0.05*(T_target[0:3,1])
						# to_vec_3 = from_vec + 0.05*(T_target[0:3,2])

						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_1, 0.005, [1, 0, 0]))
						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_2, 0.005, [0, 1, 0]))
						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_3, 0.005, [0, 0, 1]))

					pidx += 1

				if (r_valid_count == num_path): #add scoring here to pick right over left or vice versa
					best_displacement = displacement_vec
					best_path = new_path
					best_sols = r_sols
					best_arm = 'rightarm'

					self.valid_transfer_path = []
					for path_point in new_path:
						# self.draw_handles.append(self.env.plot3(points=array(((path_point.position.x, path_point.position.y, path_point.position.z))),
						# 								   pointsize=5.0,
						# 								   colors=array(((0,0,1)))))

						T_rot = transformations.quaternion_matrix(self.utils.QuatToArray(path_point.orientation))
						from_vec = self.utils.PointToArray(path_point.position)

						to_vec_1 = from_vec + 0.05*(T_rot[0:3,0])
						to_vec_2 = from_vec + 0.05*(T_rot[0:3,1])
						to_vec_3 = from_vec + 0.05*(T_rot[0:3,2])

						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_1, 0.002, [1, 0, 0]))
						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_2, 0.002, [0, 1, 0]))
						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_3, 0.002, [0, 0, 1]))

						T_mat = np.copy(T_rot)
						T_mat[0:3,3] = [path_point.position.x, path_point.position.y, path_point.position.z]
						self.valid_transfer_path.append(T_mat)


					# self.valid_transfer_starts_right.append(new_path[0])
					# self.valid_transfer_ends_right.append(new_path[len(new_path)-1])

					print ('for right, best displacement \n', displacement_vec)
					print i
					return #pick first solution only for testing

				if (l_valid_count == num_path): #add scoring here to pick right over left or vice versa
					best_displacement = displacement_vec
					best_path = new_path
					best_sols = l_sols
					best_arm = 'leftarm'

					self.valid_transfer_path = []
					for path_point in new_path:
						# self.draw_handles.append(self.env.plot3(points=array(((path_point.position.x, path_point.position.y, path_point.position.z))),
						# 								   pointsize=5.0,
						# 								   colors=array(((0,0,1)))))

						T_rot = transformations.quaternion_matrix(self.utils.QuatToArray(path_point.orientation))
						from_vec = self.utils.PointToArray(path_point.position)

						to_vec_1 = from_vec + 0.05*(T_rot[0:3,0])
						to_vec_2 = from_vec + 0.05*(T_rot[0:3,1])
						to_vec_3 = from_vec + 0.05*(T_rot[0:3,2])

						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_1, 0.002, [1, 0, 0]))
						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_2, 0.002, [0, 1, 0]))
						# self.draw_handles.append(self.env.drawarrow(from_vec, to_vec_3, 0.002, [0, 0, 1]))

						T_mat = np.copy(T_rot)
						T_mat[0:3,3] = [path_point.position.x, path_point.position.y, path_point.position.z]
						self.valid_transfer_path.append(T_mat)

					# self.valid_transfer_starts_right.append(new_path[0])
					# self.valid_transfer_ends_right.append(new_path[len(new_path)-1])

					print ('for left, best displacement \n', displacement_vec)
					return #pick first solution only for testing


		print ('Returning best displacement')
		return tuple([best_displacement, best_path, best_sols, best_arm])

	def ObtainGridPositions(self, env):
		if (len(self.ppePathTransforms) == 0 and len(self.ppePathLocations) == 0):
			print ('\n Error: Path does not exist')
			return False

		if (self.path_start_point == None):
			self.path_start_point = self.ppePathTransforms[0][0:3,3]

		start_point = Point()
		start_point.x = self.path_start_point[0]
		start_point.y = self.path_start_point[1]
		start_point.z = self.path_start_point[2]

		self.gridPositions = self.ObtainGridPositionsService(start_point)

		# for pos in self.gridPositions:
		# 	self.draw_handles.append(env.plot3(points=array(((pos.x, pos.y, pos.z))),
		# 							   pointsize=5.0,
		# 							   colors=array(((1,1,0)))))

	def ObtainGrid(self):
		self.grid = self.ObtainGridService()

	#search whole grid
	def FindValidTransferEndpoints(self):
		if (self.transfer_path == None):
			print ('\n Error: Transfer path does not exist')
			return 
		if (len(self.grid) == 0):
			print ('\n Error: Grid does not exist')
			return 

	def ConstructInitTransferTraj(self, start = None, end = None):
		new_transfer_traj = []
		if (start == None or end == None):
			print ('No start or end transforms')
			return

		idx = 0
		displacement 	= self.utils.SubPoint(self.transfer_path[0], start)
		adjustment_quat = self.utils.QuaternionFromTo(self.transfer_path[0].orientation)

		for transfer_traj_pose in self.transfer_path:
			new_transfer_traj_pose = Pose()
			if (idx == 0):
				new_transfer_traj_pose = self.utils.CopyPose(start)
			elif (idx == len(self.transfer_path) - 1):
				new_transfer_traj_pose = self.utils.CopyPose(end)
			else:
				new_transfer_traj_pose.position 	= self.utils.AddPoint(transfer_traj_pose.position, displacement)
				new_transfer_traj_pose.orientation 	= self.utils.MultiplyQuaternion(transfer_traj_pose.orientation, adjustment_quat)

			new_transfer_traj.append(new_transfer_traj_pose)

		return new_transfer_traj

	def GenerateTraj(self, pose_waypoints, manip):
		traj = []
		for pose in pose_waypoints:
			sol = self.GetIKSolutions(pose, manip)
			if (len(sol) > 0):
				traj.append(sol[0].tolist())

		if (len(traj) >= 2):
			return traj

		else:
			print ('Invalid trajectory: has less than 2 configuration waypoints')
			return None

	def CalcSlope(self, p1, p2, p3): #caclulate slope
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

		p1p2mag = self.utils.VecMag(self.utils.SubVec(p2_arr, p1_arr))
		p2p3mag = self.utils.VecMag(self.utils.SubVec(p3_arr, p2_arr))

		p1p2xy_slope = (p2_arr[2] - p1_arr[2])/p1p2mag
		p1p2xz_slope = (p2_arr[1] - p1_arr[1])/p1p2mag
		p1p2yz_slope = (p2_arr[0] - p1_arr[0])/p1p2mag

		p2p3xy_slope = (p3_arr[2] - p2_arr[2])/p2p3mag
		p2p3xz_slope = (p3_arr[1] - p2_arr[1])/p2p3mag
		p2p3yz_slope = (p3_arr[0] - p2_arr[0])/p2p3mag

		return [p1p2xy_slope, p1p2xz_slope, p1p2yz_slope], [p2p3xy_slope, p2p3xz_slope, p2p3yz_slope]

	def EvalSlopeDir(self, p1, p2, p3): #returns if there is a sign change in the direction vector from 
		p1p2dir, p2p3dir = self.CalcSlope(p1, p2, p3)
		if (np.sign(p1p2dir[0]) != np.sign(p2p3dir[0]) and np.fabs(p1p2dir[0] - p2p3dir[0]) > 1e-4):
			# print 'Direction vector from p1 to p2', p1p2dir
			# print 'Direction vector from p2 to p3', p2p3dir
			# print ('significant direction change')
			return True
		elif (np.sign(p1p2dir[1]) != np.sign(p2p3dir[1]) and np.fabs(p1p2dir[1] - p2p3dir[1]) > 1e-4):
			# print 'Direction vector from p1 to p2', p1p2dir
			# print 'Direction vector from p2 to p3', p2p3dir
			# print ('significant direction change')
			return True 
		elif (np.sign(p1p2dir[2]) != np.sign(p2p3dir[2]) and np.fabs(p1p2dir[2] - p2p3dir[2]) > 1e-4):
			# print 'Direction vector from p1 to p2', p1p2dir
			# print 'Direction vector from p2 to p3', p2p3dir
			# print ('significant direction change')
			return True
		return False

	def GetRelevantPathWaypoints(self, pose_waypoints): #retrieve points where slope signs or concavity change
		relevant_waypoints = []

		if (len(pose_waypoints) < 3):
			return relevant_waypoints

		for i in range(1, len(pose_waypoints) - 2): #don't include start and end points in calculation curvature at a given point
			before 	= pose_waypoints[i-1][:]
			current = pose_waypoints[i][:]
			after 	= pose_waypoints[i+1][:]

			if (self.EvalSlopeDir(before, current, after) == True):
				relevant_waypoints.append(tuple([current, i]))
				# self.draw_handles.append(self.env.plot3(current[0:3,3],
				# 								   pointsize=5.0,
				# 								   colors=array(((0.7,0.3,0.2)))))

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

		o_angle = np.arccos(np.dot(p1p2, p2p3)/(p1p2mag * p2p3mag))
		angle = np.arctan2(np.sin(o_angle), np.cos(o_angle))

		m_angle = np.arctan2(np.linalg.norm(np.cross(p1p2,p2p3)), np.dot(p1p2, p2p3))
		# print 'p1 to p2', p1p2
		# print 'p2 to p3', p2p3
		# print 'has angle', angle * 180 / np.pi 
		# print 'has o_angle', o_angle * 180 / np.pi 
		# print 'has m_angle', m_angle * 180 / np.pi 

		# if (180 - np.fabs(m_angle * 180 / np.pi) > 15):
			# print 'p1 to p2', p1p2
			# print 'p2 to p3', p2p3
			# print 'has angle', angle * 180 / np.pi 
			# print 'has o_angle', o_angle * 180 / np.pi 

			# self.draw_handles.append(self.env.drawarrow(p2_arr, p1_arr, 0.001, [1, 0, 0]))
			# self.draw_handles.append(self.env.drawarrow(p2_arr, p3_arr, 0.001, [0, 1, 0]))

		return m_angle * 180 / np.pi

	def CalcContCurvature(self, a, b, c, p):
		return None

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
				# self.draw_handles.append(self.env.plot3(current[0:3,3],
				# 								   pointsize=5.0,
				# 								   colors=array(((0.3,0.7,0.2)))))

		return relevant_waypoints

	def GenerateTrajoptRequest(self, end_xyz_target, end_rot_target, n_steps, manip_name, cspace_waypoints, init_joint_target):
		request = {
		  "basic_info" : {
			"n_steps" : n_steps, #num points of init traj
			"manip" : manip_name, # see below for valid values
			"start_fixed" : False #True # i.e., DOF values at first timestep are fixed based on current robot state
		  },
		  "costs" : [
		  {
			"type" : "joint_vel", # joint-space velocity cost
			"params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
		  },
		  {
			"type" : "collision",
			"name" :"cont_coll",
			"params" : {
			  "continuous" : True,
			  "coeffs" : [1000], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			},    
		  }
		  ],
		  "constraints" : [
		  {
			"type" : "pose", 
			"params" : {"xyz" : end_xyz_target.tolist(), 
						"wxyz" : end_rot_target, 
						"link": "right_hand",
						"timestep" : n_steps-1 #-1 of n steps
						}
						 
		  }
		  ],
		  "init_info" : {
			  # "type" : "given_traj",
			  # "data" : cspace_waypoints
			  "type" : "straight_line", # straight line in joint space.
			  "endpoint" : init_joint_target.tolist()
		  }
		}
		
		return request

	def ExtendRequestForGoggles(self, request, waypoints, linkname):
		for waypoint in waypoints:
			# print 'waypoint'
			# print waypoint[0][0:3,3].tolist()
			# print transformations.quaternion_from_matrix(waypoint[0][0:3,0:3]).tolist()
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
				
		print 'new request:'
		print request

		return request

	def TrajoptTransferPath(self):
		if (self.transfer_path == None or len(self.transfer_path) == 0):
			print ('\n Error: Transfer path does not exist')
			return 

		print ('Trajopt-ing transfer path')
			# same number of transfer starts and ends (guaranteed)
		num_valid_right = len(self.valid_transfer_starts_right)
		num_valid_left 	= len(self.valid_transfer_starts_left)

		trajectories = []
		for i in range (0, num_valid_right): 
			print 'iteration:',i
			trajoptpy.SetInteractive(False) # pause every iteration, until you press 'p'. Press escape to disable further plotting
			time.sleep(0.1)

			start_xyz_target = self.utils.PointToArray(self.valid_transfer_starts_right[i].position)
			start_rot_target = self.utils.QuatToArray(self.valid_transfer_starts_right[i].orientation)

			end_xyz_target = self.utils.PointToArray(self.valid_transfer_ends_right[i].position)
			end_rot_target = self.utils.QuatToArray(self.valid_transfer_ends_right[i].orientation)

			start_t_target 	= openravepy.matrixFromPose(np.r_[start_rot_target, start_xyz_target])
			end_t_target 	= openravepy.matrixFromPose(np.r_[end_rot_target, end_xyz_target])

			r_sol = self.GetIKSolutions(start_t_target, self.r_manip)

			total_waypoints = self.GetRelevantPathWaypoints(self.valid_transfer_waypoints_right[i])[:]
			for curve_waypoint in self.GetPathWaypointsCurvature(self.valid_transfer_waypoints_right[i]):
				total_waypoints.append(curve_waypoint)

			self.draw_handles.append(self.env.plot3(start_xyz_target,
											   pointsize=15.0,
											   colors=array(((0.3,0.2,0.8)))))
			self.draw_handles.append(self.env.plot3(end_xyz_target,
											   pointsize=15.0,
											   colors=array(((0.3,0.2,0.8)))))

			# raw_input("Press enter to continue...")
			if (len(r_sol) > 0):
				self.MoveManip(self.r_manip, r_sol[0]) 
				# raw_input("Press enter to continue...")

				manip_name = self.r_manip.GetName()
				cspace_waypoints = self.GenerateTraj(self.valid_transfer_waypoints_right[i], self.r_manip)
				n_steps = len(self.valid_transfer_waypoints_right[i])

				#BEGIN IK for init joint_target
				init_joint_target = ku.ik_for_link(start_t_target, self.r_manip, "right_hand", filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
				#END IK

				print 'init_joint_target', init_joint_target
				request = self.GenerateTrajoptRequest(end_xyz_target, end_rot_target, n_steps, manip_name, cspace_waypoints, init_joint_target)
				#request = self.ExtendRequestForGoggles(request, total_waypoints, manip_name)

				s = json.dumps(request) # convert dictionary into json-formatted string
				prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem

				self.costwrapper.start_t_target = start_t_target
				self.costwrapper.end_t_target 	= end_t_target
				# for t in range(1,n_steps):
				# 	prob.AddConstraint(self.costwrapper.TestCost, [(t,j) for j in xrange(7)], "EQ", "up%i"%t)
				t_start = time.time()

				raw_input("Press enter to continue...")

		try:
			with self.env:
				result = trajoptpy.OptimizeProblem(prob) # do optimization
				t_elapsed = time.time() - t_start
				print "result trajectory"
				print result.GetTraj()
				trajectories.append(result.GetTraj())
				print "optimization took %.3f seconds"%t_elapsed

				raw_input("Press enter to continue...")

				# from trajoptpy.check_traj import traj_is_safe
				# prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
				# #assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free
				# print "Is traj safe:",traj_is_safe(result.GetTraj(), self.robot)

				# if (traj_is_safe(result.GetTraj(), self.robot)):
				# 	self.ExecuteTrajectory(result.GetTraj())

				#raw_input("Press enter to exit...")

		except:
			print "Unexpected error:", sys.exc_info()[0]

		for traj in trajectories:
			self.VisualizeTraj(traj, self.r_manip)
			raw_input("Press enter to continue...")

	def ModifySupportPath(self):
		if (self.support_path == None):
			return 

	def ExecuteTrajectory(self, trajToExecute):
		print '---------------EXECUTING TRAJECTORY---------------'
		with self.env:
		  traj = RaveCreateTrajectory(self.env,'')
		  spec = IkParameterization.GetConfigurationSpecificationFromType(IkParameterizationType.Transform6D,'linear')

		  #print self.robot.GetActiveConfigurationSpecification()

		  traj.Init(self.robot.GetActiveConfigurationSpecification());

		  for traj_pts in trajToExecute:
			print traj_pts
			traj.Insert(traj.GetNumWaypoints(), traj_pts)

		#don't execute yet
		#   planningutils.RetimeActiveDOFTrajectory(traj, self.robot, hastimestamps=False, maxvelmult=0.01, maxaccelmult=0.01, plannername='LinearTrajectoryRetimer')
		
		#   self.robot.GetController().SetPath(traj)
		# self.robot.WaitForController(0)

	def MoveManip(self, manip, joint_target):
		with self.env:
			self.robot.SetActiveDOFs(manip.GetArmIndices())
			self.robot.SetActiveDOFValues(joint_target)
			self.robot.GetController().SetDesired(self.robot.GetDOFValues())
		waitrobot(self.robot)

	def VisualizeTraj(self, trajectory, manip):
		print "visualizing trajectory"
		for joint_vals in trajectory:
			with self.env:
				self.robot.SetActiveDOFs(manip.GetArmIndices())
				self.robot.SetActiveDOFValues(joint_vals)
				self.robot.GetController().SetDesired(self.robot.GetDOFValues())
			waitrobot(self.robot)
			ee_transform = manip.GetEndEffectorTransform()
			self.draw_handles.append(self.env.plot3(ee_transform[0:3,3],
											   pointsize=15.0,
											   colors=array(((0.3,0.2,0.8)))))
			raw_input("Press enter to continue...")

	#test functions
	def RelevantPathWaypointsTest(self):
		total_waypoints = self.GetRelevantPathWaypoints(self.ppePathTransforms)
		for curve_waypoint in self.GetPathWaypointsCurvature(self.ppePathTransforms):
			idx = curve_waypoint[1]
			appeared = False
			for t_waypoint in total_waypoints:
				if (t_waypoint[1] == idx):
					appeared = True
					break
			if (appeared == False):
				total_waypoints.append(curve_waypoint)

		print total_waypoints

	#costs
class CostWrapper:

	robot 		= None
	human 		= None
	env 		= None
	r_manip 	= None
	l_manip 	= None
	utils 		= None

	start_t_target 	= []
	end_t_target 	= []

	grid = []

	def Init(self, robot, env, human, utils, r_manip, l_manip):
		self.robot 		= robot
		self.env 		= env
		self.human 		= human
		self.utils 		= utils
		self.r_manip 	= r_manip
		self.l_manip 	= l_manip

	def CartesianDeviationCost(x, n, waypoint):
		tool_pose = [] #do forward kinematics
		return np.linalg.norm(self.utils.SubVec(waypoint[0:3,3], tool_pose[0:3,3]))

	def RotationDeviationCost(x, n, waypoint):
		tool_pose = []

		way_point_quat 	= transformations.quaternion_from_matrix(waypoint[0:3, 0:3])
		quat 			= transformations.quaternion_from_matrix(tool_pose[0:3, 0:3])
		adjustment_quat = self.utils.QuaternionFromTo(way_point_quat, quat)
		axis_diff_mag 	= np.fabs(sum(self.utils.SubVec(way_point_quat[1:4], adjustment_quat[1:4]))) 

		return (0.25 * axis_mag + 0.75 * adjustment_quat[0]) #25% of cost from different in axis

	def TestCost(self, x):
		print 'args are',x,self.end_t_target
		return 0

	def LegibilityCost(self):
		return 0

	def PredictabilityCost(self):
		return 0