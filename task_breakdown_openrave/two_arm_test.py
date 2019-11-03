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

human = None
robot = None
init_dist = None
target_left_trans = None
target_right_trans = None

def GetIKSolutions(env, Tgoal, manip = None, getAll = True): #manip is OpenRAVE manip
	sol = None
	if (env == None or robot == None or manip == None):
		print('\n Error: Undefined env, robot, or manip \n')
		return sol

	with env:
		#sol = manip.FindIKSolutions(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
		if (getAll):
			sol = ku.ik_for_link(Tgoal, manip, manip.GetEndEffector().GetName(), filter_options = openravepy.IkFilterOptions.CheckEnvCollisions, return_all_solns = True)
		else:
			sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) 
	return sol

def GenerateTrajoptRequest(l_start_xyz, l_start_rot, l_end_xyz, l_end_rot, r_start_xyz, r_start_rot, r_end_xyz, r_end_rot, n_steps, end_joint_target):
		r_manip = robot.SetActiveManipulator('rightarm')
		robot.SetActiveDOFs(r_manip.GetArmIndices())
		print "right arm dofs", r_manip.GetArmIndices()
		# dofs = r_manip.GetArmIndices().tolist()
		l_manip = robot.SetActiveManipulator('leftarm')
		robot.SetActiveDOFs(l_manip.GetArmIndices())
		print "left arm dofs", l_manip.GetArmIndices()
		# for dof in l_manip.GetArmIndices():
		# 	dofs.append(dof)


		dof_idxs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
		robot.SetActiveDOFs(dof_idxs)
		jointnames = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',
					  'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']

		request = {
		  "basic_info" : {
			"n_steps" : n_steps, #num points of init traj
			# "dofs_fixed" : dofs_fixed, # see below for valid values
			"manip" : "active",
			"start_fixed" : True #True # i.e., DOF values at first timestep are fixed based on current robot state
		  },
		  "costs" : [
		  {
			"type" : "joint_vel", # joint-space velocity cost
			"params": {"coeffs" : [25]} # a list of length one is automatically expanded to a list of length n_dofs
		  },
		  {
			"type" : "collision",
			"name" :"cont_coll",
			"params" : {
			  "continuous" : True,
			  "coeffs" : [50], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.0025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			}, 
		  },   
		  {
			"type" : "collision",
			"name" :"dis_coll",
			"params" : {
			  "continuous" : False,
			  "coeffs" : [50], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
			  "dist_pen" : [0.0025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
			},
		  }
		  ],
		  "constraints" : [
		  {
			"type" : "pose", 
			"params" : {"xyz" : r_end_xyz.tolist(), 
						"wxyz" : r_end_rot.tolist(), 
						"link": "right_hand",
						"timestep" : n_steps-1 #-1 of n steps
						}
						 
		  },
			 #  {
				# "type" : "pose", 
				# "params" : {"xyz" : l_end_xyz.tolist(), 
				# 			"wxyz" : l_end_rot.tolist(), 
				# 			"link": "left_hand",
				# 			"timestep" : n_steps-1 #-1 of n steps
				# 			}
							 
			 #  },
		  {
			"type" : "cart_vel",
			"name" : "cart_vel",
			"params" : {
				"max_displacement" : 0.5,
				"first_step" : 0,
				"last_step" : n_steps-1, #inclusive
				"link" : "right_hand"
			},
		  },
		  {
			"type" : "cart_vel",
			"name" : "cart_vel",
			"params" : {
				"max_displacement" : 0.5,
				"first_step" : 0,
				"last_step" : n_steps-1, #inclusive
				"link" : "left_hand"
			},
		  }
		  ],
		  "init_info" : {
			  # "type" : "stationary"
			  "type" : "straight_line",
			  "endpoint" : end_joint_target
		  }
		}
		return request


def tuckarms(env):
	with env:
		jointnames = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',
					  'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
		robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
		robot.SetActiveDOFValues([-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50,
								   0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]);     

		# [0.04420055259270217, -0.19594282870316357, -2.0864891526533293, 2.1041240690183676, 1.1783420585762165, 1.9000000000000006, -1.0996960352608651, 
		#  -0.0067842913738776749, -0.21397319946011228, 1.4820739907512248, 1.9777164150711857, -1.4636258642540712, 1.4000000000000001, 0.88932466703515245]

def SetToInitialConfig(env):
	for link in human.GetLinks():
		print link.GetName()
	# target_part = human.G
	r_manip = robot.SetActiveManipulator('rightarm')


def GetHandDistance():
	r_manip = robot.SetActiveManipulator('rightarm')
	tright 	= r_manip.GetEndEffector().GetTransform()
	l_manip = robot.SetActiveManipulator('leftarm')
	tleft 	= l_manip.GetEndEffector().GetTransform()


	tdiff = np.dot(np.linalg.inv(tright), tleft)

	delta = np.array([tdiff[0,3],
					  tdiff[1,3],
					  tdiff[2,3],
					  np.arctan2(tdiff[2,1], tdiff[2,2]),
					  -np.arcsin(tdiff[2,0]),
					  np.arctan2(tdiff[1,0], tdiff[0,0])])

	delta_pos = delta[0:3]
	delta_rot = delta[3:6]

	return delta

def ClosureConstraint(x):	
	tright 	= None
	tleft 	= None
	r_state = robot.CreateRobotStateSaver()
	r_manip = robot.SetActiveManipulator('rightarm')
	l_manip = robot.SetActiveManipulator('leftarm')
	with r_state:
		jointnames = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',
					  'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
		robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
		robot.SetActiveDOFValues(x)
		tright 	= r_manip.GetEndEffector().GetTransform()
		tleft 	= l_manip.GetEndEffector().GetTransform()

	tdiff = np.dot(np.linalg.inv(tright), tleft)

	delta = np.array([tdiff[0,3],
					  tdiff[1,3],
					  tdiff[2,3],
					  np.arctan2(tdiff[2,1], tdiff[2,2]),	
					  -np.arcsin(tdiff[2,0]),
					  np.arctan2(tdiff[1,0], tdiff[0,0])])

	delta_pos = delta[0:3]
	delta_rot = delta[3:6]

	return np.linalg.norm(delta - init_dist)

def FKSolutionCost(x):
	tright 	= None
	tleft 	= None
	r_state = robot.CreateRobotStateSaver()
	r_manip = robot.SetActiveManipulator('rightarm')
	l_manip = robot.SetActiveManipulator('leftarm')
	with r_state:
		jointnames = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2',
					  'right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
		robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
		robot.SetActiveDOFValues(x)
		tright 	= r_manip.GetEndEffector().GetTransform()
		tleft 	= l_manip.GetEndEffector().GetTransform()

	tdiff_right = np.dot(np.linalg.inv(tright), target_right_trans)
	tdiff_left = np.dot(np.linalg.inv(tleft), target_left_trans)

	delta_right = np.array([tdiff_right[0,3],
					 		tdiff_right[1,3],
							tdiff_right[2,3],
							np.arctan2(tdiff_right[2,1], tdiff_right[2,2]),	
						   -np.arcsin(tdiff_right[2,0]),
							np.arctan2(tdiff_right[1,0], tdiff_right[0,0])])

	delta_left = np.array([tdiff_left[0,3],
					 	   tdiff_left[1,3],
						   tdiff_left[2,3],
						   np.arctan2(tdiff_left[2,1], tdiff_left[2,2]),	
						  -np.arcsin(tdiff_left[2,0]),
						   np.arctan2(tdiff_left[1,0], tdiff_left[0,0])])

	return np.linalg.norm(delta_right) + np.linalg.norm(delta_left);


# if planning for one arm only
def IkSolutionCost(x):
	tright = None;

def VisualizeTwoArmTraj(env, trajectory, orig_traj = None, isGreen = True):
	print "Visualizing trajectory", type(trajectory[0])
	r_manip = robot.SetActiveManipulator('rightarm')
	l_manip = robot.SetActiveManipulator('leftarm')

	for joint_vals in trajectory:
		with env:
			robot.SetActiveDOFs(l_manip.GetArmIndices())
			robot.SetActiveDOFValues(joint_vals[0:7])
		with env:
			robot.SetActiveDOFs(r_manip.GetArmIndices())
			robot.SetActiveDOFValues(joint_vals[7:14])
		dist = GetHandDistance()
		print "hand dist", dist
		print "init dist", init_dist
		print "target", np.linalg.norm(dist - init_dist)
		# print "target", np.fabs(dist[0] - init_dist[0])
		raw_input("Press enter to continue...")

def JointVelCost(x):
	cost = 0
	dof_count = 14
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

	diag_mat = np.eye(x_new.shape[1])

	print 'diffAxis', diffAxis
	print 'diag_mat', diag_mat
	print 'dot', (np.square(diffAxis).dot(diag_mat))
	print 'value', (np.square(diffAxis).dot(diag_mat)).sum()

	return (np.square(diffAxis).dot(diag_mat)).sum()

if __name__ == "__main__":

	env = Environment()
	env.SetViewer('qtcoin')
	env.Reset()        
	env.Load("/home/anto/ebolabot_ws/src/task_breakdown_openrave/src/task_breakdown_openrave/empty2.env.xml")
	time.sleep(0.1)

	InitOpenRAVELogging() 
	trajoptpy.SetInteractive(False)
	# trajoptpy.SetInteractive(True)

	robot = env.GetRobots()[0]

	print 'BEFORE TUCK', GetHandDistance()

	raw_input('press enter to tuck')
	tuckarms(env)

	utils = Utilities()

	r_manip = robot.SetActiveManipulator('rightarm')
	start_right_trans = r_manip.GetEndEffector().GetTransform()
	start_right_quat = transformations.quaternion_from_matrix(start_right_trans)
	target_right_trans = np.copy(start_right_trans)
	target_right_trans[0,3] -= 0.1
	target_right_trans[2,3] += 0.3
	target_right_quat = transformations.quaternion_from_matrix(target_right_trans)

	l_manip = robot.SetActiveManipulator('leftarm')
	start_left_trans = l_manip.GetEndEffector().GetTransform()
	start_left_quat = transformations.quaternion_from_matrix(start_left_trans)
	target_left_trans = np.copy(start_left_trans)
	target_left_trans[0,3] -= 0.1
	target_left_trans[2,3] += 0.3
	# target_left_trans[1,3] += 0.5
	target_left_quat = transformations.quaternion_from_matrix(target_left_trans)

	print "left_hand target:", target_left_trans
	print "right_hand target:", target_right_trans

	sols_r = GetIKSolutions(env, target_right_trans, r_manip)
	sols_l = GetIKSolutions(env, target_left_trans, l_manip)
	end_joint_target = sols_l[0].tolist()
	for sol in sols_r[0]:
		end_joint_target.append(sol)

	n_steps = 25
	request = GenerateTrajoptRequest(start_left_trans[0:3,3], start_left_quat, target_left_trans[0:3,3], start_left_quat, 
											start_right_trans[0:3,3], start_right_quat, target_right_trans[0:3,3], start_right_quat, n_steps, end_joint_target)


	# print request

	init_dist = GetHandDistance()
	print 'INIT DIST', init_dist
	s = json.dumps(request) # convert dictionary into json-formatted string
	prob = None 

	with env:	
		prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
		for t in range(0, n_steps):
			prob.AddConstraint(ClosureConstraint, [(t,j) for j in xrange(14)], "EQ", "up%i"%t)
			# prob.AddCost(ClosureConstraint, [(t,j) for j in xrange(14)], "ABS")
			# prob.AddCost(FKSolutionCost, [(t,j) for j in xrange(14)], "ABS")

	trajectories = []
	raw_input("press enter to solve")

	try:
		with env:
			result = trajoptpy.OptimizeProblem(prob) # do optimization
			trajectories.append(result.GetTraj())
			# print "optimization took %.3f seconds"%t_elapsed
			raw_input("Press enter to continue...")
	except:
		print "Unexpected error:", sys.exc_info()[0]

	for traj in trajectories:
		VisualizeTwoArmTraj(env, traj)
		raw_input("Press enter to continue...")

	print "left_hand current:", l_manip.GetEndEffector().GetTransform()[0:3,3]
	print "right_hand current:", r_manip.GetEndEffector().GetTransform()[0:3,3]

	# left_hand target: [ 0.5651351  0.1789879  0.6845309]
	# 				  [ -0.01601409  0.76013495  0.3845309   1.
	raw_input("press enter to exit")

	# GenerateTrajoptRequest()