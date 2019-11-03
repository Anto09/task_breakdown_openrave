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
from costwrapper import CostWrapper
from goggles_robot import GogglesRobot

#arm and torso are dim 7 each
class CostRegion:
	#get limits from human model
	cost = 0
	dim = 7
	cost_range = np.zeros(shape=(7,2))
	center = np.zeros(7)

	def Init(self, cost_range, cost):
		self.cost_range = cost_range
		self.cost = cost
		for i in range(0,self.dim):
			center[i] = (self.cost_range[1,i] + self.cost_range[0,i])*0.5

	def Dist(self, config):
		return np.linalg.norm(config - self.center)

class HumanRobot:
	env 			= None
	human 			= None
	support_path 	= []
	rula_vertices 	= []
	target_joints 	= []
	joint_names 	= ["HeadX", "HeadY", "HeadZ","TorsoX", "TorsoY", "TorsoZ", 
					   "PelvisRotX", "PelvisRotY", "PelvisRotZ", "HipX", "HipY", "HipZ",
					   "Knee", "AnkleX", "AnkleY", "AnkleZ"]
	cost_regions	= []

	def Init(self, env, human):
		self.env = env
		self.human = human
		self.GenerateRULAVertices()

		joints = self.human.GetJoints()
		for joint in joints:
			print joint.GetName()

	def TestMove(self):
		self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in self.joint_names])
		for i in range(0,10):
			a_vals = self.human.GetActiveDOFValues()
			for i in range(0, len(a_vals)):
				a_vals[i] = a_vals[i] + 0.2
			with self.env:
				self.human.SetActiveDOFValues(a_vals)
			print self.human.GetActiveDOFValues()
			raw_input('press enter to contiue')

	def SetInitialState(self):
		self.human.SetDOFValues([  0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
								   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
									0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
									6.648e-01,  -3.526e-01,   1.676e+00,  -1.924e+00,   2.921e+00,
									-1.217e+00,   1.343e+00,   0.000e+00,  -4.163e-16,   0.000e+00,
									-1.665e-16,   0.000e+00,  -6.365e-01,   9.806e-02,  -1.226e+00,
									-2.026e+00,  -3.012e+00,  -1.396e+00,  -1.929e+00,   0.000e+00,
									2.776e-17,   0.000e+00,  -3.331e-16,   0.000e+00])
		self.human.SetTransform(np.array([[-1.   ,  0.005,  0.   ,  2.93 ],
										  [-0.005, -1.   ,  0.   ,  0.575],
										  [ 0.   ,  0.   ,  1.   ,  0.   ],
										  [ 0.   ,  0.   ,  0.   ,  1.   ]]))
		DOF = openravepy.DOFAffine                             
		self.human.SetActiveDOFs([self.human.GetJoint(name).GetDOFIndex() for name in self.joint_names])
		# target joint values
		self.target_joints = [0.,  0.6808, -0.3535,  1.4343, -1.8516,  2.7542, -1.2005,
							  1.5994, -0.6929, -0.3338, -1.292 , -1.9048, -2.6915, -1.2908,
							 -1.7152,  1.3155,  0.6877, -0.0041] 

	def make_fullbody_request(self, end_t, n_steps):
		coll_coeff = 20
		dist_pen = .05
		d = {
			"basic_info" : {
				"n_steps" : n_steps,
				"manip" : "inactive",
				"start_fixed" : True
			},
			"costs" : [
				{
					"type" : "joint_vel",
					"params": {"coeffs" : [1]}
				},            
				{
					"name" : "cont_coll",
					"type" : "collision",
					"params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":True}
				},
				{
					"name": "disc_coll",
					"type" : "collision",
					"params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":False}
				}            
			],
			"constraints" : [
			  {
				"type" : "pose", 
				"params" : {"xyz" : end_t[0:3,3].tolist(), 
							"wxyz" : transformations.quaternion_from_matrix(end_t[0:3,0:3]).tolist(), 
							"link": "Head",
							"timestep" : n_steps-1
							}
							 
			  }
			],
			"init_info" : {
				"type" : "stationary",
			}
		}
		
		return d

	def GenerateRULAVertices(self):
		return 0

	def TrajoptPath(self, end_t):
		# Planner assumes current state of robot in OpenRAVE is the initial state
		request = self.make_fullbody_request(end_t, 23)
		print request

		s = json.dumps(request) # convert dictionary into json-formatted string
		prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem
		result = trajoptpy.OptimizeProblem(prob) # do optimization
		
		traj = result.GetTraj()

	def TestTrajopt(self):
		head_t = self.human.GetLink('Head').GetTransform()
		end_t = np.copy(head_t)

		print 'end_t',end_t
		end_t[0,3] += 0.1
		raw_input('press enter to continue...')
		self.TrajoptPath(end_t)

		return 0

	def ExecuteTrajectory(self, traj):
		return 0
		l1 = np.linalg.norm(knee_trans[0:3,3] - ankle_trans[0:3,3])
		l2 = np.linalg.norm(hip_trans[0:3,3] - knee_trans[0:3,3])
		p2x = hip_trans[0,3]
		p2y = hip_trans[1,3]

		#q2 calculation
		c2 = (p2x**2 + p2y**2 - l1**2 - l2**2)/(2*l1*l2)
		s2_1 = np.sqrt(1-c2**2)
		s2_2 = -np.sqrt(1-c2**2)
		s2 = s2_1
		q2 = np.arctan2(s2_1, c2)
		if (q2 < 0):
			s2 = s2_2
			q2 = np.arctan2(s2_2, c2)

		#q1 calculation
		det = (l1**2 + l2**2 + (2*l1*l2*c2))
		s1 = (p2y*(l1+l2*c2) - p2x*l2*s2)/det
		c1 = (p2x*(l1+l2*c2) + p2y*l2*s2)/det
		q1 = np.arctan2(s1,c1)

		return q1,q2