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

from ppe_base import PPEBase

class GogglesRobot(PPEBase):

	#objects
	env         	= None
	robot       	= None
	back_strap  	= None
	lens_body   	= None
	left_strap1		= None
	right_strap1	= None
	left_strap2		= None
	right_strap2	= None

	#vectors
	strap_o_trans   = []
	strap_to_lenses = []
	strap_o_diff    = []

	#transforms
	lens_body_init_trans	= []
	back_strap_init_trans	= []
	left_strap1_init_trans	= []
	right_strap1_init_trans	= []
	left_strap2_init_trans	= []
	right_strap2_init_trans	= []

	out_of_collision = False
	i_val = 0.22
	max_val = 0.27

	def Init(self, env, robot, is_attached = False):
		self.env 			 = env
		self.robot 			 = robot
		self.is_attached	 = is_attached
		self.back_strap 	 = self.robot.GetLink('backstrap')
		self.lens_body  	 = self.robot.GetLink('lenses')
		self.left_strap1	 = self.robot.GetLink('leftstrap1')
		self.left_strap2  	 = self.robot.GetLink('leftstrap2')
		self.right_strap1 	 = self.robot.GetLink('rightstrap1')
		self.right_strap2 	 = self.robot.GetLink('rightstrap2')
		self.strap_o_trans 	 = self.back_strap.GetTransform()
		self.strap_o_diff 	 = self.strap_o_trans[0:3,3] - self.robot.GetTransform()[0:3,3]
		self.strap_to_lenses = self.strap_o_trans[0:3,3] - self.lens_body.GetTransform()[0:3,3] 

		self.lens_body_init_trans		= self.lens_body.GetTransform()
		self.back_strap_init_trans		= self.back_strap.GetTransform()
		self.left_strap1_init_trans		= self.left_strap1.GetTransform()
		self.right_strap1_init_trans	= self.right_strap1.GetTransform()
		self.left_strap2_init_trans		= self.left_strap2.GetTransform()
		self.right_strap2_init_trans	= self.right_strap2.GetTransform()
		
		with self.env:
			self.robot.SetActiveDOFs([0])
			self.robot.SetActiveDOFValues([self.i_val])

	def Reset(self, env_locked = False):
		if (env_locked == True):
			self.lens_body.SetTransform(self.lens_body_init_trans)
			self.back_strap.SetTransform(self.back_strap_init_trans)
			self.left_strap1.SetTransform(self.left_strap1_init_trans)
			self.right_strap1.SetTransform(self.right_strap1_init_trans)
			self.left_strap2.SetTransform(self.left_strap2_init_trans)
			self.right_strap2.SetTransform(self.right_strap2_init_trans)
			self.robot.SetActiveDOFs([0])
			self.robot.SetActiveDOFValues([self.i_val])	
		else:
			with self.env:
				self.lens_body.SetTransform(self.lens_body_init_trans)
				self.back_strap.SetTransform(self.back_strap_init_trans)
				self.left_strap1.SetTransform(self.left_strap1_init_trans)
				self.right_strap1.SetTransform(self.right_strap1_init_trans)
				self.left_strap2.SetTransform(self.left_strap2_init_trans)
				self.right_strap2.SetTransform(self.right_strap2_init_trans)
				self.robot.SetActiveDOFs([0])
				self.robot.SetActiveDOFValues([self.i_val])	

	def UpdateTransforms(self):
		self.lens_body_init_trans		= self.lens_body.GetTransform()
		self.back_strap_init_trans		= self.back_strap.GetTransform()
		self.left_strap1_init_trans		= self.left_strap1.GetTransform()
		self.right_strap1_init_trans	= self.right_strap1.GetTransform()
		self.left_strap2_init_trans		= self.left_strap2.GetTransform()
		self.right_strap2_init_trans	= self.right_strap2.GetTransform()

	def ResetOutOfCollision(self):
		self.out_of_collision = False
		with self.env:
			self.robot.SetActiveDOFs([0])
			self.robot.SetActiveDOFValues([self.i_val])	

	def RayCollisionCheck(self, r_trans, env_locked = False):
		if (self.out_of_collision == True):
			if (env_locked == True):
				self.robot.SetTransform(r_trans)
			else:
				with self.env:
					self.robot.SetTransform(r_trans)
			return False

		inlier = False
		try:
			strap_loc 		= self.back_strap.GetTransform()[0:3,3]
			body_loc 		= self.lens_body.GetTransform()[0:3,3]
			adj_amount		= np.fabs(r_trans[0,3] - self.robot.GetTransform()[0,3])
			diff_length		= np.fabs(body_loc[0] - strap_loc[0])

			ray_start		= strap_loc - np.array([0.007, 0, 0])
			ray_end         = body_loc + np.array([0.05, 0, 0])
			ray_length 		= np.linalg.norm(ray_end - ray_start)
			ray_vec 		= ((ray_end - ray_start)/np.linalg.norm(ray_end - ray_start))*(ray_length)
			ray_endpoint 	= ray_start + ray_vec

			ray1 = array((ray_start[0],ray_start[1],ray_start[2],ray_vec[0],ray_vec[1],ray_vec[2]))
			raycast1 = Ray(ray1[0:3],ray1[3:6])
			report1 = CollisionReport()
			inlier1  = self.env.CheckCollision(raycast1, report1)
			inlier = inlier1 


			if (len(report1.contacts) > 0):
				print 'in collision'
				for c in report1.contacts:
					if (env_locked == True):
						self.robot.SetActiveDOFs([0])
						self.robot.SetActiveDOFValues([diff_length + adj_amount - np.linalg.norm(self.strap_o_diff)])
					else:
						with self.env:
							self.robot.SetActiveDOFs([0])
							self.robot.SetActiveDOFValues([diff_length + adj_amount - np.linalg.norm(self.strap_o_diff)])
				print 'new dof values', self.robot.GetActiveDOFValues()
		except openrave_exception,e:
			print e

		if (inlier == False):
			new_trans = self.robot.GetTransform()
			new_trans[0:3,3] += self.strap_o_diff
			if (env_locked == True):
				self.robot.SetActiveDOFs([0])
				self.robot.SetActiveDOFValues([0])
			else:
				with self.env:
					self.robot.SetActiveDOFs([0])
					self.robot.SetActiveDOFValues([0])
			self.out_of_collision = True

			b_transform = self.back_strap.GetTransform()
			rot_mat  = np.array([[np.cos(np.pi*0.5), -np.sin(np.pi*0.5), 0],
								 [np.sin(np.pi*0.5),  np.cos(np.pi*0.5), 0],
								 [0, 0, 1]])
			b_transform[0:3, 0:3] = np.dot(b_transform[0:3, 0:3], rot_mat)

			#flatten side straps
			self.left_strap1.SetTransform(b_transform)
			self.left_strap2.SetTransform(b_transform)
			self.right_strap1.SetTransform(b_transform)
			self.right_strap2.SetTransform(b_transform)
		return inlier

	def Collapse(self):
		with self.env:
			self.robot.SetActiveDOFs([0])
			self.robot.SetActiveDOFValues([-0.022])
		self.out_of_collision = True

		b_transform = self.back_strap.GetTransform()
		rot_mat  = np.array([[np.cos(np.pi*0.5), -np.sin(np.pi*0.5), 0],
							 [np.sin(np.pi*0.5),  np.cos(np.pi*0.5), 0],
							 [0, 0, 1]])
		b_transform[0:3, 0:3] = np.dot(b_transform[0:3, 0:3], rot_mat)

		#flatten side straps
		self.left_strap1.SetTransform(b_transform)
		self.left_strap2.SetTransform(b_transform)
		self.right_strap1.SetTransform(b_transform)
		self.right_strap2.SetTransform(b_transform)

	def Simulate(self, r_trans, env_locked = False): #PPEBase.simulate
		inlier = self.RayCollisionCheck(r_trans, env_locked)

		if (env_locked == True):
			self.robot.SetTransform(r_trans)
		else:
			with self.env:
				self.robot.SetTransform(r_trans)
		return inlier