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

from abc import ABCMeta, abstractmethod

class PPEBase:
	#abstract class stuff
	__metaclass__ = ABCMeta

	#objects
	is_attached		= False
	env         	= None
	robot       	= None
	dof_idxs		= []

	@abstractmethod #has to be overriden
	def Init(self, env, robot, is_attached = False):
		self.env 		 = env
		self.robot 		 = robot
		self.is_attached = False

	@abstractmethod
	def Reset(self):
		return

	@abstractmethod
	def ResetOutOfCollision(self):
		return

	@abstractmethod
	def Collapse(self):
		return

	@abstractmethod
	def Simulate(self):
		return

	@abstractmethod
	def UpdateTransforms(self):
		return

	@classmethod
	def __subclasshook__(cls, C):
		if cls is PPEBase:
			if any("__iter__" in B.__dict__ for B in C.__mro__):
				return True
		return NotImplemented

assert issubclass(tuple, PPEBase)
assert isinstance((), PPEBase)