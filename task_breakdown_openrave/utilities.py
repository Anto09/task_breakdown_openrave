import sys
import os
import math

import rospy

import time
import threading

from geometry_msgs.msg import *
import transformations
import numpy as np

class Utilities:
	def CopyPose(self, p1):
		p2 = Point()

		p2.position.x = p1.position.x
		p2.position.y = p1.position.y
		p2.position.z = p1.position.z

		p2.orientation.w = p1.orientation.w
		p2.orientation.x = p1.orientation.x
		p2.orientation.y = p1.orientation.y
		p2.orientation.z = p1.orientation.z

		return p2

	def PointMag(self, p):
		return math.sqrt(p.x**2 + p.y**2 + p.z**2)

	def VecMag(self, v):
		return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

	def VecToPoint(self, vec):
		if (len(vec) != 3):
			print ('\n Error: Vector not a valid R3 vector \n')
			return

		p = Point()
		p.x = vec[0]
		p.y = vec[1]
		p.z = vec[2]

		return p

	def VecToQuat(self, vec):
		if (len(vec) != 4):
			print ('\n Error: Vector not a valid R4 vector \n')
			return

		q = Quaternion()
		q.x = vec[1]
		q.y = vec[2]
		q.z = vec[3]
		q.w = vec[0]

		return q

	def AddPoint(self, p0, p1):
		p2 = Point()
		p2.x = p0.x + p1.x
		p2.y = p0.y + p1.y
		p2.z = p0.z + p1.z

		return p2

	def SubPoint(self, p0, p1):
		p2 = Point()
		p2.x = p0.x - p1.x
		p2.y = p0.y - p1.y
		p2.z = p0.z - p1.z

		return p2

	def AddVec(self, v0, v1):
		v2 = [v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2]]

		return v2

	def SubVec(self, v0, v1):
		v2 = [v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2]]

		return v2

	def AddArr(self, v0, v1):
		v2 = []
		# print 'add'
		# print v0
		# print v1
		for i in range (0, len(v0)):
			# print i
			v2.append(v0[i] + v1[i])
		return v2

	def SubArr(self, v0, v1):
		v2 = []
		for i in range (0, len(v0)):
			v2.append(v0[i] - v1[i])
		return v2

	def MultArr(self, v, s):
		v1 = []
		for i in range (0, len(v)):
			v1.append(v[i] * s)
		return v1

	def PointToArray(self, p):
		return np.array([p.x, p.y, p.z])

	def ArrayToPoint(self, p_arr):
		p = Point()
		p.x = p_arr[0]
		p.y = p_arr[1]
		p.z = p_arr[2]

		return p

	def QuatMag(self, q):
		return math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)

	def MultiplyQuaternion(self, q0, q1):
		q2 = Quaternion()

		# w0 = q0.w
		# x0 = q0.x
		# y0 = q0.y
		# z0 = q0.z

		# w1 = q1.w
		# x1 = q1.x
		# y1 = q1.y
		# z1 = q1.z

		# q2.w = -x1*x0 - y1*y0 - z1*z0 + w1*w0
		# q2.x = x1*w0 + y1*z0 - z1*y0 + w1*x0
		# q2.y = -x1*z0 + y1*w0 + z1*x0 + w1*y0
		# q2.z = x1*y0 - y1*x0 + z1*w0 + w1*z0

		# qmag = self.QuatMag(q2)
		# if (qmag > 1):
		# 	q2.w = q2.w/qmag
		# 	q2.x = q2.z/qmag
		# 	q2.y = q2.y/qmag
		# 	q2.z = q2.z/qmag

		q0_arr = self.QuatToArray(q0)
		q1_arr = self.QuatToArray(q1)

		q2_arr = transformations.quaternion_multiply(q0_arr, q1_arr)
		q2 = self.ArrayToQuat(q2_arr)

		return q2;

	def QuaternionFromTo(self, q1, q2): #usable with both geometry_msgs/Quaternion and list
		q1_arr = None
		q2_arr = None

		if (type(q1) == Quaternion):
			q1_arr = self.QuatToArray(q1)
		else:
			q1_arr = q1[:]

		if (type(q2) == Quaternion):	
			q2_arr = self.QuatToArray(q2)
		else:
			q2_arr = q2[:]

		q1_arr_inv = transformations.quaternion_inverse(q1_arr);
		q3_arr = transformations.quaternion_multiply(q1_arr_inv, q2_arr)
		return self.ArrayToQuat(q3_arr)


	def QuatDiff(self, q1, q2, a, b):
		q1_arr = None
		q2_arr = None

		if (type(q1) == Quaternion):
			q1_arr = self.QuatToArray(q1)
		else:
			q1_arr = q1[:]

		if (type(q2) == Quaternion):	
			q2_arr = self.QuatToArray(q2)
		else:
			q2_arr = q2[:]

		axis_diff  = self.SubArr([q1_arr[1], q1_arr[2], q1_arr[2]], [q2_arr[1], q2_arr[2], q2_arr[2]])
		theta_diff = np.fabs(np.arctan2(np.sin(q1_arr[0]), np.cos(q1_arr[0])) - np.arctan2(np.sin(q2_arr[0]), np.cos(q2_arr[0])))

		return (a*np.linalg.norm(axis_diff)) + (b*theta_diff)

	def MultiplyQuaternionArr(self, q0, q1):
		w0, x0, y0, z0 = q0
		w1, x1, y1, z1 = q1
		q_arr = np.array([-x1*x0 - y1*y0 - z1*z0 + w1*w0,
						   x1*w0 + y1*z0 - z1*y0 + w1*x0,
						  -x1*z0 + y1*w0 + z1*x0 + w1*y0,
						   x1*y0 - y1*x0 + z1*w0 + w1*z0], dtype=np.float64)

		qlen = vector_norm(q_arr)
		if (qlen > 1):
			q_arr = q_arr/qlen

		return q_arr

	def AddQuat(self, q0, q1):
		q2 = Quaternion()
		q2.x = q0.x + q1.x
		q2.y = q0.y + q1.y
		q2.z = q0.z + q1.z
		q2.w = q0.w + q1.w

		return q2

	def SubQuat(self, q0, q1):
		q2 = Quaternion()
		q2.x = q0.x - q1.x
		q2.y = q0.y - q1.y
		q2.z = q0.z - q1.z
		q2.w = q0.w - q1.w

		return q2

	def ArrayToQuat(self, q_arr): #numpy.array to geometry_msgs/Quaternion
		q = Quaternion()
		q.w = q_arr[0]
		q.x = q_arr[1]
		q.y = q_arr[2]
		q.z = q_arr[3]

		return q

	def QuatToArray(self, q): #numpy.array to geometry_msgs/Quaternion
		q_arr = [0, 0, 0, 0]
		q_arr[0] = q.w
		q_arr[1] = q.x
		q_arr[2] = q.y
		q_arr[3] = q.z

		return q_arr

	def RotateVector(self, q, p): #geometry_msgs/Quaternion and geometry_msgs/Point
		quat_array = self.QuatToArray(q)
		angle_transform = transformations.quaternion_matrix(quat_array) #4x4 numpy array

		p_arr = self.PointToArray(p)
		p_transform = transformations.translation_matrix(p_arr)

		new_p_transform = np.dot(angle_transform, p_transform)
		new_p_arr = new_p_transform[:3,3]
		new_p_vec = self.ArrayToPoint(new_p_arr)

		return new_p_vec

	def RotateVectorArr(self, q_arr, p_arr): #geometry_msgs/Quaternion and geometry_msgs/Point
		angle_transform = transformations.quaternion_matrix(q_ar)
		p_transform = transformations.translation_matrix(p_arr)

		new_p_transform = np.dot(angle_transform, p_transform)
		new_p_arr = new_p_transform[:3,3]

		return new_p_arr

	def CalcSlope(self, p1, p2, p3 = None): #caclulate slope
		p1_arr = []
		p2_arr = []
		p3_arr = []

		if (type(p1) == Point):
			p1_arr = self.PointToArray(p1)
		elif (len(p1) > 3): #transformation matrix
			p1_arr = p1[0:3,3]
		else:
			p1_arr = p1[:]

		if (type(p2) == Point):
			p2_arr = self.PointToArray(p2)
		elif (len(p2) > 3): #transformation matrix
			p2_arr = p2[0:3,3]
		else:
			p2_arr = p2[:]

		if (p3 != None):
			if (type(p3) == Point):
				p3_arr = self.PointToArray(p3)
			elif (len(p3) > 3): #transformation matrix
				p3_arr = p3[0:3,3]
			else:
				p3_arr = p3[:]

		p1p2mag = self.VecMag(self.SubVec(p2_arr, p1_arr))

		p1p2xy_slope = (p2_arr[2] - p1_arr[2])/p1p2mag
		p1p2xz_slope = (p2_arr[1] - p1_arr[1])/p1p2mag
		p1p2yz_slope = (p2_arr[0] - p1_arr[0])/p1p2mag

		if (p3 != None):
			p2p3mag = self.VecMag(self.SubVec(p3_arr, p2_arr))
			p2p3xy_slope = (p3_arr[2] - p2_arr[2])/p2p3mag
			p2p3xz_slope = (p3_arr[1] - p2_arr[1])/p2p3mag
			p2p3yz_slope = (p3_arr[0] - p2_arr[0])/p2p3mag


		if (p3 != None):
			return [p1p2xy_slope, p1p2xz_slope, p1p2yz_slope], [p2p3xy_slope, p2p3xz_slope, p2p3yz_slope]
		return [p1p2xy_slope, p1p2xz_slope, p1p2yz_slope]

	def EvalSlopeDir(self, p1, p2, p3): #returns if there is a sign change in the direction vector from 
		p1p2dir, p2p3dir = self.CalcSlope(p1, p2, p3)
		if (np.sign(p1p2dir[0]) != np.sign(p2p3dir[0]) and np.fabs(p1p2dir[0] - p2p3dir[0]) > 1e-4):
			return True
		elif (np.sign(p1p2dir[1]) != np.sign(p2p3dir[1]) and np.fabs(p1p2dir[1] - p2p3dir[1]) > 1e-4):
			return True 
		elif (np.sign(p1p2dir[2]) != np.sign(p2p3dir[2]) and np.fabs(p1p2dir[2] - p2p3dir[2]) > 1e-4):
			return True
		return False

	def InterpolateConfig(self, sol_0, sol_n, sol_diff, x_0, x_n, x):
		scaler =  (x - x_0)/(x_n - x_0)
		new_sol = self.AddArr(sol_0, self.MultArr(sol_diff, scaler))
		return new_sol

	def DisplaceTransform(self, transform, vector):
		transform[0,3] += vector.x
		transform[1,3] += vector.y
		transform[2,3] += vector.z
		return #numpy transforms are PBR


	def ModifyTransform(self, transform, disp_vec, adjustment_quat):
		t = np.copy(transform)
		new_pose = Pose()
		new_pose.position = self.AddPoint(self.VecToPoint(t[0:3,3]), disp_vec)
		new_pose.orientation = self.MultiplyQuaternion(self.VecToQuat(transformations.quaternion_from_matrix(t[0:3, 0:3])), adjustment_quat)
		new_transform = transformations.quaternion_translation_matrix(self.QuatToArray(new_pose.orientation), self.PointToArray(new_pose.position))
		
		return new_transform

	def TransformToPoint(self, transform):
		t = np.copy(transform)
		new_pose = Pose()
		new_pose.position = self.VecToPoint(t[0:3,3])
		new_pose.orientation = self.VecToQuat(transformations.quaternion_from_matrix(t[0:3, 0:3]))
		
		return new_pose

	def ModifyTransformToPoint(self, transform,  disp_vec, adjustment_quat):
		new_transform = self.ModifyTransform(transform,  disp_vec, adjustment_quat)

		return self.TransformToPoint(new_transform)

	def RediscretizeTrajectory(self, traj, num_between_points):
		or_points = len(traj)
		if (num_between_points < 1):
			return None

		new_traj = []
		for i in range (0, or_points - 1):
			cur_traj_point = traj[i]
			next_traj_point = traj[i+1]

			old_pos = cur_traj_point[0:3,3]
			next_pos = next_traj_point[0:3,3]
			pos_diff = (next_pos - old_pos)

			or_quat = transformations.quaternion_from_matrix(cur_traj_point)
			next_quat = transformations.quaternion_from_matrix(next_traj_point)

			new_traj.append(np.copy(cur_traj_point))
			for j in range(0, num_between_points):
				fraction = float(j+1) / float(num_between_points+1) 
				interp_quat = transformations.quaternion_slerp(or_quat, next_quat, fraction)
				interp_pos = old_pos + (fraction * pos_diff)
				
				interp_trans = transformations.quaternion_translation_matrix(interp_quat, interp_pos)
				new_traj.append(np.copy(interp_trans))
		new_traj.append(np.copy(traj[len(traj)-1])) #append last traj point

		return new_traj

	def DiscretizeCspaceTrajectory(self, traj, num_between_points):
		or_points = len(traj)
		if (num_between_points < 1):
			return None

		new_traj = []
		for i in range (0, or_points-1):
			cur_traj_point = traj[i]
			next_traj_point = traj[i+1]
			diff = next_traj_point - cur_traj_point

			for j in range(0, num_between_points):
				fraction = float(j+1) / float(num_between_points+1)
				interp_config = cur_traj_point + (fraction * diff)
				new_traj.append(interp_config)

		return new_traj

	def Yoshikawa(self, env, robot, manip, config, start_config):
		ymi = 0
		with env:
			robot.SetActiveDOFs(manip.GetArmIndices())
			robot.SetActiveDOFValues(config)
			jac = manip.CalculateJacobian()
			jac = manip.CalculateAngularVelocityJacobian()
			ymi = np.sqrt(np.linalg.det(np.dot(jac, jac.transpose())))
		return ymi

	def JointLimitPenalty(self, manip, robot, config):
		p_prod = 1
		idx = 0	
		joints = robot.GetJoints()
		k = 0

		print '\n printing joint limit differences \n'

		for m_idx in manip.GetArmIndices():
			joint = joints[m_idx]
			limits = joint.GetLimits()

			# print 'limits', limits

			l_diff = config[idx] - limits[0]
			g_diff = limits[1] - config[idx]
			lim_diff = limits[1] - limits[0]

			print 'min diff', min(l_diff, g_diff), 'for', joint.GetName()

			k = max(k, lim_diff * 180.0 / np.pi)

			# print l_diff, g_diff, lim_diff

			p_prod *= (g_diff * l_diff)/(lim_diff**2)
			idx += 1

		print '\n --------------------------------- \n'

		print 'k', k
		print 'p_prod', p_prod
		print 'exp', np.exp(-k * p_prod)
		print 'score', 1-np.exp(-k * p_prod)
		return 1-np.exp(-k * p_prod)

	def RankConfigurationsManipulability(self, env, configs, robot, manip, start_config):
		best_config = None
		manip_score = -9999999

		# print manip.GetChildJoints()
		# print 'limits'
		# for idx in manip.GetArmIndices():
		# 	print robot.GetJoints()[idx].GetName()
		# 	print robot.GetJoints()[idx].GetLimits()[0], robot.GetJoints()[idx].GetLimits()[1]

		for config in configs:
			score = 0
		
			yosh = self.Yoshikawa(env, robot, manip, config, start_config)
			jlp = self.JointLimitPenalty(manip, robot, config)

			print 'config', config
			print 'Yoshikawa manipulability index\n', yosh
			print 'Joint Limit Penalty\n', jlp
			print 'Product\n', yosh * 1/jlp
			print '\n'

			# with env:
			# 	robot.SetActiveDOFs(manip.GetArmIndices())
			# 	robot.SetActiveDOFValues(config)

			# 	jac = manip.CalculateJacobian()
			# 	jac = manip.CalculateAngularVelocityJacobian()
			# 	score = np.sqrt(np.linalg.det(np.dot(jac, jac.transpose())))
				
			score = yosh * 1/jlp

			raw_input('press enter to continue')
			with env:
				robot.SetActiveDOFValues(start_config)
			if (score > manip_score):
				manip_score = score
				best_config = config

		print 'best config', best_config
		print 'best score', manip_score

		with env:
			robot.SetActiveDOFs(manip.GetArmIndices())
			robot.SetActiveDOFValues(best_config)
		raw_input('press enter to continue')
		
		with env:
			robot.SetActiveDOFValues(start_config)

		return best_config