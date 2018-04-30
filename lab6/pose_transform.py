#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy as np
from cozmo.util import degrees,Pose,radians
import time
from math import sin,cos,asin,acos

def to_mat(tuples, n, m):
	return np.matrix([tuples[(n*i):(n*(i+1))] for i in range(m)])

def to_matrix(pose):
	# rx,ry,rz = pose.rotation.euler_angles
	rx,ry,rz = 0,0,pose.rotation.angle_z.radians
	RX = np.matrix([ # rot X
		[1,       0,        0, 0],
		[0, cos(rx), -sin(rx), 0],
		[0, sin(rx),  cos(rx), 0],
		[0,       0,        0, 1]
	])
	RY = np.matrix([ # rot Y
		[ cos(ry), 0, sin(ry), 0],
		[       0, 1,       0, 0],
		[-sin(ry), 0, cos(ry), 0],
		[       0, 0,       0, 1]
	])
	RZ = np.matrix([ # rot Z
		[ cos(rz), -sin(rz), 0, 0],
		[ sin(rz),  cos(rz), 0, 0],
	    [       0,        0, 1, 0],
		[       0,        0, 0, 1]
	])

	pos = pose.position
	T = np.matrix([ # move
		[1,0,0,pos.x],
		[0,1,0,pos.y],
		[0,0,1,pos.z],
		[0,0,0,1]
	])
	# print(RX)
	# print(RY)
	# print(RZ)
	return T * RX * RY * RZ

def matrix_to_pose(rel):
	rz = acos(rel[0,0])
	if rel[1,0] < 0:   # sin(theta) < 0 -> theta < 0
		rz = -rz

	return Pose(rel[0,-1], rel[1,-1], rel[2,-1], angle_z=radians(rz))


def get_relative_pose(object_pose, refrence_frame_pose):
	ref,obj = [to_matrix(x) for x in [refrence_frame_pose, object_pose]]

	return matrix_to_pose(ref.I * obj)

def get_absolute_pose(rel_object_pose, refrence_frame_pose):
	ref,obj = [to_matrix(x) for x in [refrence_frame_pose, rel_object_pose]]

	return matrix_to_pose(ref * obj)

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % (robot.pose,))
				print("Cube pose: %s" % (cube.pose,))
				rel = get_relative_pose(cube.pose, robot.pose)
				print("Cube pose in the robot coordinate frame: %s" % (rel,))
				print("Calc cube: %s" % (robot.pose.define_pose_relative_this(rel),))
				print("-" * 20)
		except asyncio.TimeoutError:
			print("Didn't find a cube")
		time.sleep(1)


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
