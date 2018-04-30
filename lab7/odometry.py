#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees,radians, Angle, Pose, distance_mm, speed_mmps
import math
import time

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# I ran the following code
	#   cozmo_drive_straight(robot, 3.14 * 2 * 50, 30)
	# and I counted 13 rotations of 120 deg (the wheels have three radial marks)
	# Thus, 13/3 rotations takes you pi * 2 * r * (13/3) = pi * 2 * 50 mm
	#  so r = 50 * (3/13)
	return (50 * 3) / 13

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""

	# Running the following code
	#   s = 20 * 3.14
	#   robot.drive_wheels(s, -s, duration=10)
	# the robot spins 2.25 times.
	# Since  speed * time = pi * 20 * 10 = distance = pi * rotations * d 
	# d = 200 / 2.25 ~= 88
	# However, I can measure d as ~45mm, so I'm missing a factor of 2 somewhere
	return 200 / 2.25 / 2
DEG_PER_MM = 2.25 * 360 / (200 * 3.14) # deg/(mm/s * s)

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	# ####
	# TODO: Implement this function.
	# ####
	pass

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	if dist < 0:
		sign = -1
	else:
		sign = 1
	robot.drive_wheels(sign * speed, sign * speed, duration = sign * dist/speed)

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	if angle < 0:
		sign = -1
	else:
		sign = 1
	# deg/s = deg/mm * mm/s,   mm/s = (deg/s) / (deg/mm)
	s = sign * speed / DEG_PER_MM
	warm_up_time = 0.8   # The warm up appears to be necessary for small angles
	robot.drive_wheels(-s, s, duration = sign * angle / speed + warm_up_time)

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""

	angle = radians(math.atan2(y,x)).degrees
	my_turn_in_place(robot, angle, 60)
	time.sleep(1)
	my_drive_straight(robot, math.sqrt(x*x + y*y), 50)
	time.sleep(1)
	my_turn_in_place(robot, angle_z - angle, 60)
	time.sleep(1)

def delta(x, y, robot):
	return (dest_x - robot.pose.position.x, dest_y - robot.pose.position.y)

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	dest_x = x + robot.pose.position.x
	dest_y = y + robot.pose.position.y
	while(1):
		dx, dy = delta(dest_x, dest_y, robot)
		theta_r = robot.pose.rotation.angle_z.degrees
		p = math.sqrt( dx ** 2 + dy ** 2)
		a = theta_r - math.atan2(dy,dx)
		n = angle_z - theta_r

		v = 0.5 * p
		theta = 1.0 * (1.0 * a +  1.0 * n)
		l,r = v + theta , v - theta
		robot.drive_wheels(l, r, duration = 0.1)


	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the 
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####
	pass

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# as fast as possible. You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	pass

def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))


	## Example tests of the functions

	# cozmo_drive_straight(robot, 62, 50)
	# cozmo_turn_in_place(robot, 45, 30)
	# cozmo_go_to_pose(robot, 100, 100, 45)

	# rotate_back_wheel(robot, 90)
	# my_drive_straight(robot, 62, 50)
	# my_turn_in_place(robot, 45, 30)

	# cozmo_go_to_pose(robot, 300, 100, 0)
	# my_go_to_pose1(robot, -100, -100, 45)
	my_go_to_pose2(robot, 100, 100, 45)
	# my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	cozmo.run_program(run)



