#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
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
	# ####
	# Found that front wheel makes one full rotation with 87 mm of travel
	# So radius = C / (2pi) = 87 / (2pi) = 13.84648
	# ####
	return 13.84648

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# Set robot to turn by having one tread move at 10 mm/s fwd, and the other in reverse.
	# One half-turn took 14.5 s
	# radius = C / (2pi) = (14.5 * 10 * 2) / (2pi) = 46.155
	# ####
	return 46.155

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	C = 2 * math.pi * get_front_wheel_radius()
	d = C * (angle_deg/360)
	cozmo_drive_straight(robot, d, 10)
	pass

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	time = dist/speed
	robot.drive_wheels(speed, speed, None, None, time)
	pass

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	distance = 2 * math.pi * get_distance_between_wheels() * (abs(angle)/360)
	time = distance/speed
	if angle < 0:
		robot.drive_wheels(speed, -speed, None, None, time)
	else:
		robot.drive_wheels(-speed, speed, None, None, time)
	pass

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""

	distance = math.sqrt(x**2 + y**2)
	theta = math.degrees(math.atan(y / x))
	print("theta: " + str(theta))
	my_turn_in_place(robot, theta, 30)
	time.sleep(0.25)
	my_drive_straight(robot, distance, 30)
	time.sleep(0.25)
	my_turn_in_place(robot, angle_z - theta, 30)
	pass

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	theta = (2 * math.pi * angle_z)/360

	d = get_distance_between_wheels()
	r = get_front_wheel_radius()
	t = 10

	#xr = (x * math.cos(theta)) - (y * math.sin(theta))
	xr = (x * math.cos(theta)) + (y * math.sin(theta))

	#Correll 3.64
	phiL = ((2 * xr) - (theta * d)) / (2 * r)
	phiR = ((2 * xr) + (theta * d)) / (2 * r)

	speedL = r * phiL / t
	speedR = r * phiR / t

	print(speedL)
	print(speedR)
	if x < 0:
		robot.drive_wheels(-speedL, -speedR, None, None, t)
	else:
		robot.drive_wheels(speedL, speedR, None, None, t)
	pass

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""

	if(x > 0):
		my_go_to_pose2(robot, x, y, angle_z)
	else:
		my_go_to_pose1(robot, x, y, angle_z)
	pass

def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	cozmo_drive_straight(robot, 62, 50)
	cozmo_turn_in_place(robot, 60, 30)
	cozmo_go_to_pose(robot, 100, 100, 45)

	rotate_front_wheel(robot, 90)
	my_drive_straight(robot, 62, 50)
	my_turn_in_place(robot, 90, 30)

	my_go_to_pose1(robot, 100, 100, 45)
	my_go_to_pose2(robot, 100, 100, 45)
	my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	cozmo.run_program(run)



