#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy as np
from cozmo.util import degrees
import math

def get_relative_pose(object_pose, refrence_frame_pose):
	# ####
	# TODO: Implement computation of the relative frame using numpy.
	# Try to derive the equations yourself and verify by looking at
	# the books or slides bfore implementing.
	# ####

	cubeposx = object_pose.position.x
	cubeposy = object_pose.position.y
	cuberot = object_pose.rotation.angle_z.radians
	botposx = refrence_frame_pose.position.x
	botposy = refrence_frame_pose.position.y
	botrot = refrence_frame_pose.rotation.angle_z.radians

	diffx = cubeposx - botposx
	diffy = cubeposy = botposy
	difft = degrees(cuberot - botrot)

	matrix = np.matrix([[math.cos(cuberot), -math.sin(cuberot), cubeposx], [math.sin(cuberot), math.cos(cuberot), cubeposy], [0, 0, 1]])
	vector = np.matrix([[diffx, diffy, 0]]).T

	result = np.matmul(matrix, vector)

	print(result)

	newpose = cozmo.util.Pose(result[0], result[1], 0, angle_z=difft)

	return newpose

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
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
