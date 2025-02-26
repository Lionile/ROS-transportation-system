#!/usr/bin/env python3
import rospy
import moveit_commander
import tf2_ros
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
from math import pi
import numpy as np

cube_position = [-0.55, 0, 0.1]

rospy.init_node('arm_trajectory_publisher')


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

move_group_arm = moveit_commander.MoveGroupCommander('arm')


orientation = Quaternion(*quaternion_from_euler(0, pi, 0)) # grab cube from above
cube_pose = Pose(position=Point(*cube_position), orientation=orientation)
cube_approach_pose = Pose(position=Point(*cube_position), orientation=orientation)
cube_approach_pose.position.z += 0.25


# set arm to approach position
move_group_arm.set_pose_target(cube_approach_pose, end_effector_link = 'hand')
plan = move_group_arm.go(wait=True)
move_group_arm.stop()
move_group_arm.clear_pose_targets()

rospy.sleep(1)