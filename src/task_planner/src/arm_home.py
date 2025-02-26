#!/usr/bin/env python3
import rospy
import moveit_commander
import tf2_ros
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
from math import pi
import numpy as np

cube_position = [-0.3, 0, 0.1]

rospy.init_node('arm_trajectory_publisher')


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

move_group_arm = moveit_commander.MoveGroupCommander('arm')

# set arm to home position
move_group_arm.set_named_target('home')
plan = move_group_arm.go(wait=True)
move_group_arm.stop()
move_group_arm.clear_pose_targets()

rospy.sleep(1)