#!/usr/bin/env python3
import rospy
import moveit_commander
import tf2_ros
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
import numpy as np

def arm_goto_cube():
    move_group_arm = moveit_commander.MoveGroupCommander('arm')


    listener = tf.TransformListener()
    cube_position = None
    cube_rotation = None

    while not rospy.is_shutdown() and cube_position is None:
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/cube', rospy.Time(0))
            cube_position = trans
            cube_rotation = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    e = euler_from_quaternion(cube_rotation)
    q = quaternion_from_euler(0, pi, e[2])

    orientation = Quaternion(*q)
    cube_pose = Pose(position=Point(*cube_position), orientation=orientation)
    cube_pose.position.z += 0.07
    approach_pose = Pose(position=Point(*cube_position), orientation=orientation)
    approach_pose.position.z += 0.2


    # get arm to approach position
    move_group_arm.set_pose_target(approach_pose, end_effector_link = 'hand')
    plan = move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()

    # open gripper
    move_group_gripper = moveit_commander.MoveGroupCommander('gripper')
    move_group_gripper.set_named_target('open')
    plan = move_group_gripper.go(wait=True)
    move_group_gripper.stop()
    move_group_gripper.clear_pose_targets()

    # get arm to cube (in a straight line)
    waypoints = []
    waypoints.append(approach_pose)  # initial pose
    waypoints.append(cube_pose)     # final pose

    # Compute a cartesian path
    (plan, fraction) = move_group_arm.compute_cartesian_path(waypoints, eef_step=0.01)
    move_group_arm.execute(plan, wait=True)

    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('arm_trajectory_publisher')
    arm_goto_cube()
