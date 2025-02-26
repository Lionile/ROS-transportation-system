#!/usr/bin/env python3
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('arm_trajectory_publisher')

gripper_trajectory_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
rospy.sleep(1)

trajectory_msg = JointTrajectory()
trajectory_msg.joint_names = ["gripper_joint_left", "gripper_joint_right"]  # Replace with your gripper joint names

point = JointTrajectoryPoint()
point.positions = [-0.015, -0.015]  # gripper open values
point.effort = [-100000.0, -100000.0]  # effort values
point.time_from_start = rospy.Duration(1.0)

trajectory_msg.points.append(point)
gripper_trajectory_pub.publish(trajectory_msg)

# wait for the gripper to open
rospy.sleep(1)