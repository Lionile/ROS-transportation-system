#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('initial_pose_publisher')
pub = rospy.Publisher('/mob_rob/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)

x = rospy.get_param('initial_pose_publisher/robot_x')
y = rospy.get_param('initial_pose_publisher/robot_y')
pose = PoseWithCovarianceStamped()
pose.header.frame_id = 'map'
pose.pose.pose.position.x = x
pose.pose.pose.position.y = y
pose.pose.pose.position.z = 0
pose.pose.pose.orientation.x = 0
pose.pose.pose.orientation.y = 0
pose.pose.pose.orientation.z = 0
pose.pose.pose.orientation.w = 1

rospy.sleep(1)  # Give time for the publisher to connect
pub.publish(pose)
rospy.loginfo("Initial pose published")

rospy.sleep(1)