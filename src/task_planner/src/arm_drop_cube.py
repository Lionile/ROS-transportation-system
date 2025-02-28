#!/usr/bin/env python3
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent


grasped = True

def grasp_callback(msg):
    global grasped
    grasped = msg.attached

def arm_drop_cube():
    global grasped
    gripper_trajectory_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    grasped = True
    sub = rospy.Subscriber('/gazebo_grasp_event_publisher/grasp_events', GazeboGraspEvent, grasp_callback)
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ["gripper_joint_left", "gripper_joint_right"]

    point = JointTrajectoryPoint()
    point.positions = [-0.015, -0.015]  # gripper open values
    point.effort = [-100000.0, -100000.0]  # effort values
    point.time_from_start = rospy.Duration(1.0)

    trajectory_msg.points.append(point)
    gripper_trajectory_pub.publish(trajectory_msg)

    # wait for the gripper to open
    while not rospy.is_shutdown():
        if not grasped:
            return

if __name__ == '__main__':
    rospy.init_node('arm_trajectory_publisher')
    arm_drop_cube()
