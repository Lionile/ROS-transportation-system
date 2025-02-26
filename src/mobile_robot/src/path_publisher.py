#!/usr/bin/env python3
import rospy, actionlib
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('path_publisher')
pose_array = PoseArray()

goals = [
    # [position, quaternion]
    # [x, y, z,  x, z, y, w]
    [-4, -4, 0, 0, 0, 0, 1],
]

target_poses = []
for i, goal in enumerate(goals):
    pose = PoseStamped()
    pose.pose.position.x = goal[0]
    pose.pose.position.y = goal[1]
    pose.pose.position.z = goal[2]
    pose.pose.orientation.x = goal[3]
    pose.pose.orientation.y = goal[4]
    pose.pose.orientation.z = goal[5]
    pose.pose.orientation.w = goal[6]
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    target_poses.append(pose)
    pose_array.poses.append(pose.pose)

# post pose array of goals
pub = rospy.Publisher(rospy.resolve_name('key_points'), PoseArray, queue_size=10, latch=True)
pose_array.header.stamp = rospy.Time.now()
pose_array.header.frame_id = 'map'
pub.publish(pose_array)


# send actions until the robot completes the path
client = actionlib.SimpleActionClient(rospy.resolve_name('move_base'), MoveBaseAction)
client.wait_for_server()


for target in target_poses:
    goal = MoveBaseGoal()
    goal.target_pose = target

    client.send_goal(goal)
    wait = client.wait_for_result()