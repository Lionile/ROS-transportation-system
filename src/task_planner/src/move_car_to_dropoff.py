#!/usr/bin/env python3
import rospy, actionlib
from math import pi
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from tf.transformations import quaternion_from_euler
import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive



def move_car_to_dropoff():
    pose_array = PoseArray()


    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()

    # Define the cube as a collision object
    cube = CollisionObject()
    cube.id = "cube"
    cube.header.frame_id = "map"

    # Define the cube's shape and dimensions
    cube_primitive = SolidPrimitive()
    cube_primitive.type = SolidPrimitive.BOX
    cube_primitive.dimensions = [0.05, 0.05, 0.05]  # dimensions of the cube

    # Define the cube's pose
    cube_pose = Pose()
    cube_pose.position.x = 0
    cube_pose.position.y = 0
    cube_pose.position.z = 0.025  # half of the cube's height

    # Add the cube's shape and pose to the collision object
    cube.primitives.append(cube_primitive)
    cube.primitive_poses.append(cube_pose)
    cube.operation = CollisionObject.ADD

    # Add the cube to the planning scene
    scene.add_object(cube)









    goals = [
        # [position, quaternion]
        # [x, y, z,  x, z, y, w]
        # [-4, -4, 0, 0, 0, 0, 1],
    ]
    dropoff_point = [-4, -4, 0]
    q = quaternion_from_euler(0, 0, -pi/2)
    goals.append([dropoff_point[0], dropoff_point[1], dropoff_point[2], q[0], q[1], q[2], q[3]])


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
    pub = rospy.Publisher('key_points', PoseArray, queue_size=10, latch=True)
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = 'map'
    pub.publish(pose_array)



    # send actions until the robot completes the path
    client = actionlib.SimpleActionClient('/mob_rob/move_base', MoveBaseAction)
    client.wait_for_server()

    for target in target_poses:
        goal = MoveBaseGoal()
        goal.target_pose = target

        client.send_goal(goal)
        wait = client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('car_path_publisher')
    move_car_to_dropoff()