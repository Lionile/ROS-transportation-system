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
from move_car_to_cube import move_car_to_cube
from move_car_to_dropoff import move_car_to_dropoff
from arm_set_cube import arm_set_cube
from arm_grab_cube import arm_grab_cube
from arm_drop_cube import arm_drop_cube
from arm_goto_cube import arm_goto_cube
from cube_respawn import respawn_cube


def run_task():
    arm_goto_cube()
    arm_grab_cube()
    arm_set_cube()
    move_car_to_cube()
    arm_drop_cube()
    move_car_to_dropoff()
    respawn_cube()

if __name__ == '__main__':
    rospy.init_node('task_planner')
    while not rospy.is_shutdown():
        run_task()