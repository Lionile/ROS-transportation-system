#!/usr/bin/env python3
import rospy
import rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
import time

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_service(model_name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Delete Model service call failed: {e}")

def spawn_model(model_name, model_xml, model_pose):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_model_service(model_name, model_xml, "", model_pose, "map")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn Model service call failed: {e}")

def respawn_cube():
    model_name = 'cube'
    package_path = rospkg.RosPack().get_path('task_planner')
    model_path = f'{package_path}/urdf/cube.urdf'
    with open(model_path, 'r') as model_file:
        model_xml = model_file.read()

    model_pose = Pose()
    model_pose.position.x = -0.3
    model_pose.position.y = 0
    model_pose.position.z = 0.15
    
    delete_model(model_name)
    time.sleep(1)
    spawn_model(model_name, model_xml, model_pose)

if __name__ == '__main__':
    rospy.init_node('cube_respawn')

    respawn_cube()