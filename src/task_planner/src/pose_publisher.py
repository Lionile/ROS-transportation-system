#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
import tf2_ros
import geometry_msgs.msg

last_timestamp = None


def callback(data):
    global last_timestamp
    try:
        # find the index of the object by name
        object_index = data.name.index('cube')
        pose = data.pose[object_index]

        # Check if the pose has changed
        if last_timestamp is not None and last_timestamp == rospy.Time.now():
            return
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "cube"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        br.sendTransform(t)
        last_timestamp = t.header.stamp
    except ValueError:
        print("Object not in list")

rospy.init_node('gazebo_object_pose_publisher')
rospy.sleep(2)
br = tf2_ros.TransformBroadcaster()

sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

rospy.spin()