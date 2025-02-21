#!/usr/bin/env python3
import rospy
import tf
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

rospy.init_node("collision_detection")

collision_trigger_distance = rospy.get_param('collision_detection/collision_trigger_distance')

def laser_callback(msg):
    ranges = msg.ranges
    min_range = min(ranges)

    if min_range <= collision_trigger_distance:
        pb.publish(True)
    else:
        pb.publish(False)
        


pb = rospy.Publisher('/collision', Bool, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, laser_callback)


rospy.spin()
