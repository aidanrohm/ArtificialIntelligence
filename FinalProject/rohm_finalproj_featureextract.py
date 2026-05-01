#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan    # ROS laser msg
from tf.transformations import euler_from_quaternion # manipulate angles

from std_msgs.msg import String
import json

class TerrainFeatureNode:

    def __init__(self):
        rospy.init_node("terrain_features")

        self.cmd_vel = 0.0
        self.odom_vel = 0.0
        self.closest = (0,0)
        self.location = (0,0)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.pub = rospy.Publisher("/terrain_features", String, queue_size=10)
        rospy.Timer(rospy.Duration(0.2), self.publish)

    def cmd_callback(self,msg):
        self.cmd_vel = msg.linear.x

    def odom_callback(self,msg):
        self.odom_vel = msg.twist.twist.linear.x
        self.location = (msg.pose.pose.position.x,msg.pose.pose.position.y)

    def laser_callback(self,msg):
       filtered = [ (i,v) for (i,v) in enumerate(msg.ranges)\
                                     if not np.isnan(v) and not np.isinf(v) and v>0.1]
       self.closest = min(filtered,key=lambda x: x[1])

    def publish(self,event):

        slip = abs(self.cmd_vel - self.odom_vel)
       
        msg = {
            "slip": slip,
            "pose": self.location
        }
        
        print(msg)
        self.pub.publish(json.dumps(msg))

if __name__ == "__main__":
    TerrainFeatureNode()
    rospy.spin()
