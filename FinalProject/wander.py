#!/usr/bin/env python
# 
# Wander untill close to wall
#
import rospy
import numpy as np
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan    # ROS laser msg
from tf.transformations import euler_from_quaternion # manipulate angles

class WanderNode:

    def __init__(self):
        rospy.init_node("wander_node")
        self.closest = (0,0)
        self.loc     = [0,0,0]      
        self.publish=rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.on_shutdown(self.callback_shutdown)
        rospy.Timer(rospy.Duration(1.0), self.log)

    def odom_callback(self,msg):
         orient = msg.pose.pose.orientation
         quat = [orient.x, orient.y, orient.z, orient.w]
         (roll,pitch,yaw)=euler_from_quaternion(quat)
         if yaw<0: # make the yaw angle positive only
             yaw+=2*math.pi    
         self.loc = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]

    def laser_callback(self,msg):
       filtered = [ (i,v) for (i,v) in enumerate(msg.ranges)\
                                     if not np.isnan(v) and not np.isinf(v) and v>0.1]
       self.closest = min(filtered,key=lambda x: x[1])

    def log(self,event):
        print(f"X={self.loc[0]:.2f} Y={self.loc[1]:.2f}")

    def run(self):
        # Simple mindded random wandering
        msg = Twist()
        msg.linear.x = 0.1
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            a,d = self.closest
            msg.angular.z=0.0
            msg.linear.x = 0.1 # m per sec
            if d<1: # too close
                if 270<a<330:
                    msg.angular.z = 0.5 # rad per sec
                elif 30<a<90:
                    msg.angular.z = -0.5
                elif 0<a<30 or 330<a<359:
                    msg.angular.z = -0.5
                    msg.linear.x = -1
            self.publish.publish(msg)
            rate.sleep()

    def callback_shutdown(self):
        print("Shutting down")
        msg = Twist()
        msg.angular.z=0.0
        msg.linear.x=0.0
        self.publish.publish(msg) 
        rospy.sleep(1)
            
            
if __name__ == "__main__":
     node = WanderNode()
     node.run()
