#!/usr/bin/env python
# license removed for brevity
#
# Example program that moves
# the robot to a location goalx,goaly
# using the command line
# nbp 2021
#
#

# Aidan Rohm ----- Modifications made to this file in accordance to Homework 2 ----- 2/19/26

import math
import sys
import rospy # needed for ROS
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist      # ROS Twist message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan			        # ROS laser msg
from tf.transformations import euler_from_quaternion 	# Manipulate angles

#topics 
laserTopic = '/scan' 		# Name for the laser scan topic
motionTopic='/cmd_vel'
poseTopic = '/odom' 

# global variable
gLoc = [0,0,0]    # pose of the robot

#
# laserCallback
# This procedure is called to accept ROS Laser topic info
#
def callback_laser(msg):
    '''Call back function for laser range data'''
    # Loop for each range value from the sensor
    for i, reading in enumerate(msg.ranges):
        if not math.isnan( reading ) and not math.isinf( reading) and reading > 0:
            # TODO: COMPLETE NECESSARY CODE HERE
    return

#
# poseCallback
# this procedure is called to accept ROS pose topic info
#
def poseCallback(data):
    global gLoc
    gLoc[0] = data.pose.pose.position.x
    gLoc[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll,pitch,yaw)=euler_from_quaternion(quat)
    if yaw<0: # make the yaw angle positive only
        yaw+=2*math.pi    
    gLoc[2]=yaw
    return

# Euclidean distance
def dist(x1,y1,x2,y2):
    delx = x2-x1
    dely = y2-y1
    d = math.sqrt( delx*delx+dely*dely)
    return d
    
def toCartesian(x, y, theta, a, d):
    '''Convert a single laser reading (angle a and distance d) into map related coordinates'''
    
    # Convert the laser angle to radians
    a_rad = math.radians(a)
    
    # Point in the robot's frame
    x_r = d * math.cos(a_rad)
    y_r = d * math.sin(a_rad)
    
    # Rotate into the map frame and translate
    X = x + x_r * math.cos(theta) - y_r * math.sin(theta)
    Y = y + x_r * math.sin(theta) + y_r * math.sin(theta)
    
    return X, Y
    

# goto_node
# this procedure generates the velocity commands
# to move the turtlebot (TG) to a goal x and y 
#

def gotoTG_node(goalx, goaly):
    '''bring turtlenot to location (goalx,goaly)'''

    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('GotoTG_Node', anonymous=True)

    # register as a ROS publisher for the velocity topic 
    pub = rospy.Publisher(motionTopic, Twist, queue_size=0)
    # register as a subscriber for the pose topic
    rospy.Subscriber(poseTopic, Odometry, poseCallback)
    scan_sub = rospy.Subscriber(laserTopic, LaserScan, callback_laser)
    rospy.sleep(1) # wait for everything to start

    # this is how frequently the message is published
    rate = rospy.Rate(20) # Hz
    msg = Twist() # empty new velocity message
    ctr = 0       # counter used to produce messages
    angularGain = 1.0 #  gain for angular velocity error
    linearGain = 0.1  #  gain for linear velocity error
    while not rospy.is_shutdown() and dist(goalx,goaly,gLoc[0], gLoc[1])>0.5:

        targetTheta = math.atan2(goaly-gLoc[1], goalx-gLoc[0])
        delTheta =  targetTheta - gLoc[2] # angular error
        if delTheta>0: # pick smallest way to turn
            altTheta = delTheta - 2*math.pi
        else:
            altTheta = delTheta + 2*math.pi
        if abs(delTheta)>abs(altTheta):
            delTheta = altTheta
        # allow for occasional status
        if ctr>20:
            print (round(gLoc[0],2), round(gLoc[1],2) )
            ctr=0
        else:
            ctr = ctr+1
            
        msg.angular.z = 1.0 * delTheta # should saturate angular velocity
        delDist = dist(goalx,goaly,gLoc[0], gLoc[1]) # distance error
        msg.linear.x = 0.1 * delDist # should saturate linear velocity

        pub.publish(msg)
        rate.sleep()
    print  ("Done ","d=",round(dist(goalx,goaly,gLoc[0], gLoc[1]),2))
    print  (" Loc= ",round(gLoc[0],2), round(gLoc[1],2)) 
    
    #reset the velocities to 0
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)
    return
    
def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=0)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(1)
    return


#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    rospy.on_shutdown(callback_shutdown)
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        print("Moving to ",x,y)
        gotoTG_node(x,y)
    except rospy.ROSInterruptException:
        pass
