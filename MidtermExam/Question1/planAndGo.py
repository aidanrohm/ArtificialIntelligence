#!/usr/bin/env python
# license removed for brevity
#
# Example program that calls a 
# global path planer and then moves the robot 
# dml 2016
#
#

import math
import sys
import numpy as np
import matplotlib.pyplot as plt

import rospy # needed for ROS

from geometry_msgs.msg import Vector3      
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist      # ROS Twist message

import pplib # our own special path planning module

#topics 

motionTopic='/cmd_vel'
poseTopic = '/odom' 

from tf.transformations import euler_from_quaternion

# global variables

gLoc = [0,0,0] # global for pose

gLogging=False # True will log the data
gTrackX=[] # log the
gTrackY=[] # actual track followed

#
# poseCallback
# this procedure is called to accept ROS pose topic info
#
def poseCallback(data):
    global gLoc,gTrackX,gTrackY
    gLoc[0] = data.pose.pose.position.x
    gLoc[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [axis for axis in [orient.x, orient.y, orient.z, orient.w]]
    (roll,pitch,yaw)=euler_from_quaternion(quat) 
    if yaw<0: # make the yaw angle positive only
        yaw+=2*math.pi   
    gLoc[2]=yaw  
    if gLogging:
        gTrackX.append(gLoc[0])
        gTrackY.append(gLoc[1])
    return


# Euclidean distance
def dist(x1,y1,x2,y2):
    delx = x2-x1
    dely = y2-y1
    d = math.sqrt( delx*delx+dely*dely)
    return d

# initialize subsriber and publisher, return publisher
def init_node():
    # register as a ROS publisher for the velocity topic 
    pub = rospy.Publisher(motionTopic, Twist, queue_size=0)
    # register as a subscriber for the pose topic
    rospy.Subscriber(poseTopic, Odometry, poseCallback)
    rospy.sleep(1) # wait for everything to start
    return pub


def goto_node(pub,goalx, goaly):
    '''bring turtlenot to location (goalx,goaly)'''

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
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return


#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        # all ROS 'nodes' (ie your program) have to do the following
        rospy.init_node('GotoTG_Node', anonymous=True)
        pub = init_node() # set up topics/callbacks
        start = [2.0,0.1]
        goal=[2.0, -4.0]
        goto_node(pub,start[0], start[1])
        path = pplib.planPath(start,goal)
        gLogging=True
        print("Starting path from ",start," to ",goal)
        
        plast=path[0]
        for p in path:
            if dist(p[0], p[1], plast[0], plast[1]) > 1.0:
                print("Going to waypoint:", p)
                goto_node(pub, p[0], p[1])
                plast = p
                
        if dist(goal[0], goal[1], plast[0], plast[1]) > 0.5:
            print("Going to final goal:", goal)
            goto_node(pub, goal[0], goal[1])    

        print("Finished Path, at ",goal)
        gLogging=False
        plt.scatter(gTrackY,gTrackX)
        plt.gca().invert_yaxis()
        plt.show()
        
    except rospy.ROSInterruptException:
        pass
