#!/usr/bin/env python
# license removed for brevity
#
# Example program that moves
# the robot to a location goalx,goaly
# using the command line
# nbp 2021
#
# Aidan Rohm ----- Modifications made to this file in accordance to Homework 2 ----- 2/19/26

import math
import sys
import rospy  # needed for ROS
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist      # ROS Twist message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan               # ROS laser msg
from tf.transformations import euler_from_quaternion  # Manipulate angles

# topics
laserTopic = '/scan'        # Name for the laser scan topic
motionTopic = '/cmd_vel'
poseTopic = '/odom'

# global robot pose
gLoc = [0, 0, 0]  # [x, y, theta(yaw radians)]

# -------------------- MAP ARRAY CONSTRAINTS --------------------
# A small, simple occupancy "hit map" like the class example:
# 1 = surface detected at that cell, 0 = no surface detected
MAP_RES = 0.10          # meters per cell (10 cm)
MAP_W_CELLS = 120       # map width in cells
MAP_H_CELLS = 120       # map height in cells

MAP_W_M = MAP_W_CELLS * MAP_RES
MAP_H_M = MAP_H_CELLS * MAP_RES

occ_map = np.zeros((MAP_H_CELLS, MAP_W_CELLS), dtype=np.uint8)

# Anchoring the map to the robot's starting pose (in odom frame)
map_anchor_set = False
ANCHOR_X = 0.0      # Temporary initial value, will be set to robot's starting x position
ANCHOR_Y = 0.0      # Temporary initial value, will be set to robot's starting y position


# -------------------- (1) LASER -> CARTESIAN COORDINATES --------------------
def toCartesian(x, y, theta, angle_rad, d):
    """
    Takes robot pose (x,y,theta) and a single laser reading (angle_rad, d),
    and returns the cartesian coordinates (X,Y) of the detected surface in the map frame.

    NOTE: angle_rad must be in radians.
    """
    global_angle = theta + angle_rad
    X = x + d * math.cos(global_angle)
    Y = y + d * math.sin(global_angle)
    return X, Y


def world_to_map(X, Y):
    """
    Translate world/map coordinates (X,Y) in meters into an index (r,c) in the numpy map array.
    Returns (r,c) if in bounds, otherwise None, this prevents noisy coordinates
    """
    global map_anchor_set, ANCHOR_X, ANCHOR_Y

    if not map_anchor_set:
        return None

    # Map origin is anchored around the robot's starting pose
    origin_x = ANCHOR_X - MAP_W_M / 2.0
    origin_y = ANCHOR_Y - MAP_H_M / 2.0

    c = int((X - origin_x) / MAP_RES)
    r = int((Y - origin_y) / MAP_RES)

    if 0 <= r < MAP_H_CELLS and 0 <= c < MAP_W_CELLS:
        return (r, c)
    return None


# -------------------- (2) LASER CALLBACK --------------------
def callback_laser(msg):
    '''Call back function for laser range data'''
    global occ_map, gLoc, map_anchor_set, ANCHOR_X, ANCHOR_Y

    x, y, theta = gLoc[0], gLoc[1], gLoc[2]

    # Set the anchor once using the robot's starting pose (in odom frame)
    if not map_anchor_set:
        ANCHOR_X = x
        ANCHOR_Y = y
        map_anchor_set = True

    # Loop for each range value from the sensor
    for i, reading in enumerate(msg.ranges):

        if not math.isnan(reading) and not math.isinf(reading) and reading > 0:
            
            # ROS laser angles come in radians
            angle_rad = msg.angle_min + i * msg.angle_increment

            # Determine the coordinates of the detected surface in the map frame
            X, Y = toCartesian(x, y, theta, angle_rad, reading)

            # Safety check
            if not (math.isfinite(X) and math.isfinite(Y)):
                continue

            # Translate the world coordinates (X,Y) into map array indices (r,c)
            rc = world_to_map(X, Y)
            if rc is None:
                continue

            r, c = rc
            occ_map[r, c] = 1  # Mark the cell as occupied

    return


# -------------------- POSE CALLBACK --------------------
def poseCallback(data):
    global gLoc
    gLoc[0] = data.pose.pose.position.x
    gLoc[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat)
    if yaw < 0:  # make the yaw angle positive only
        yaw += 2 * math.pi
    gLoc[2] = yaw
    return


# Euclidean distance
def dist(x1, y1, x2, y2):
    delx = x2 - x1
    dely = y2 - y1
    return math.sqrt(delx * delx + dely * dely)


# -------------------- GOTO NODE --------------------
def gotoTG_node(goalx, goaly):
    """bring turtlebot to location (goalx,goaly)"""

    rospy.init_node('GotoTG_Node', anonymous=True)

    # publisher for velocity
    pub = rospy.Publisher(motionTopic, Twist, queue_size=0)

    # subscribers for pose and laser
    rospy.Subscriber(poseTopic, Odometry, poseCallback)
    rospy.Subscriber(laserTopic, LaserScan, callback_laser)

    rospy.sleep(1)  # wait for everything to start

    rate = rospy.Rate(20)  # Hz
    msg = Twist()
    ctr = 0

    while not rospy.is_shutdown() and dist(goalx, goaly, gLoc[0], gLoc[1]) > 0.5:

        targetTheta = math.atan2(goaly - gLoc[1], goalx - gLoc[0])
        delTheta = targetTheta - gLoc[2]  # angular error

        # pick smallest way to turn
        if delTheta > 0:
            altTheta = delTheta - 2 * math.pi
        else:
            altTheta = delTheta + 2 * math.pi
        if abs(delTheta) > abs(altTheta):
            delTheta = altTheta

        # occasional status
        if ctr > 20:
            print(round(gLoc[0], 2), round(gLoc[1], 2))
            ctr = 0
        else:
            ctr += 1

        # simple proportional control
        msg.angular.z = 1.0 * delTheta
        delDist = dist(goalx, goaly, gLoc[0], gLoc[1])
        msg.linear.x = 0.1 * delDist

        pub.publish(msg)
        rate.sleep()

    print("Done ", "d=", round(dist(goalx, goaly, gLoc[0], gLoc[1]), 2))
    print(" Loc= ", round(gLoc[0], 2), round(gLoc[1], 2))

    # stop the robot
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)

    # ------------------------- (4) DISPLAY THE OCCUPANCY MAP -------------------------
    plt.figure()
    plt.title("Occupancy hit map (1 = Surface detected, 0 = No surface detected)")
    plt.imshow(occ_map, origin='lower', interpolation='nearest')
    plt.xlabel("Map X-axis (cells)")
    plt.ylabel("Map Y-axis (cells)")
    plt.show()

    return


def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=0)
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub.publish(msg)
    rospy.sleep(1)
    return


# ------------------------------- MAIN ----------------------
if __name__ == '__main__':
    rospy.on_shutdown(callback_shutdown)
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        print("Moving to ", x, y)
        gotoTG_node(x, y)
    except rospy.ROSInterruptException:
        pass

