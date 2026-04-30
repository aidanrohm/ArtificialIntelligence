# Aidan Rohm
# Professor Lyons
# Artificial Intelligence
# Final Project, May 2 2026

"""
coverage_mover.py

Drives the turtlebot over the full enclosure area using a lawnmower sweep pattern in the hopes that every grid cell
is visited at least once

Strategy
============================
	* Divide the driveable area into vertical strips spaced LANE_SPACING apart
	* Drive each strip from one end to the other, then step sideways and reverse
	* Use a simple proportional heading controller to stay on course
	* Stop and rotate in place when transitioning between strips
	* Obstacle avoidance via laser scan -- back up and turn if too close
"""

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

# ── Coverage plan parameters ──────────────────────────────────────────────────
WORLD_X_MIN    = -3.2   # slightly inside the barriers
WORLD_X_MAX    =  3.2
WORLD_Y_MIN    = -4.8
WORLD_Y_MAX    =  1.5
 
LANE_SPACING   = 0.45   # meters between parallel strips (< cell size → overlap)
WAYPOINT_TOL   = 0.25   # meters -- "arrived" threshold
TURN_TOL       = 0.05   # radians -- heading tolerance for in-place turns
 
# ── Speed parameters ─────────────────────────────────────────────────────────
LINEAR_SPEED   = 0.15   # m/s forward
TURN_SPEED     = 0.5    # rad/s for in-place rotation
HEADING_KP     = 1.2    # proportional gain for heading correction while driving
 
# ── Safety ───────────────────────────────────────────────────────────────────
SAFE_DIST      = 0.45   # meters -- minimum clearance to obstacles

def wrap_angle(a):
	"""Wrap angle to [-pi, pi]"""
	return (a + math.pi) % (2 * math.pi) - math.pi
	
def build_waypoints():
	"""
	Build a coverage path as a list of (x, y) waypoints
	Sweeps along x-strips varying y, then steps to the next x strip
	"""
	
	xs = list(np.arrage(WORLD_X_MIN, WORLD_X_MAX + LANE_SPACING, LANE_SPACING))
	waypoints = []
	go_forward = True
	
	for i, x in enumerate(xs):
		if go_forward:
			y_start, y_end = WORLD_Y_MIN, WORLD_Y_MAX
		else:
			y_start, y_emd = WORLD_Y_MAX, WORLD_Y_MIN
			
		# Start of this strip
		waypoints.append((x, y_start))
		# End of this strip
		waypoints.append((x, y_end))
		
		# Step to the next strip if it is not last
		if i + 1 < len(xs):
			waypoints.append((xs[i + 1], y_end))
			
		go_forward = not go_forward
		
	return waypoints
	
class CoverageMover:
	
	def __init__(self):
		rospy.init_node("coverage_mover")
		
		self.loc = [0.0, 0.0, 0.0] 	# [x, y, yaw]
		self.scan = []			# Latest laser ranges
		
		self.pub = rospy.Publisher("/cmd_ve", Twist, queue_size=1)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		rospy.on_shutdown(self.shutdown)
		
		self.waypoints = build_waypoints()
		rospy.loginfo(f"Coverage plan: {len(self.waypoints)} waypoints")
		
	# ── Sensor callbacks ──────────────────────────────────────────────────
	def odom_callback(self, msg):
		orient = msg.pose.pose.orientation
		q = [orient.x, orient.y, orient.z, orient.w]
		_, _, yaw = euler_from_quaternion(q)
		self.loc = [
			msg.pose.pose.position.x,
			msg.pose.pose.position.y,
			yaw
		]
 
	def scan_callback(self, msg):
		self.scan = [v for v in msg.ranges if not math.isnan(v) and not math.isinf(v) and v > 0.05]
		
	# Low level motion primitives
	
	def stop(self):
		self.pub.publish(Twist())
 
	def is_obstacle_ahead(self):
		"""True if any laser reading within plus/minus 30 deg forward is under SAFE_DIST."""
		if not self.scan:
			return False
		n = len(self.scan)
		front_indices = list(range(0, n // 12)) + list(range(11 * n // 12, n))
		front = [self.scan[i] for i in front_indices if i < len(self.scan)]
		return bool(front) and min(front) < SAFE_DIST
 
	def rotate_to(self, target_yaw, rate):
		"""Rotate in place until heading matches target_yaw within TURN_TOL."""
		while not rospy.is_shutdown():
			err = wrap_angle(target_yaw - self.loc[2])
			if abs(err) < TURN_TOL:
				break
			twist = Twist()
			twist.angular.z = TURN_SPEED * np.sign(err)
			self.pub.publish(twist)
			rate.sleep()
		self.stop()
 
	def drive_to(self, gx, gy, rate):
		"""
		Drive toward waypoint (gx, gy) with proportional heading control.
		Handles obstacle avoidance by stopping and rotating to escape.
		Returns when within WAYPOINT_TOL of goal or rospy shuts down.
		"""
		while not rospy.is_shutdown():
			dx = gx - self.loc[0]
			dy = gy - self.loc[1]
			dist = math.hypot(dx, dy)
 
			if dist < WAYPOINT_TOL:
				break
 
			desired_yaw = math.atan2(dy, dx)
			heading_err = wrap_angle(desired_yaw - self.loc[2])
 
			# Obstacle avoidance
			if self.is_obstacle_ahead():
				rospy.logwarn("Obstacle detected -- backing up")
				self.backup_and_turn()
				continue
 
			twist = Twist()
			# Slow down near goal
			twist.linear.x  = min(LINEAR_SPEED, dist * 0.8)
			twist.angular.z = HEADING_KP * heading_err
			self.pub.publish(twist)
			rate.sleep()
 
		self.stop()
 
	def backup_and_turn(self):
		"""Simple obstacle escape: back up briefly, then turn."""
		twist = Twist()
		twist.linear.x = -0.1
		t_start = rospy.Time.now()
		rate = rospy.Rate(20)
		while (rospy.Time.now() - t_start).to_sec() < 1.0 and not rospy.is_shutdown():
			self.pub.publish(twist)
			rate.sleep()
		self.rotate_to(self.loc[2] + math.pi / 2, rate)
		
	# ── Main coverage loop ──────────────────────────────────────────────────
	
	def run(self):
		rate = rospy.Rate(20)
 
		# Wait for sensors to come online
		rospy.loginfo("Waiting for odometry...")
		rospy.sleep(2.0)
 
		rospy.loginfo("Starting coverage sweep")
 
		for idx, (gx, gy) in enumerate(self.waypoints):
			if rospy.is_shutdown():
				break
 
			rospy.loginfo(f"Waypoint {idx + 1}/{len(self.waypoints)}: "
						  f"({gx:.2f}, {gy:.2f})  "
						  f"robot at ({self.loc[0]:.2f}, {self.loc[1]:.2f})")
 
			# 1. Face the waypoint
			dx = gx - self.loc[0]
			dy = gy - self.loc[1]
			if math.hypot(dx, dy) > WAYPOINT_TOL:
				desired_yaw = math.atan2(dy, dx)
				self.rotate_to(desired_yaw, rate)
 
			# 2. Drive to the waypoint
			self.drive_to(gx, gy, rate)
 
		rospy.loginfo("Coverage sweep complete!")
		self.stop()
 
	def shutdown(self):
		rospy.loginfo("Shutting down coverage mover")
		self.stop()
		rospy.sleep(0.5)
 
 
if __name__ == "__main__":
	node = CoverageMover()
	node.run()
	
