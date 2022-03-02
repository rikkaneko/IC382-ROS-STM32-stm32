#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0
theta = 0.0
state = 0


def newOdom(msg):
	global x
	global y
	global theta
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()

while not rospy.is_shutdown():
	
	if state == 0:
		goal.x = 13
		goal.y = 0
	if state == 1:
		goal.x = 15
		goal.y = 2
	if state == 2:
		goal.x = 15
		goal.y = 6
	if state == 3:
		goal.x = 14
		goal.y = 8
	if state == 4:
		goal.x = 9
		goal.y = 8
	if state > 4:
		goal.x = 0
		goal.y = 0
	
	inc_x = goal.x - x
	inc_y = goal.y - y
	
	angle_to_goal = atan2(inc_y, inc_x)
	
	if angle_to_goal - theta > 0.05:
		speed.linear.x = 0.2
		speed.angular.z = 0.3
	elif angle_to_goal - theta < -0.05:
		speed.linear.x = 0.2
		speed.angular.z = -0.3
	else:
		speed.linear.x = 0.6
		speed.angular.z = 0.0
	
	if abs(inc_x) < 0.5 and abs(inc_y) < 1:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		state += 1
	if state > 4:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
	
	pub.publish(speed)
	r.sleep()
