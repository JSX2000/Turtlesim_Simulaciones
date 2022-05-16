#! /usr/bin/env python

from decimal import localcontext
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty


#Position
x = 0
y = 0
theta = 0

#turtle2 position
x2 = 0
y2 = 0
theta2= 0

def poseCallback2(pose_message):
	global x
	global y
	global theta

	x = pose_message.x
	y = pose_message.y
	theta = pose_message.theta

def poseCallback1(pose_message):
	global x2
	global y2
	global theta2

	x2 = pose_message.x
	y2 = pose_message.y
	theta2 = pose_message.theta
	#print ('x2=', x2, 'y2=', y2)
	
def orientate (xgoal, ygoal):
	global x
	global y
	global theta

	velocity_message = Twist()
	cmd_vel_topic = '/turtle2/cmd_vel'

	while(True):
		ka = 4.0
		desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
		dtheta = desired_angle_goal-theta        
		angular_speed = ka * (dtheta)

		velocity_message.linear.x = 5.0
		velocity_message.angular.z = angular_speed
		velocity_publisher.publish(velocity_message)
		#print ('x=', x, rvice ca'y=', y)

		if (dtheta < 1.5):
		    break

def circle(radius):
	global x
	global y
	global z
	global theta
	rate = 10
	vel = Twist()
	rate = rospy.Rate(rate)
	while not rospy.is_shutdown():    
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = np.pi
		vel.linear.x = radius*vel.angular.z
		vel.linear.y = 0
		vel.linear.z = 0
		velocity_publisher.publish(vel)
		rate.sleep()
	

def go_to_goal (xgoal, ygoal):
	global x
	global y
	global theta

	velocity_message = Twist()
	cmd_vel_topic = '/turtle2/cmd_vel'

	while(True):
		kv = 1
		distance_turtle = abs(math.sqrt(((x2-x)**2)+((y2-y)**2)))
		distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))       
		linear_speed = kv * distance

		if distance_turtle < 4:
			orientate(1, 6)

		ka = 4.0
		desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
		dtheta = desired_angle_goal-theta        
		angular_speed = ka * (dtheta)

		velocity_message.linear.x = linear_speed
		velocity_message.angular.z = angular_speed
		velocity_publisher.publish(velocity_message)
		#print ('x=', x, 'y=', y)

if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim_motion_pose', anonymous = True)

		cmd_vel_topic = '/turtle2/cmd_vel'
		velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

		position_topic2 = "/turtle2/pose"
		pose_subscriber2 = rospy.Subscriber(position_topic2, Pose, poseCallback2)

		position_topic1 = "/turtle1/pose"
		pose_subscriber1 = rospy.Subscriber(position_topic1, Pose, poseCallback1)
		time.sleep(1)

		time.sleep(0.1)
		orientate(6,10)
		time.sleep(0.1)
		go_to_goal(6,10)
		time.sleep(0.1)

		# while True:
		# 	t2_p = np.array([x, y])
		# 	t1_p = np.array([x2, y2])
		# 	d = np.linalg.norm(t1_p - t2_p)
		# 	if d < 2:
		# 		time.sleep(0.1)
		# 		orientate(x+0.1,y+0.1)
		# 		time.sleep(0.1)
		# 		go_to_goal(x+0.1,y+0.1)
		# 		time.sleep(0.1)	
		# 		break

	except rospy.ROSInterruptException:        
		pass
