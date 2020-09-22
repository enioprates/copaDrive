#!/usr/bin/env python

from image_processing.msg import coords
from image_processing.msg import drive_param
import rospy
import numpy as np


pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
kp_vel=2
ki_vel=0
kd_vel=0

kp_steer=20
ki_steer=0
kd_steer=5

servo_offset = 18.0
velocity = 0.0
prev_error_steer=0.0
prev_error_vel=0.0
error_steer=0.0
error_vel=0.0
angle=0.0
velocity=0.0

def callback(data):
	global prev_error_steer 
	global prev_error_vel
	global error_steer
	global error_vel
	global angle
	global velocity
	global velocity_number
	global cnt
	x = data.X
	y = data.Y
	
	#slope from image_processing
	#slope = data.Dist
	
	#if slope!=0:
	error_steer = (x-400)/400
	"""
	if np.isnan(distance)==False:
		error_vel = (distance-1)/float(1)
	else:
		error_vel = -4.0
	"""	
	
	
	control_error_steer = kp_steer*error_steer + kd_steer*(prev_error_steer - error_steer)#+ ki*integral
	print "Control Error Steer:", control_error_steer
	angle = angle - control_error_steer#*np.pi/180
	
	velocity = 25
	"""
	control_error_vel = kp_vel*error_vel #+ kd_vel*(prev_error_vel - error_vel)
	print "Control Error Vel:", control_error_vel
	velocity = velocity + (control_error_vel)
	nan_verify = velocity

	if np.isnan(velocity)==False:
		velocity_number = velocity
	
	if np.isnan(velocity)==True:
		if nan_verify == velocity:
			cnt = cnt+1
			if cnt==5:
				velocity = 0
				print "RESET"
				cnt=0		
		else:
			velocity = velocity_number
	"""
	
	
	
	prev_error_steer = error_steer
	#prev_error_vel = error_vel

	msg = drive_param()
	if velocity<0:
		velocity = 0
	#angle = 30/float(-320) * x + 30
	#angle = -angle
	if angle > 30:
		angle = 30
	if angle < -30:
		angle = -30
	print "angle:",angle

	#if distance <= 1:
	#	velocity = 0
	#if distance > 1:
	#	velocity = distance * 10	
	print "vel:",velocity
	#if velocity > 30:
	#	velocity=30
	msg.velocity = velocity
	msg.angle = angle
	pub.publish(msg)	


def listener():
	rospy.init_node('vel_angle_calc', anonymous=True)
	rospy.Subscriber('X_Y', coords, callback)
	rospy.spin()

   
if __name__ == '__main__':
	listener()
