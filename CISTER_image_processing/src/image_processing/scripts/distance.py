#!/usr/bin/env python
#-----------------------------------------------------------------------
#distance.py
#Codigo que controla a aceleracao, frenagem e angulacao do carro
#v0
#	versao original
#-----------------------------------------------------------------------

#Lista de imports
from image_processing.msg import coords
from image_processing.msg import drive_param
import rospy
import numpy as np

#Onde serao publicadas as informacoes
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1) 

#PID de velocidade
kp_vel=2					#Kp
ki_vel=0					#Ki
kd_vel=0					#Kd

#PID das rodas
kp_steer=5					#Original = 20	#Kp
ki_steer=0					#Ki
kd_steer=0					#Original = 5	#Kd

#parametros Iniciais
servo_offset = 18.0
velocity = 0.0
prev_error_steer=0.0
prev_error_vel=0.0
error_steer=0.0
error_vel=0.0
angle=0.0
velocity=0.0
count = 0

def callback(data):
	global prev_error_steer 
	global prev_error_vel
	global error_steer
	global error_vel
	global angle
	global velocity
	global velocity_number
	global cnt
	global count
	x = data.X		#posicao X da linha detectada
	y = data.Y		#posicao Y da linha detectada
	
	print "x: ", x
	
	#slope from image_processing
	#slope = data.Dist
	
	#if slope!=0:
	error_steer = (x-400)/400	#posicao X da linha em relacao ao centro da imagem 800 pixels
	print "Error: ",error_steer
	"""
	if np.isnan(distance)==False:
		error_vel = (distance-1)/float(1)
	else:
		error_vel = -4.0
	"""	
	
	
	control_error_steer = kp_steer*error_steer + kd_steer*(prev_error_steer - error_steer)#+ ki*integral
	print "Control Error Steer:", control_error_steer
	#angle = angle - control_error_steer#*np.pi/180
	angle = 0
	print "Count: ",count
	if (count <1000):
		velocity = 0.5
		count = count+1
	else:
		#count = count+1
		velocity = 0.0
	
	"""
	if (control_error_steer>19 or control_error_steer<-19):
		velocity = 0
	else:	
		velocity = 0
	"""
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
	print "-----------"
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
