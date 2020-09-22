#!/usr/bin/env python
#-----------------------------------------------------------------------
#Simulation_connector.py
#Codigo responsavel pela ligacao entre o sistema de controle e o simulador do veiculo
#v0
#	Codigo alterado para ler os dados de velocidade do topico carINFO
#	Criada a funcao car_info
#v1
#	Incorporado o codigo do distance.py
#v2
#	Convertendo velocidade em aceleracao e frenagem
#-----------------------------------------------------------------------

#Import commands
import rospy
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
from image_processing.msg import AckermannDriveStamped
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math
import numpy as np
#------------------------------------------------------------------------

#Publication Topic
pub = rospy.Publisher('car1/prius', Control, queue_size=10)	#topico de controle do carro
pub_2 = rospy.Publisher('drive_parameters', drive_param, queue_size=1) 
#------------------------------------------------------------------------

#global variables  
speed_real = 0.0											#velocidade global do veiculo
#PID de velocidade
kp_vel=2				#Kp
ki_vel=0				#Ki
kd_vel=0				#Kd

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
count = 0					#variavel de teste de numero de iteracoes
MAX_SPEED = 30.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 30.0			#maxima angulacao da roda em graus
MIN_STEER = -30.0			#minima angulacao da roda em graus
#-----------------------------------------------------------------------

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(data)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(s_sp, s_real):
	
	erro_vel = s_sp/3.6 - s_real						#calcula o erro de velocidade -> "/3.6" converete para m/s
	print "erro_vel=", erro_vel				
	erro_vel_norm = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	
														#normaliza o erro (0 - 1)
	print "erro_vel_norm=", erro_vel_norm
	accel_control = min(erro_vel_norm,1.0)				#acao de controle
	print "accel_control=", accel_control
	accel_control = max(accel_control,-1.0)				#acao de controle
	print "accel_control=", accel_control

	if(abs(accel_control) < 0.05 and s_sp == 0.0):		#necessario para zerar a velocidade
		aceleracao = 0.0
		freio = 1.0
	else:
		aceleracao = max(0.0,accel_control)				#acao de aceleracao
		freio = max(0.0,-accel_control)					#acao de frenagem
	return aceleracao, freio

#-----------------------------------------------------------------------
#FUNCAO: vel_and_angle(data)
#	Funcao chamada pelo topico de parametros desejados do veiculo (drive_parameters)
#	Define variaveis de controle: 
# 	aceleracao - throttle
# 	frenagem - Brake 
# 	Angulo das rodas: steer
#	Publica no topico "car1/prius"
#-----------------------------------------------------------------------
def vel_and_angle(data):
	global speed_real										#chamada para a variavel global
	msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
	msg.header.stamp = rospy.Time.now();					#tempo atualizado da leitura
	msg.header.frame_id = "base_link";						
	#msg.throttle = data.velocity							#aceleracao desejada 	(0-1)
	#msg.brake = data.brake									#frenagem desejada 		(0-1)
	msg.steer = data.angle									#angulacao desejada para as rodas (0-30) GRAUS OU RADIANOS?
	
	print "------------------------------------------"
	print "vel=", data.velocity
	print "speed_real=", speed_real
	
	"""
	erro_vel = data.velocity/3.6 - speed_real
	print "erro_vel=", erro_vel
	erro_vel_norm = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)
	print "erro_vel_norm=", erro_vel_norm
	accel_control = min(erro_vel_norm,1.0)
	print "accel_control=", accel_control
	accel_control = max(accel_control,-1.0)
	print "accel_control=", accel_control

	if(abs(accel_control) > 0.05):
		msg.throttle = max(0.0,accel_control)
		msg.brake = max(0.0,-accel_control)
	elif (data.velocity == 0.0):
		msg.throttle = 0.0
		msg.brake = 1.0
	"""
	msg.throttle, msg.brake = calc_throttle_brake (data.velocity, speed_real)
	
	#if data.velocity == 0:
	#	msg.brake = 1
	
	print "Throttle=",msg.throttle
	print "brake=",msg.brake 
	#print "angle=",data.angle
	pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'

#-----------------------------------------------------------------------
#FUNCAO: car_info(data)
#	Funcao chamada pelo topico de dados atuais do veiculo ('/car1/carINFO')
#-----------------------------------------------------------------------
def car_info(data):
	global speed_real										#chamada para a variavel global
	speed_real = data.speed_speedValue						#salva o valor da velocidade no instante
	print "real_vel=", speed_real
  #inputs (ros_its_msgs::CAM_simplified)
  #msg->latitude (float32)
  #msg->longitude (float64)
  #msg->altitude_altitudeValue (float32)
  #msg->heading_headingValue (float32)
  #msg->speed_speedValue (float32)
  #msg->driveDirection (int8)
  #msg->steeringWheelAngle_steeringWheelAngleValue (float32)
  #msg->gasPedalPercent_Value (float32)
  #msg->brakePedalPercent_Value (float32)
  	#msg = Control() 
  	#msg.car_info_global = data.speed_speedValue;
  #getTime();
  
  #ROS_INFO("carINFO");

#-----------------------------------------------------------------------
#FUNCAO: vehicle_control(data)
#	Le os dados de posicao da camera ('X_Y')
#-----------------------------------------------------------------------
def vehicle_control(data):
	global prev_error_steer 
	global prev_error_vel
	global error_steer
	global error_vel
	global angle
	global velocity
	global velocity_number
	global cnt
	global count											#variavel de teste de numero de iteracoes
	x = data.X												#posicao X da linha detectada
	y = data.Y												#posicao Y da linha detectada
	
	#print "x: ", x											#posicao X da linha detectada
	

	error_steer = (x-400)/400								#posicao X da linha em relacao ao centro da imagem 800 pixels
	#print "Error: ",error_steer
	
	#sistema de controle de angulacao
	control_error_steer = kp_steer*error_steer + kd_steer*(prev_error_steer - error_steer)#+ ki*integral
	#print "Control Error Steer:", control_error_steer
	#angle = angle - control_error_steer#*np.pi/180
	angle = 0
	prev_error_steer = error_steer
	#prev_error_vel = error_vel

	#angle = 30/float(-320) * x + 30
	#angle = -angle
	if angle > 30:
		angle = 30
	if angle < -30:
		angle = -30
	print "angle:",angle



	#sistema de controle de velocidade
	print "Count: ",count
	if (count <250):
		velocity = 10.0
		count = count+1
	elif(count>250 and count<10000):
		velocity =10.0
		count = count+1
	else: 
		velocity =10.0
	
	print "vel:",velocity
	print "-----------"

	#prepara msg para publicacao
	msg = drive_param()
	msg.velocity = velocity
	msg.angle = angle
	pub_2.publish(msg)


#-----------------------------------------------------------------------
#FUNCAO: listener()
#	inicializa o no
#	assina os topicos de leitura desejados
#-----------------------------------------------------------------------
def listener():
	rospy.init_node('PriusHybridPlugin', anonymous=True)			#inicializa o no
	rospy.Subscriber('X_Y', coords, vehicle_control)				#subscreve no topico e chama a funcao vehicle_control
	rospy.Subscriber('drive_parameters', drive_param, vel_and_angle)#subscreve no topico e chama a funcao vel_and_angle
	rospy.Subscriber('/car1/carINFO',CAM_simplified,car_info)		#subscreve no topico e chama a funcao car_info
	rospy.spin()													#mantem o listener aberto

if __name__ == '__main__':
	listener()


