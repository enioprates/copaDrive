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
#v3
#	Tratando o erro de angulacao
#v4
#	Alterando equacao de controle
#v5
#	(OLD) Salvando dados no csv (Influi no desempenho! Melhor deixar em NO separado)
#	Enviando dados para topico error_control
#-----------------------------------------------------------------------

#Import commands
import rospy
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
from image_processing.msg import AckermannDriveStamped
from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math
import numpy as np
import time as tp
import csv
#------------------------------------------------------------------------

#Publication Topic
pub = rospy.Publisher('car1/prius', Control, queue_size=10)	#topico de controle do carro
pub_2 = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
pub_3 = rospy.Publisher('error_control', error_control, queue_size=1) 
#------------------------------------------------------------------------

#global variables  
speed_real = 0.0											#velocidade global do veiculo
longitude_old = 0.0
latitude_old = 0.0
longitude_real = 0.0
latitude_real = 0.0
lap = 0
count_lap = 0
status = 0


#PID de velocidade
kp_vel=2				#Kp
ki_vel=0				#Ki
kd_vel=0				#Kd

#PID das rodas
""" 54 km
kp_steer=6.0 #2.5					#Original = 5	#Kp 40 km - 3.2
ki_steer=0.007					#0.003				#Ki 40 km - 0.005
kd_steer=8.5 #3.0					#Original = 5	#Kd 40 km - 5.2
steer_integral = 0.0			
"""
kp_steer= 3.2 #2.5					#Original = 5	#Kp 40 km - 3.2
ki_steer= 0.005						#0.003			#Ki 40 km - 0.005
kd_steer= 5.2 #3.0					#Original = 5	#Kd 40 km - 5.2
steer_integral = 0.0			

#used variables
tp_corr = 1 #segundos  - tempo correcao
time1 = 0.0

#parametros Iniciais
servo_offset = 18.0
velocity = 0.0
prev_error_steer=0.0
prev_error_vel=0.0
prev_theta = 0.0
error_steer=0.0
error_vel=0.0
angle=0.0
velocity=0.0
count = 0					#variavel de teste de numero de iteracoes


MAX_SPEED = 30.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 30.0			#maxima angulacao da roda em graus
MIN_STEER = -30.0			#minima angulacao da roda em graus

ANG_TO_RAD	= np.pi/180		#converte angulo para radiano
RAD_TO_ANGLE = 180/np.pi	#converte radiano para angulo

KMH_TO_MS = 1/3.6			#converte de Km/H para m/s
MS_TO_KMH = 3.6				#converte de m/s para Km/h

EDGE_LOW_SPEED	= 0.05		#ajuste para direcionar velocidade para zero (Erro do Gazebo) - m/s
MIN_VEL = 0.0				#
MAX_ACEL = 1.0				#
MAX_BRAKE = 1.0				#
MIN_ACEL = 0.0				#
MIN_BRAKE = 0.0				#


VEL_CRUZEIRO = 40.0			#KM/H
VEL_FRENAGEM = 20.0			#KM/H
NORMAL = 1
CORRECAO = 0

#-----------------------------------------------------------------------

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(data)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(s_sp, s_real):
	
	erro_vel = s_sp/3.6  - s_real						#calcula o erro de velocidade -> "/3.6" converte para m/s
	erro_vel_norm = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	
														#normaliza o erro (0 - 1)
	accel_control = min(erro_vel_norm,1.0)				#acao de controle
	accel_control = max(accel_control,-1.0)				#acao de controle

	if(abs(accel_control) < EDGE_LOW_SPEED and s_sp == MIN_VEL):		#necessario para zerar a velocidade
		aceleracao = MIN_ACEL
		freio = MAX_BRAKE
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
	msg.steer = data.angle * ANG_TO_RAD						#angulacao desejada para as rodas (0-30) GRAUS OU RADIANOS?
	
	#print "----START-vel_and_angle----------"
	#print "vel=", data.velocity
	#print "speed_real=", speed_real
	
	msg.throttle, msg.brake = calc_throttle_brake (data.velocity, speed_real)	#funcao que calcul aceleracao e frenagem
	
	#print "Throttle=",msg.throttle
	#print "brake=",msg.brake 
	#print "angle=",data.angle
	#print "----END-vel_and_angle----------"
	pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'

#-----------------------------------------------------------------------
#FUNCAO: grava_dados()
#	Grava os dados desejados em um arquivo CSV
#-----------------------------------------------------------------------
def grava_dados():
	global speed_real										#chamada para a variavel global
	global longitude_real
	global latitude_real
	global lap
	global count_lap
	global longitude_old
	global latitude_old
	global count
	global status
	global error_steer
	var_print = 0											#permite imprimir na funcao
	
	if (abs(longitude_real-longitude_old)>0.5 or abs(latitude_real-latitude_old)>0.5):	#atualiza as leituras de posicao se maior que um limiar 
		
	 	longitude_old = longitude_real					#atualiza as ultimas leituras validas de longitude
	 	latitude_old = latitude_real					#atualiza as ultimas leituras validas de latitude
		
	 	if var_print:									#ultimas leituras
	 		print "latitude_real: ", latitude_real
	 		print "longitude_real: ", longitude_real
        
	 	if (abs(longitude_real-10)<1 and abs(latitude_real-180)<1):		#conta o numero de voltas
	 		if  count_lap == 0:											#so adiciona voltas se count_lap for igual a zero
	 			lap = lap + 1											#incrementa o numero de voltas
	 			count_lap = count_lap + 1								#incrementa o indicador de posicao
				
	 			if var_print:
	 				print 'Voltas: ', lap
	 				print 'Count_lap: ', count_lap
		else:
			count_lap = 0												#indica que o veiculo ja passou do ponto inicial
        
    #     if count == 0 :
    #         with open('/home/enio/Desktop/ROSintegrationPlatooning/carving/CISTER_image_processing/src/image_processing/log_position.csv', mode='w') as log_position_file:
    #             log_writer = csv.writer(log_position_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #             log_writer.writerow(['count','latitude', 'longitude', 'speed_real','speed km','status','erro_steer','lap'])
    #             log_writer.writerow([count,latitude_real, longitude_real,speed_real, speed_real * MS_TO_KMH,status,error_steer,lap])
    #         count = count + 1
    #     else :
    #         with open('/home/enio/Desktop/ROSintegrationPlatooning/carving/CISTER_image_processing/src/image_processing/log_position.csv', mode='a') as log_position_file:
    #             log_writer = csv.writer(log_position_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #             log_writer.writerow([count,latitude_real, longitude_real, speed_real,speed_real * MS_TO_KMH,status,error_steer,lap])
    #         count = count + 1
	

#-----------------------------------------------------------------------
#FUNCAO: car_info(data)
#	Funcao chamada pelo topico de dados atuais do veiculo ('/car1/carINFO')
#-----------------------------------------------------------------------
def car_info(data):
	global speed_real										#chamada para a variavel global
	global longitude_real
	global latitude_real
	global longitude_real
	global latitude_real
	speed_real = data.speed_speedValue						#salva o valor da velocidade no instante
	latitude_real = data.latitude
	longitude_real = data.longitude
	heading_real = data.heading_headingValue
	drive_dir_real = data.driveDirection
	steer_real = data.steeringWheelAngle_steeringWheelAngleValue

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
	global prev_theta
	global cnt
	global count											#numero de iteracoes
	global time1											#tempo entre execucoes
	global steer_integral									#acumulador do erro
	var_print = 1
	
	if var_print:
		print "-----------Vehicle CONTROL ----------"	
		print "count", count
	
	time_now = tp.time()									#armazena o tempo atual
	if (count == 0):
		dt = 0.0												#inicializa a variavel
	else:	
		dt = time_now - time1								#tempo entre iteracoes
	time1 = time_now										#armazena o tempo atual para a proxima iteracao

	if var_print:
		print "dt: ", dt

	x = data.X												#posicao X da linha detectada
	y = data.Y												#posicao Y da linha detectada

	#print "x: ", x											#posicao X da linha detectada
	#print "y: ", y											#posicao Y da linha detectada
	

	error_steer = (x-400)/400								#posicao X da linha em relacao ao centro da imagem 800 pixels
	#print "Error: ",error_steer
	
	#sistema de controle de velocidade
	#print "Count: ",count
	if (count < 1):
		velocity = VEL_CRUZEIRO
		status = NORMAL
	elif (abs(error_steer) > 0.075) and y > 450:  
		velocity = VEL_FRENAGEM
		status = CORRECAO
		if var_print:
			print "CORRECAO"
	else: 
		velocity = VEL_CRUZEIRO
		status = NORMAL
		if var_print:
			print "NORMAL"
	count = count+1
	if var_print:
		print "vel:",velocity

	#--------------------------
	dist_corr = velocity * KMH_TO_MS						#distancia a ser percorrida em metros em 1 s
	#print "dist_corr: ", dist_corr

	dist_ang = error_steer/dist_corr						#angulo do erro
	#print "cat op sobre hip: ", error_steer/dist_corr

	if(abs(dist_ang)<=1):
		theta_error = -np.arcsin(error_steer/dist_corr) * RAD_TO_ANGLE	#correcao normal
	else:
		theta_error = -np.arcsin((error_steer/abs(error_steer))/(dist_corr/abs(dist_corr))) * RAD_TO_ANGLE	#se a distancia inicial for pequena para o erro

	#print "theta_error: ", theta_error
	
	steer_integral = steer_integral + theta_error * dt
	#print "steer_integral: ", steer_integral

	steer_deriv = prev_theta - theta_error
	#print "previous error:", (prev_theta - theta_error)

	#sistema de controle de angulacao
	control_error_steer_2 = kp_steer*theta_error + ki_steer * steer_integral + kd_steer * steer_deriv 
	control_error_steer_1 = kp_steer*theta_error + ki_steer * steer_integral
	control_error_steer_0 = kp_steer*theta_error

	control_error_steer = control_error_steer_2
	
	prev_theta = theta_error
	
	#print "Control Error Steer:", control_error_steer
	#print "Control Error Steer_0:", control_error_steer_0
	#print "Control Error Steer_1:", control_error_steer_1
	#print "Control Error Steer_2:", control_error_steer_2
	angle = control_error_steer#*ANG_TO_RAD
	#print "angle:", angle
	#print "angle:", angle * ANG_TO_RAD
	#prev_error_vel = error_vel

	#angle = 30/float(-320) * x + 30
	#angle = -angle
	if angle > 30:
		angle = 30
	if angle < -30:
		angle = -30
	#print "angle:",angle
	#--------------------------
	if var_print:
		print "-----------END Vehicle CONTROL ----------"

	#prepara msg para publicacao
	msg = drive_param()
	msg.velocity = velocity
	msg.angle = angle
	pub_2.publish(msg)

	msg_e = error_control()
	msg_e.error_steer = error_steer
	msg_e.control_error_steer = control_error_steer
	msg_e.steer_integral = steer_integral
	msg_e.steer_deriv = steer_deriv
	msg_e.status = status
	pub_3.publish(msg_e)

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


