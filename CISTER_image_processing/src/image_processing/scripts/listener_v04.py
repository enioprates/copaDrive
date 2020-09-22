#!/usr/bin/env python
#-----------------------------------------------------------------------
#Listener.py
#Codigo responsavel por guardar os dados de posicao do veiculo
#v00
#	Codigo inicial, gravando apenas dados de posicao, velocidade e angulacao do veiculo
#v01	
#   Gravando dados dos sonares
#v02
#	Retirando gravacao dos sonares
#	Gravando posicoes de carN (entrada de dados)
#v03
#v04
#	Change the event that triggers the record
#v05
#	adding new cars
#-----------------------------------------------------------------------

import rospy
import sys                                                  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
from image_processing.msg import AckermannDriveStamped
from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
from ros_its_msgs.msg import Sonar						#msg do tipo Sonar
import math
import numpy as np
import time as tp
import csv

#global variables  
longitude_TV = 0.0                                       #longitude
latitude_TV = 0.0                                        #latitude 
heading_TV = 0.0                                         #heading (angulacao global do veiculo)
speed_TV = 0.0											 #vehicle speed (m/s)
steering_TV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_TV = 0.0                                        #percentual do acelerador (0 - 1)
brake_TV = 0.0                                           #percentual do freio (0 - 1)
old_latitude_TV = 0.0
old_longitude_TV = 0.0

SV_name = 0												#name of the saved SV
longitude_SV = 0.0                                       #longitude
latitude_SV = 0.0                                        #latitude 
heading_TV = 0.0                                         #heading (angulacao global do veiculo)
speed_SV = 0.0											 #vehicle speed (m/s)
steering_SV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0                                        #percentual do acelerador (0 - 1)
brake_SV = 0.0                                           #percentual do freio (0 - 1)
old_latitude_SV = 0.0
old_longitude_SV = 0.0

PID_error = 0.0
THETA_error = 0.0
distance = 0.0

count_TV = 0
lap_TV = 0
count_lap_TV = 0
init_lat_TV = 150
init_long_TV = 45.85

count_SV = 0
lap_SV = 0
count_lap_SV = 0
init_lat_SV = 170
init_long_SV = 10

param = 0											#param lsit

time_base = 0										#start time of the system
elapsed_time_TV = 0
old_time_TV = 0
elapsed_time_SV = 0
old_time_SV = 0

TIME_SPACE = 0.01


#-----------------------------------------------------------------------
#FUNCAO: grava_dados()
#	Grava os dados desejados em um arquivo CSV
#-----------------------------------------------------------------------
def grava_dados_TV():

	global longitude_TV
	global latitude_TV
	global heading_TV
	global speed_TV
	global steering_TV
	global throttle_TV
	global brake_TV
	global old_longitude_TV
	global old_latitude_TV
	global count_TV
	global lap_TV
	global count_lap_TV

	global time_base
	global elapsed_time_TV
	global old_time_TV


	#print 'GRAVA_DADOS'

	var_print = 0  # permite imprimir na funcao

	current_time_TV = tp.time()
	elapsed_time_TV = current_time_TV - old_time_TV

	# atualiza as leituras de posicao se maior que um limiar
	#if (abs(longitude_TV-old_longitude_TV) > 0.5 or abs(latitude_TV-old_latitude_TV) > 0.5):
	if elapsed_time_TV >= TIME_SPACE:
		old_time_TV = current_time_TV			#record the last time event
		old_longitude_TV = longitude_TV  # atualiza as ultimas leituras validas de longitude
		old_latitude_TV = latitude_TV  # atualiza as ultimas leituras validas de latitude

		if var_print:  # ultimas leituras
			print "latitude_TV: ", latitude_TV
			print "longitude_TV: ", longitude_TV
			print "lap:" , lap
			a = abs(longitude_TV-init_long_TV)
			b = abs(latitude_TV-init_lat_TV)
			print "a:", a
			print "b:", b

		if (abs(longitude_TV-init_long_TV) < 1 and abs(latitude_TV-init_lat_TV) < 2):  	# conta o numero de voltas
			if count_lap_TV == 0:  												# so adiciona voltas se count_lap for igual a zero
				lap_TV = lap_TV + 1  														# incrementa o numero de voltas
				print "lap_TV:" , lap_TV
				count_lap_TV = count_lap_TV + 1  											# incrementa o indicador de posicao
		else:
			count_lap_TV = 0 # indica que o veiculo ja passou do ponto inicial
		if var_print:
			print 'Voltas: ', lap_TV
			print 'Count_lap: ', count_lap_TV
	if count_TV == 0:
		with open('/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/log_position_TV.csv', mode='w') as log_position_file:
			log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			#log_writer.writerow(['time','count','latitude_TV', 'longitude_TV', 'speed_TV','speed km','heading_TV','lap'])
			log_writer.writerow([tp.time(), count_TV,latitude_TV, longitude_TV,speed_TV, speed_TV * 3.6, heading_TV, lap_TV])
		count_TV = count_TV + 1
	else :
		with open('/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/log_position_TV.csv', mode='a') as log_position_file:
			log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			log_writer.writerow([tp.time()-time_base, count_TV,latitude_TV, longitude_TV, speed_TV,speed_TV * 3.6,heading_TV,lap_TV])
		count_TV = count_TV + 1

#-----------------------------------------------------------------------
#FUNCAO: grava_dados()
#	Grava os dados desejados em um arquivo CSV
#-----------------------------------------------------------------------
def grava_dados_SV():
	global SV_name
	global longitude_SV
	global latitude_SV
	global heading_SV
	global speed_SV
	global steering_SV
	global throttle_SV
	global brake_SV
	global old_longitude_SV
	global old_latitude_SV
	global count_SV
	global lap_SV
	global count_lap_SV

	global PID_error
	global THETA_error
	global distance

	global time_base
	global elapsed_time_SV
	global old_time_SV
	#print 'GRAVA_DADOS'

	var_print = 0  # permite imprimir na funcao

	current_time_SV = tp.time()
	elapsed_time_SV = current_time_SV - old_time_SV

	# atualiza as leituras de posicao se maior que um limiar
	#if (abs(longitude_SV-old_longitude_SV) > 0.5 or abs(latitude_SV-old_latitude_SV) > 0.5):
	if elapsed_time_SV >= TIME_SPACE:
		old_time_SV = current_time_SV			#record the last time event
		old_longitude_SV = longitude_SV  # atualiza as ultimas leituras validas de longitude
		old_latitude_SV = latitude_SV  # atualiza as ultimas leituras validas de latitude

		if var_print:  # ultimas leituras
			print "latitude_SV: ", latitude_SV
			print "longitude_SV: ", longitude_SV
			print "lap_SV:" , lap_SV
			a = abs(longitude_SV-init_long_SV)
			b = abs(latitude_SV-init_lat_SV)
			print "a:", a
			print "b:", b

		if (abs(longitude_SV-init_long_SV) < 1 and abs(latitude_SV-init_lat_SV) < 2):  	# conta o numero de voltas
			if count_lap_SV == 0:  												# so adiciona voltas se count_lap for igual a zero
				lap_SV = lap_SV + 1  														# incrementa o numero de voltas
				print "lap_SV:" , lap_SV
				count_lap_SV = count_lap_SV + 1  											# incrementa o indicador de posicao
		else:
			count_lap_SV = 0 # indica que o veiculo ja passou do ponto inicial
		if var_print:
			print 'Voltas_SV: ', lap_SV
			print 'Count_lap_SV: ', count_lap_SV
	if count_SV == 0:
		with open('/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/log_position_SV_'+SV_name+'.csv', mode='w') as log_position_file:
			log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			#log_writer.writerow(['time','count_SV','latitude_SV', 'longitude_SV', 'speed_SV','speed km','heading_SV','PID Error','Theta Error','lap_SV','distance'])
			log_writer.writerow([tp.time()-time_base, count_SV,latitude_SV, longitude_SV,speed_SV, speed_SV * 3.6, heading_SV, PID_error, THETA_error,lap_SV,distance])
        	count_SV = count_SV + 1
	else :
		with open('/home/enio/OneDrive/Cister/ROS/Inline/CISTER_image_processing/src/image_processing/log_position_SV_'+SV_name+'.csv', mode='a') as log_position_file:
			log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			log_writer.writerow([tp.time()-time_base, count_SV,latitude_SV, longitude_SV, speed_SV,speed_SV * 3.6, heading_SV, PID_error, THETA_error, lap_SV, distance])
		count_SV = count_SV + 1

def dados_controle(data):
	global PID_error
	global THETA_error
	global distance

	PID_error = data.pid_error_value
	THETA_error = data.theta_error_value
	distance = data.dist_tv


def car_info_TV(data):
	global longitude_TV
	global latitude_TV
	global heading_TV
	global speed_TV
	global steering_TV
	global throttle_TV
	global brake_TV

	#print 'CAR_INFO'
	speed_TV = data.speed_speedValue						# salva o valor da velocidade no instante
	latitude_TV = data.latitude
	longitude_TV = data.longitude
	heading_TV = data.heading_headingValue
	drive_dir_TV = data.driveDirection
	steering_TV = data.steeringWheelAngle_steeringWheelAngleValue
	throttle_TV = data.gasPedalPercent_Value
	brake_TV = data.brakePedalPercent_Value
	grava_dados_TV()


def car_info_SV(data):
	global longitude_SV
	global latitude_SV
	global heading_SV
	global speed_SV
	global steering_SV
	global throttle_SV
	global brake_SV

	#print 'CAR_INFO'
	speed_SV = data.speed_speedValue						# salva o valor da velocidade no instante
	latitude_SV = data.latitude
	longitude_SV = data.longitude
	heading_SV = data.heading_headingValue
	drive_dir_SV = data.driveDirection
	steering_SV = data.steeringWheelAngle_steeringWheelAngleValue
	throttle_SV = data.gasPedalPercent_Value
	brake_SV = data.brakePedalPercent_Value
	grava_dados_SV()


def listener():
	global param
	global SV_name

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)
	print 'LISTENER!'

	if 'car1' in param:
		print "car1"
		rospy.Subscriber('/car1/carINFO',CAM_simplified,car_info_TV)
	if 'car2' in param:
		print "car2"
		SV_name = "car2"
		rospy.Subscriber('/car2/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car2/error_control',error_control,dados_controle)
	elif 'car3' in param:
		print "car3"
		SV_name = "car3"
		rospy.Subscriber('/car3/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car3/error_control',error_control,dados_controle)
	elif 'car4' in param:
		print "car4"
		SV_name = "car4"
		rospy.Subscriber('/car4/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car4/error_control',error_control,dados_controle)
	elif 'car5' in param:
		print "car5"
		SV_name = "car5"
		rospy.Subscriber('/car5/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car5/error_control',error_control,dados_controle)
	elif 'car6' in param:
		print "car6"
		SV_name = "car6"
		rospy.Subscriber('/car6/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car6/error_control',error_control,dados_controle)
	elif 'car7' in param:
		print "car7"
		SV_name = "car7"
		rospy.Subscriber('/car7/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car7/error_control',error_control,dados_controle)
	elif 'car8' in param:
		print "car8"
		SV_name = "car8"
		rospy.Subscriber('/car8/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car8/error_control',error_control,dados_controle)
	elif 'car9' in param:
		print "car9"
		SV_name = "car9"
		rospy.Subscriber('/car9/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car9/error_control',error_control,dados_controle)
	elif 'car10' in param:
		print "car10"
		SV_name = "car10"
		rospy.Subscriber('/car10/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car10/error_control',error_control,dados_controle)
	elif 'car11' in param:
		print "car11"
		SV_name = "car11"
		rospy.Subscriber('/car11/carINFO',CAM_simplified,car_info_SV)
		rospy.Subscriber('/car11/error_control',error_control,dados_controle)
	# rospy.Subscriber('/car1/front_sonar_left_far_range',Sonar,sonar_front_LF)           #subscreve ao topicoi e chama a funcao sonar_LF
	# rospy.Subscriber('/car1/front_sonar_left_middle_range',Sonar,sonar_front_LM)        #subscreve ao topicoi e chama a funcao sonar_LM
	# rospy.Subscriber('/car1/front_sonar_right_middle_range',Sonar,sonar_front_RM)       #subscreve ao topicoi e chama a funcao sonar_RM
	# rospy.Subscriber('/car1/front_sonar_right_far_range',Sonar,sonar_front_RF)          #subscreve ao topicoi e chama a funcao sonar_RF
	# rospy.Subscriber('/car1/side_sonar_left_front_range',Sonar,sonar_side_L)            #subscreve ao topicoi e chama a funcao sonar_RF
	# rospy.Subscriber('/car1/side_sonar_right_front_range',Sonar,sonar_side_R)           #subscreve ao topicoi e chama a funcao sonar_RF
	#rospy.Subscriber('error_control',error_control,dados_controle)

    	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	
	param = sys.argv[1:]
	time_base = tp.time()
	listener()
