#!/usr/bin/env python
#-----------------------------------------------------------------------
#Listener.py
#Codigo responsavel por guardar os dados de posicao do veiculo
#v00
#	Codigo inicial, gravando apenas dados de posicao, velocidade e angulacao do veiculo
#v01	
#   Gravando dados dos sonares
#-----------------------------------------------------------------------

import rospy
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
speed_real = 0.0											#velocidade global do veiculo
longitude_old = 0.0
latitude_old = 0.0
longitude_real = 0.0
latitude_real = 0.0
heading_real = 0.0
count = 0
lap = 0
count_lap = 0
status  = 0

range_LF = 0.0												#variaveis de leitura do sonar
range_LM = 0.0												#variaveis de leitura do sonar
range_RM = 0.0												#variaveis de leitura do sonar
range_RF = 0.0												#variaveis de leitura do sonar
range_side_L = 0.0
range_side_R = 0.0
max_range_LF = 0.0
max_range_LM = 0.0
max_range_RM = 0.0
max_range_RF = 0.0
max_range_side_L = 0.0
max_range_side_R = 0.0

#-----------------------------------------------------------------------
#FUNCAO: grava_dados()
#	Grava os dados desejados em um arquivo CSV
#-----------------------------------------------------------------------
def grava_dados():
    global speed_real  # chamada para a variavel global
    global longitude_real
    global latitude_real
    global lap
    global count_lap
    global longitude_old
    global latitude_old
    global heading_real
    global count

    global range_LF											#leitura do sonar
    global range_LM											#leitura do sonar
    global range_RM											#leitura do sonar
    global range_RF											#leitura do sonar
    global range_side_L
    global range_side_R
    global max_range_LF
    global max_range_LM
    global max_range_RM
    global max_range_RF
    global max_range_side_L
    global max_range_side_R
    
    #print 'GRAVA_DADOS'

    var_print = 0  # permite imprimir na funcao
    

    # atualiza as leituras de posicao se maior que um limiar
    if (abs(longitude_real-longitude_old) > 0.5 or abs(latitude_real-latitude_old) > 0.5 or range_LF < max_range_LF or range_LM < max_range_LM  or range_RM < max_range_RM or range_RF < max_range_RF or range_side_L < max_range_side_L or range_side_R < max_range_side_R):

	    longitude_old = longitude_real  # atualiza as ultimas leituras validas de longitude
	    latitude_old = latitude_real  # atualiza as ultimas leituras validas de latitude

	    if var_print:  # ultimas leituras
		    print "latitude_real: ", latitude_real
		    print "longitude_real: ", longitude_real
		    print "lap:" , lap
		    a = abs(longitude_real-10.65)
		    b = abs(latitude_real-70)
		    print "a:", a
		    print "b:", b

	    if (abs(longitude_real-70) < 1 and abs(latitude_real-10.65) < 2):  # conta o numero de voltas
	 	    if count_lap == 0:  # so adiciona voltas se count_lap for igual a zero
			lap = lap + 1  # incrementa o numero de voltas
			print "lap:" , lap
			count_lap = count_lap + 1  # incrementa o indicador de posicao
			if var_print:
				print 'Voltas: ', lap
				print 'Count_lap: ', count_lap
	    else:
		    count_lap = 0   # indica que o veiculo ja passou do ponto inicial
            #print 'Count_lap: ', count_lap
            if count == 0:
            	with open('/home/enio/Desktop/ROSintegrationPlatooning/carving/CISTER_image_processing/src/image_processing/log_position.csv', mode='w') as log_position_file:
               		log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                	log_writer.writerow(['count','latitude', 'longitude', 'speed_real','speed km','heading','lap','SLF','SLM','SRM','SRF','SSL','SSR'])
			count = count + 1
                	log_writer.writerow([count,latitude_real, longitude_real,speed_real, speed_real * 3.6, heading_real, lap, range_LF, range_LM, range_RM, range_RF, range_side_L, range_side_R])
                count = count + 1
            else :
            	with open('/home/enio/Desktop/ROSintegrationPlatooning/carving/CISTER_image_processing/src/image_processing/log_position.csv', mode='a') as log_position_file:
                	log_writer = csv.writer(log_position_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                	log_writer.writerow([count,latitude_real, longitude_real, speed_real,speed_real * 3.6,heading_real,lap,range_LF,range_LM,range_RM,range_RF,range_side_L,range_side_R])
            count = count + 1


def car_info(data):
    global speed_real										#chamada para a variavel global
    global longitude_old
    global latitude_old
    global longitude_real
    global latitude_real
    global count
    global lap
    global heading_real
    global count_lap
    #print 'CAR_INFO'
    speed_real = data.speed_speedValue						# salva o valor da velocidade no instante
    latitude_real = data.latitude
    longitude_real = data.longitude
    heading_real = data.heading_headingValue
    drive_dir_real = data.driveDirection
    steer_real = data.steeringWheelAngle_steeringWheelAngleValue
    grava_dados()

#-----------------------------------------------------------------------
#FUNCAO: sonar_XX(data)
#	Le os dados do sonar do veiculo
#-----------------------------------------------------------------------
def sonar_front_LF(data):
	global range_LF
	global max_range_LF
	range_LF = data.range
	max_range_LF = data.max_range
def sonar_front_LM(data):
	global range_LM
	global max_range_LM
	range_LM = data.range
	max_range_LM = data.max_range
def sonar_front_RM(data):
	global range_RM
	global max_range_RM
	range_RM = data.range
	max_range_RM = data.max_range
def sonar_front_RF(data):
	global range_RF
	global max_range_RF
	range_RF = data.range
	max_range_RF = data.max_range
def sonar_side_R(data):
	global range_side_R
	global max_range_side_R
	range_side_R = data.range
	max_range_side_R = data.max_range
def sonar_side_L(data):
	global range_side_L
	global max_range_side_L
	range_side_L = data.range
	max_range_side_L = data.max_range    

def listener():

    	# In ROS, nodes are uniquely named. If two nodes with the same
    	# name are launched, the previous one is kicked off. The
    	# anonymous=True flag means that rospy will choose a unique
    	# name for our 'listener' node so that multiple listeners can
    	# run simultaneously.
	rospy.init_node('listener', anonymous=True)
    	#print 'LISTENER!'
	rospy.Subscriber('/car1/carINFO',CAM_simplified,car_info)
	rospy.Subscriber('/car1/front_sonar_left_far_range',Sonar,sonar_front_LF)           #subscreve ao topicoi e chama a funcao sonar_LF
	rospy.Subscriber('/car1/front_sonar_left_middle_range',Sonar,sonar_front_LM)        #subscreve ao topicoi e chama a funcao sonar_LM
	rospy.Subscriber('/car1/front_sonar_right_middle_range',Sonar,sonar_front_RM)       #subscreve ao topicoi e chama a funcao sonar_RM
	rospy.Subscriber('/car1/front_sonar_right_far_range',Sonar,sonar_front_RF)          #subscreve ao topicoi e chama a funcao sonar_RF
	rospy.Subscriber('/car1/side_sonar_left_front_range',Sonar,sonar_side_L)            #subscreve ao topicoi e chama a funcao sonar_RF
	rospy.Subscriber('/car1/side_sonar_right_front_range',Sonar,sonar_side_R)           #subscreve ao topicoi e chama a funcao sonar_RF
    	#rospy.Subscriber('error_control',error_control,dados_controle)

    	# spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
    listener()
