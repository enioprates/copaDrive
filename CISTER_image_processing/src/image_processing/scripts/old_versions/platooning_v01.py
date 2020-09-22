#!/usr/bin/env python
#-----------------------------------------------------------------------
#platooning.py TV (target Vehicle) SV
#Codigo responsavel pelo controle do veiculo seguidor (SV)
#v0
#   Le os dados do TV (local) (param0) - TV (target Vehicle)
#   Le os dados atuais do SV (param1) - SV (subject vehicle)
#   Atua sobre o SV
#-----------------------------------------------------------------------
import rospy
import sys                                                  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
#from image_processing.msg import AckermannDriveStamped
#from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math
import numpy as np
import time as tp
import csv

#------------------------------------------------------------------------

#Publication Topic

#------------------------------------------------------------------------



#global variables  
#dados do TV a ser seguido --------------------------------------------
longitude_TV = 0.0                                       #longitude
latitude_TV = 0.0                                        #latitude 
heading_TV = 0.0                                         #heading (angulacao global do veiculo)
speed_TV = 0.0											#velocidade do veiculo (m/s)
steering_TV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_TV = 0.0                                        #percentual do acelerador (0 - 1)
brake_TV = 0.0                                           #percentual do freio (0 - 1)

#dados do proprio veiculo --------------------------------------------
longitude_SV = 0.0                                       #longitude
latitude_SV = 0.0                                        #latitude 
heading_SV = 0.0                                         #heading (angulacao global do veiculo)
speed_SV = 0.0											#velocidade do veiculo
steering_SV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0                                        #percentual do acelerador (0 - 1)
brake_SV = 0.0                                           #percentual do freio (0 - 1)

longitude_old = 0.0
latitude_old = 0.0
heading_SV = 0.0                                         #heading old (angulacao global do veiculo)
speed_SV = 0.0											#velocidade old do veiculo
steering_SV = 0.0                                        #steering old (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0                                        #percentual old do acelerador (0 - 1)
brake_SV = 0.0                                           #percentual old do freio (0 - 1)

#variaveis de controle --------------------------------------------
kp_vel = 1
ki_vel = 0
kd_vel = 0

vel_integral = 0.0
vel_deriv = 0.0
erro_vel_old = 0.0
erro_vel = 0.0

#variaveis gerais -------------------------------------------------
TV_topic = 0                                           #topico do TV
SV_topic = 0                                         #topico do SV
publication_topic = 0                                      #topico a publicar dados do veiculo

time1=0.0
count = 0
lap = 0
count_lap = 0
error_steer = 0.0
control_error_steer  = 0.0
steer_integral  = 0.0
steer_deriv  = 0.0
status  = 0

#defines------------------------------------------------------------
MAX_SPEED = 30.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 30.0				#maxima angulacao da roda em graus
MIN_STEER = -30.0				#minima angulacao da roda em graus

ANG_TO_RAD	= np.pi/180			#converte graus para radiano
RAD_TO_ANGLE = 180/np.pi		#converte radiano para graus

KMH_TO_MS = 1/3.6				#converte de Km/H para m/s
MS_TO_KMH = 3.6					#converte de m/s para Km/h
EDGE_LOW_SPEED	= 0.05		#ajuste para direcionar velocidade para zero (Erro do Gazebo) - m/s
MIN_VEL = 0.0				#
MAX_ACEL = 1.0				#
MAX_BRAKE = 1.0				#
MIN_ACEL = 0.0				#
MIN_BRAKE = 0.0				#

MINIMUM_DISTANCE = 8        #distancia de seguranca - m
SAFETY_TIME = 2             #tempo de seguranca - s
MAX_ERRO_INTEGRADOR_VEL = 50
ZERO = 0

#-----------------------------------------------------------------------
#FUNCAO: controla_velocidade()
#	
#-----------------------------------------------------------------------
def controla_velocidade (v_TV, v_SV):
    #variaveis globais-----------------
    global vel_integral
    global vel_deriv
    global erro_vel_old
    global time1
    global erro_vel
    #define dt------------------------------------------------------------
    time_now = tp.time()									#armazena o tempo atual
    dt = time_now - time1								    #tempo entre iteracoes
    time1 = time_now										#armazena o tempo atual para a proxima iteracao

    erro_vel = v_TV - v_SV                 #erro de velocidade
    print "vel_TV: ", v_TV
    print "vel_SV: ", v_SV
    if (erro_vel >= EDGE_LOW_SPEED):                #se erro de velocidade significativo
        print "VEL_DIFERENTE"
        #determina valor do integrador
        if (abs(vel_integral) >= MAX_ERRO_INTEGRADOR_VEL or dt>10):
            vel_integral = ZERO
        else:
            vel_integral = vel_integral + erro_vel * dt
        
        #valor do derivativo
        vel_deriv = erro_vel_old - erro_vel
        erro_vel_old = erro_vel
        control_vel = ( kp_vel*erro_vel + ki_vel*vel_integral +kd_vel*vel_deriv)
        print "Control_vel: ", control_vel
    else:
        print "VEL_IGUAL"
        erro_vel = 0.0
        control_vel = 0.0
    return control_vel

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(data)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(acao_vel, v_TV):
	
	erro_vel_norm = (acao_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	#normaliza o erro (0 - 1)
	accel_control = min(erro_vel_norm,1.0)				                #acao de controle
	accel_control = max(accel_control,-1.0)				                #acao de controle

	if(abs(accel_control) < EDGE_LOW_SPEED and v_TV < EDGE_LOW_SPEED):		#necessario para zerar a velocidade
		aceleracao = MIN_ACEL
		freio = MAX_BRAKE
	else:
		aceleracao = max(0.0,accel_control)				#acao de aceleracao
		freio = max(0.0,-accel_control)					#acao de frenagem
	return aceleracao, freio

#-----------------------------------------------------------------------
#FUNCAO: controla_veiculo()
#	
#-----------------------------------------------------------------------
def controla_veiculo():
    #dados do TV
    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    #dados do veiculo
    global longitude_SV
    global latitude_SV
    global heading_SV
    global speed_SV							                #chamada para a variavel global
    global steering_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1)

    global publication_topic

    global erro_vel

    pub = rospy.Publisher(publication_topic, Control, queue_size=10)				#topico de controle do carro
    msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
    msg.header.stamp = rospy.Time.now();					#tempo SVizado da leitura
    msg.header.frame_id = "base_link";						
    msg.steer = 0 * ANG_TO_RAD						#angulacao desejada para as rodas (0-30) GRAUS

    #calcula distancia
    distance = ((longitude_TV - longitude_SV)**2 + (latitude_TV - latitude_SV)**2)**0.5
    #print "Distance", distance
    maximum_distance = MINIMUM_DISTANCE + speed_TV * 2
    #ajusta velocidade e aceleracao
    if (speed_TV >= EDGE_LOW_SPEED and distance >= MINIMUM_DISTANCE):
        print "TV moving"
        acao_velocidade = controla_velocidade (speed_TV, speed_SV)         #m/s
        msg.throttle, msg.brake =calc_throttle_brake(acao_velocidade, speed_TV)
    else:
        print "TV stopped"
        msg.throttle = 0.0
        msg.brake = 1.0
    print "throttle", msg.throttle
    print "brake", msg.brake
    #ajusta heading
    #SViza topico carro
    pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'





def TV_info(data):

    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    global longitude_SV
    global latitude_SV
    global heading_SV
    global speed_SV							                #chamada para a variavel global
    global steering_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1)

    global count
    global lap
    global count_lap
    #print 'TV_INFO'

    latitude_TV = data.latitude
    longitude_TV = data.longitude
    heading_TV = data.heading_headingValue
    speed_TV = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_TV = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_TV = data.gasPedalPercent_Value
    brake_TV = data.brakePedalPercent_Value

  

def auto_info(data):

    global longitude_SV
    global latitpublication_topicude_SV
    global headipublication_topicng_SV
    global speedpublication_topic_SV							                #chamada para a variavel global
    global steerpublication_topicing_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throtpublication_topictle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1)

    global longitude_SV
    global latitude_SV
    global heading_SV
    global speed_SV							                #chamada para a variavel global
    global steering_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1

    #print 'SV_INFO'

    latitude_SV = data.latitude
    longitude_SV = data.longitude
    heading_SV = data.heading_headingValue
    speed_SV = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_SV = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_SV = data.gasPedalPercent_Value
    brake_SV = data.brakePedalPercent_Value
    controla_veiculo()

def listener():
    global TV_topic
    global SV_topic
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print TV_topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(TV_topic,CAM_simplified,TV_info)
    rospy.Subscriber(SV_topic,CAM_simplified,auto_info)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #global TV_topic
    #global SV_topic
    #global publication_topic
    param = sys.argv[1:]                                        #le os dados passados como argumentos (TV SV)
    #TODO - tratamento de erros para parametros de entrada
    #TODO - Tornar generico parametros de leitura
    TV_topic = "/" + param[0] +"/carINFO"                   #cria a string do topico do TV
    SV_topic = "/" + param[1] +"/carINFO"                 #cria a string do topico do SV
    publication_topic = "/" + param[1] +"/prius"                 #cria a string do topico do SV
    listener()
