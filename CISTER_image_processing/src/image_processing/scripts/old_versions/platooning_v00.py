#!/usr/bin/env python
#-----------------------------------------------------------------------
#platooning.py leader follower
#Codigo responsavel pelo controle do veiculo seguidor
#v0
#   Le os dados do veiculo lider (local) (param0)
#   Le os dados atuais do veiculo (param1)
#   Atua sobre o veiculo
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
#dados do lider a ser seguido --------------------------------------------
longitude_leader = 0.0                                       #longitude
latitude_leader = 0.0                                        #latitude 
heading_leader = 0.0                                         #heading (angulacao global do veiculo)
speed_leader = 0.0											#velocidade do veiculo (m/s)
steering_leader = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_leader = 0.0                                        #percentual do acelerador (0 - 1)
brake_leader = 0.0                                           #percentual do freio (0 - 1)

#dados do proprio veiculo --------------------------------------------
longitude_atual = 0.0                                       #longitude
latitude_atual = 0.0                                        #latitude 
heading_atual = 0.0                                         #heading (angulacao global do veiculo)
speed_atual = 0.0											#velocidade do veiculo
steering_atual = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_atual = 0.0                                        #percentual do acelerador (0 - 1)
brake_atual = 0.0                                           #percentual do freio (0 - 1)

longitude_old = 0.0
latitude_old = 0.0
heading_atual = 0.0                                         #heading old (angulacao global do veiculo)
speed_atual = 0.0											#velocidade old do veiculo
steering_atual = 0.0                                        #steering old (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_atual = 0.0                                        #percentual old do acelerador (0 - 1)
brake_atual = 0.0                                           #percentual old do freio (0 - 1)

#variaveis de controle --------------------------------------------
kp_vel = 1
ki_vel = 0
kd_vel = 0

vel_integral = 0.0
vel_deriv = 0.0
erro_vel_old = 0.0
erro_vel = 0.0

#variaveis gerais -------------------------------------------------
leader_topic = 0                                           #topico do lider
follower_topic = 0                                         #topico do follower
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
def controla_velocidade (v_lider, v_follower):
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

    erro_vel = v_lider - v_follower                 #erro de velocidade
    print "vel_lider: ", v_lider
    print "vel_follower: ", v_follower
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
def calc_throttle_brake(acao_vel, v_lider):
	
	erro_vel_norm = (acao_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	#normaliza o erro (0 - 1)
	accel_control = min(erro_vel_norm,1.0)				                #acao de controle
	accel_control = max(accel_control,-1.0)				                #acao de controle

	if(abs(accel_control) < EDGE_LOW_SPEED and v_lider < EDGE_LOW_SPEED):		#necessario para zerar a velocidade
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
    #dados do lider
    global longitude_leader
    global latitude_leader
    global heading_leader
    global speed_leader							                #chamada para a variavel global
    global steering_leader                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_leader                                       #percentual do acelerador (0 - 1)
    global brake_leader                                          #percentual do freio (0 - 1)

    #dados do veiculo
    global longitude_atual
    global latitude_atual
    global heading_atual
    global speed_atual							                #chamada para a variavel global
    global steering_atual                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_atual                                       #percentual do acelerador (0 - 1)
    global brake_atual                                          #percentual do freio (0 - 1)

    global publication_topic

    global erro_vel

    pub = rospy.Publisher(publication_topic, Control, queue_size=10)				#topico de controle do carro
    msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
    msg.header.stamp = rospy.Time.now();					#tempo atualizado da leitura
    msg.header.frame_id = "base_link";						
    msg.steer = 0 * ANG_TO_RAD						#angulacao desejada para as rodas (0-30) GRAUS

    #calcula distancia
    distance = ((longitude_leader - longitude_atual)**2 + (latitude_leader - latitude_atual)**2)**0.5
    #print "Distance", distance
    maximum_distance = MINIMUM_DISTANCE + speed_leader * 2
    #ajusta velocidade e aceleracao
    if (speed_leader >= EDGE_LOW_SPEED and distance >= MINIMUM_DISTANCE):
        print "Leader moving"
        acao_velocidade = controla_velocidade (speed_leader, speed_atual)         #m/s
        msg.throttle, msg.brake =calc_throttle_brake(acao_velocidade, speed_leader)
    else:
        print "Leader stopped"
        msg.throttle = 0.0
        msg.brake = 1.0
    print "throttle", msg.throttle
    print "brake", msg.brake
    #ajusta heading
    #atualiza topico carro
    pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'





def leader_info(data):

    global longitude_leader
    global latitude_leader
    global heading_leader
    global speed_leader							                #chamada para a variavel global
    global steering_leader                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_leader                                       #percentual do acelerador (0 - 1)
    global brake_leader                                          #percentual do freio (0 - 1)

    global longitude_atual
    global latitude_atual
    global heading_atual
    global speed_atual							                #chamada para a variavel global
    global steering_atual                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_atual                                       #percentual do acelerador (0 - 1)
    global brake_atual                                          #percentual do freio (0 - 1)

    global count
    global lap
    global count_lap
    #print 'leader_INFO'

    latitude_leader = data.latitude
    longitude_leader = data.longitude
    heading_leader = data.heading_headingValue
    speed_leader = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_leader = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_leader = data.gasPedalPercent_Value
    brake_leader = data.brakePedalPercent_Value

  

def auto_info(data):

    global longitude_atual
    global latitpublication_topicude_atual
    global headipublication_topicng_atual
    global speedpublication_topic_atual							                #chamada para a variavel global
    global steerpublication_topicing_atual                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throtpublication_topictle_atual                                       #percentual do acelerador (0 - 1)
    global brake_atual                                          #percentual do freio (0 - 1)

    global longitude_atual
    global latitude_atual
    global heading_atual
    global speed_atual							                #chamada para a variavel global
    global steering_atual                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_atual                                       #percentual do acelerador (0 - 1)
    global brake_atual                                          #percentual do freio (0 - 1

    #print 'follower_INFO'

    latitude_atual = data.latitude
    longitude_atual = data.longitude
    heading_atual = data.heading_headingValue
    speed_atual = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_atual = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_atual = data.gasPedalPercent_Value
    brake_atual = data.brakePedalPercent_Value
    controla_veiculo()

def listener():
    global leader_topic
    global follower_topic
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print leader_topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(leader_topic,CAM_simplified,leader_info)
    rospy.Subscriber(follower_topic,CAM_simplified,auto_info)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #global leader_topic
    #global follower_topic
    #global publication_topic
    param = sys.argv[1:]                                        #le os dados passados como argumentos (Lider Follower)
    #TODO - tratamento de erros para parametros de entrada
    #TODO - Tornar generico parametros de leitura
    leader_topic = "/" + param[0] +"/carINFO"                   #cria a string do topico do lider
    follower_topic = "/" + param[1] +"/carINFO"                 #cria a string do topico do follower
    publication_topic = "/" + param[1] +"/prius"                 #cria a string do topico do follower
    listener()
