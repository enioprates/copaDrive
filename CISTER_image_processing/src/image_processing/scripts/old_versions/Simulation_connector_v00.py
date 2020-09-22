#!/usr/bin/env python
#-----------------------------------------------------------------------
#Simulation_connector.py
#Codigo responsavel pela ligacao entre o sistema de controle e o simulador do veiculo
#v0
#	Codigo alterado para ler os dados de velocidade do topico carINFO
#	Criada a funcao car_info
#-----------------------------------------------------------------------

#Import commands
import rospy
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import AckermannDriveStamped
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math

#Publication Topic
pub = rospy.Publisher('car1/prius', Control, queue_size=10)	#topico de controle do carro

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
	msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
	msg.header.stamp = rospy.Time.now();					#tempo atualizado da leitura
	msg.header.frame_id = "base_link";						
	msg.throttle = data.velocity							#aceleracao desejada 	(0-1)
	#msg.brake = data.brake									#frenagem desejada 		(0-1)
	msg.steer = data.angle									#angulacao desejada para as rodas (0-30) GRAUS OU RADIANOS?
	if data.velocity == 0:
		msg.brake = 1
	print "vel=", data.velocity
	print "angle=",data.angle
	pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'

#-----------------------------------------------------------------------
#FUNCAO: car_info(data)
#	Funcao chamada pelo topico de dados atuais do veiculo ('/car1/carINFO')
#-----------------------------------------------------------------------
def car_info(data):
	print "teste"
	print "real_vel=", data.speed_speedValue
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
#FUNCAO: listener()
#	inicializa o no
#	assina os topicos de leitura desejados
#-----------------------------------------------------------------------
def listener():
	rospy.init_node('PriusHybridPlugin', anonymous=True)			#inicializa o no
	rospy.Subscriber('/car1/carINFO',CAM_simplified,car_info)		#subscreve no topico e chama a funcao car_info
	rospy.Subscriber('drive_parameters', drive_param, vel_and_angle)#subscreve no topico e chama a funcao vel_and_angle
	rospy.spin()													#mantem o listener aberto

if __name__ == '__main__':
	listener()


