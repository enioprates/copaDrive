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
#v6
#	Limitando o crescimento do fator de integracao
#	Diminuindo o tempo de resposta pela metade
#	PID funcionando! (Curva e Reta)
#v7
#	Comentando o texto
#	Substituindo magical numbers
#v7.1
#	lendo sonares
#	criando funcao separada do PID
#v7.3 
#	desvio de multiplos obstaculos (carros)
#v7.4
#	acrescentando sonares laterais
#v7.5
#	mudando o topico de publicacao do carro de drive_param para car1/drive_param
#	mudando limitacao de velocidade maxima
#v7.6
#	deixando de publicar no topico de ERROS
#v8.0
#	reduzindo velocidade em curvas   (REDUCED_SPEED)
#-----------------------------------------------------------------------

#Import commands
import rospy												#biblioteca de python do ROS
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords						#leitura de coordenadas do carro
from image_processing.msg import AckermannDriveStamped		#RETIRAR
from image_processing.msg import error_control				#parametros para tratamentos de erro (e impressao em arquivo)
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
from ros_its_msgs.msg import Sonar						#msg do tipo Sonar
import math								
import numpy as np
import time as tp
import csv
#------------------------------------------------------------------------
#Publication Topic
pub = rospy.Publisher('car1/prius', Control, queue_size=10)				#topico de controle do carro
pub_2 = rospy.Publisher('car1/drive_parameters', drive_param, queue_size=1)	#parametros de velocidade do veiculo
#pub_3 = rospy.Publisher('car1/error_control', error_control, queue_size=1) 	#parametros de erro para controle e impressao
#------------------------------------------------------------------------

#global variables  
speed_real = 0.0											#velocidade global do veiculo
status = 0													#indica se o veiculo esta em velocidade normal ou reduzindo
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


#PID de velocidade (NAO UTILIZADO!)
kp_vel=2				#Kp
ki_vel=0				#Ki
kd_vel=0				#Kd

#PID das rodas
kp_steer_l= 18.0						#Kp 40 km - 3.2			kp 50 km 5.5  5
ki_steer_l= 0.00						#Ki 40 km - 0.01		Ki 50 km 0.01   0
kd_steer_l= 7.0						#Kd 40 km - 0.2			Kd 50 km 0.4  0.45

kp_steer_c= 18.0 						#Kp 40 km - 4.0			Kp 50 km 5.5
ki_steer_c= 0.00						#Ki 40 km - 0.005		Ki 50 km 0.075
kd_steer_c= 7.0 						#Kd 40 km - 0.25		Kd 50 km 0.25

steer_integral = 0.0
steer_deriv = 0.0			

#used variables
tp_corr = 1 							#segundos  - tempo correcao
time1 = 0.0								#variavel de contagem do tempo etre iteracoes (dt)

#parametros Iniciais
velocity = 0.0							#velocidade inicial do veiculo
prev_error_steer=0.0					#erro anterior de angulacao
prev_error_vel=0.0						#erro anterior de velocidade
prev_theta = 0.0						#angulo anterior (usar no derivativo)
error_steer=0.0							#erro atual de angulacao
error_vel=0.0							#erro atual de velocidade
angle=0.0								#angulo do veiculo
count = 0								#variavel de teste de numero de iteracoes
						
ON = 1
OFF = 0

REDUCED_SPEED = OFF


MAX_SPEED = 50.0/3.6			#velocidade maxima do veiculo (m/s)
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
MIN_SLOPE = 1.0				#indica deteccao de CURVA (<=1.0) OU RETA 
MAX_ERRO_INTEGRADOR	= 30.0 #valor maximo do I para correcao


NORMAL = 1					#status da conducao 
CORRECAO = 0				#status da conducao
ZERO = 0					#
FRENTE = 1					#direcoes de obstaculos
ESQUERDA = 2				#direcoes de obstaculos
DIREITA = 3					#direcoes de obstaculos
LADO_DIREITO = 4			#direcoes de obstaculos
LADO_ESQUERDO = 5			#direcoes de obstaculos
INDEFINIDO = 6				#direcoes de obstaculos
LIVRE = 0					#direcoes de obstaculos
DESVIO = 15.0				#angulo de desvio em caso de obstaculo

COLLISION = 0.40

VEL_CRUZEIRO = 50.0			#KM/H	50
VEL_FRENAGEM = 30.0			#KM/H	30


#-----------------------------------------------------------------------

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(s_sp, s_real) -> variaveis recebidas em km/h
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
#FUNCAO: calc_PID(data)
#	Executa os comandos do PID
#	Retorna o erro da variavel de controle
#-----------------------------------------------------------------------
def calc_PID( theta_error, dt, slope, x, x1):
	global steer_integral
	global prev_theta
	global steer_deriv
	if abs(steer_integral) >= MAX_ERRO_INTEGRADOR or abs(theta_error) <= 0.1:			#zerando erro do integrador
		steer_integral = ZERO
	else:	
		steer_integral = steer_integral + theta_error * dt								#increementando o integrador
	steer_deriv = prev_theta - theta_error

	if abs(slope) <= MIN_SLOPE and x-x1 > 10:											#definindo uma acao de controle para curva ou reta
		control_error_steer = - (kp_steer_c*theta_error + ki_steer_c * steer_integral + kd_steer_c * steer_deriv)
		#print 'CURVA: ', slope
	else:
		control_error_steer = - (kp_steer_l*theta_error + ki_steer_l * steer_integral + kd_steer_l * steer_deriv)
		#print 'RETA: ', slope
	
	return control_error_steer

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


#-----------------------------------------------------------------------
#FUNCAO: trata_sonar(data)
#	Le os dados do sonar do veiculo
# 	reyorna a posicao do obstaculo na pista
#-----------------------------------------------------------------------
def trata_sonar():
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
	desvio = 0
	divisor = 2

	var_print = 0

	if (range_LF < max_range_LF and range_RF < max_range_RF):
		obstacle = FRENTE
		if range_LF < max_range_LF/divisor or range_RF < max_range_RF/divisor:
			desvio = DESVIO * 2
		else:
			desvio = DESVIO
	elif range_LF < max_range_LF:
		obstacle = ESQUERDA
		if range_LF < max_range_LF/divisor:
			desvio = - DESVIO * 2
		else: 
			desvio = - DESVIO 
	elif  range_RF < max_range_RF:
		obstacle = DIREITA
		if range_RF < max_range_RF/divisor:
			desvio = DESVIO * 2
		else:
			desvio = DESVIO 
	elif range_LM < max_range_LM:
		obstacle = ESQUERDA
		if range_LM < max_range_LM/divisor:
			desvio = - DESVIO * 2
		else:
			desvio = - DESVIO 
	elif range_RM < max_range_RM:
		obstacle = DIREITA
		if range_RM < max_range_RM/divisor:
			desvio = DESVIO *2
		else:
			desvio = DESVIO 
	elif range_side_L < max_range_side_L:
		obstacle = LADO_ESQUERDO
		desvio = - DESVIO/2 
	elif range_side_R < max_range_side_R:
		obstacle = LADO_DIREITO
		desvio = DESVIO/2 
	else:
		obstacle = LIVRE

	if (range_LF < COLLISION or range_LM < COLLISION  or range_RM < COLLISION or range_RF < COLLISION or range_side_L < COLLISION or range_side_R < COLLISION):
		if var_print:
			print "COLLISION!"
			print "range_LF: ", range_LF
			print "range_LM: ", range_LM
			print "range_RM: ", range_RM
			print "range_RF: ", range_RF
			print "range_side_L: ", range_side_L
			print "range_side_R: ", range_side_R
		
		

	return obstacle,desvio

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
	global count
	var_print = 0
	msg = Control()											#Define msg como um dado do tipo Control -> comando de controle do veiculo
	msg.header.stamp = rospy.Time.now();					#tempo atualizado da leitura
	msg.header.frame_id = "base_link";						
	msg.steer = data.angle * ANG_TO_RAD						#angulacao desejada para as rodas (0-30) GRAUS
	
	if var_print:
		print "----START-vel_and_angle----------"
		print "vel: ", data.velocity
		print "speed_real: ", speed_real
		print "steer: ", data.angle * ANG_TO_RAD
	
	if (data.angle * ANG_TO_RAD) > 0.05 and REDUCED_SPEED:
		#A variavel data.velocity eh dada em km/h e speed_real eh definida em m/s
		msg.throttle, msg.brake = calc_throttle_brake (VEL_FRENAGEM, speed_real)	#funcao que calcul aceleracao e frenagem
	else:
		#A variavel data.velocity eh dada em km/h e speed_real eh definida em m/s
		msg.throttle, msg.brake = calc_throttle_brake (data.velocity, speed_real)	#funcao que calcul aceleracao e frenagem

	# if count < 100:
	# 	print "count: ",count
	# 	msg.throttle = 1.0
	# 	msg.brake = 0.0
	# else:
	# 	print "count: ",count
	# 	msg.throttle = 0.0
	#	msg.brake = 0.0
	#print "Throttle=",msg.throttle
	#print "brake=",msg.brake 
	#print "angle=",data.angle
	#print "----END-vel_and_angle----------"
	pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'


#-----------------------------------------------------------------------
#FUNCAO: car_info(data)
#	Funcao chamada pelo topico de dados atuais do veiculo ('/car1/carINFO')
#-----------------------------------------------------------------------
def car_info(data):
	global speed_real										#chamada para a variavel global
	global longitude_real
	global latitude_real
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
	global count											#numero de iteracoes
	global time1											#tempo entre execucoes
	global steer_integral									#acumulador do erro
	global steer_deriv
	global range_LF											#leitura do sonar
	global range_LM											#leitura do sonar
	global range_RM											#leitura do sonar
	global range_RF											#leitura do sonar
	var_print = 0
	
	if var_print:
		print "-----------Vehicle CONTR0L ----------"	
		print "count", count
	obstacle,desvio = trata_sonar()								#calcula se ha obstaculo a frente
	#print 'Obstacle: ', obstacle

	#define o dt--------------------------------------------------------------------------------------------------------------------------------------
	time_now = tp.time()									#armazena o tempo atual
	if (count == 0):
		dt = 0.0											#inicializa a variavel
	else:	
		dt = time_now - time1								#tempo entre iteracoes
	time1 = time_now										#armazena o tempo atual para a proxima iteracao

	#armazenando os dados lidos do topico--------------------------------------------------------------------------------------------------------------------------------------
	x = data.X2												#posicao X2 da linha detectada
	y = data.Y2												#posicao Y2 da linha detectada
	x1 = data.X1
	slope = data.slope
	
	#definindo o erro de posicao do veiculo em relacao a linha
	error_steer = (x-400)/400								#posicao X da linha em relacao ao centro da imagem 800 pixels
	if var_print:
		print "Error: ",error_steer
	
	#sistema de controle de velocidade--------------------------------------------------------------------------------------------------------------------------------------
	#print "Count: ",count
	velocity = VEL_CRUZEIRO									#atualiza velocidade do veiculo
	count = count+1											#conta numero de iteracoes
	if var_print:
		print "vel:",velocity

	#Controle de tempo de resposta--------------------------------------------------------------------------------------------------------------------------------------
	if abs(slope) <= MIN_SLOPE and x-x1 > 10:
		dist_corr = velocity * KMH_TO_MS * 0.5
	else:
		dist_corr = velocity * KMH_TO_MS * 1						#distancia a ser percorrida em metros em 1 s

	#define o erro em termos de angulo--------------------------------------------------------------------------------------------------------------------------------------
	if (velocity != ZERO):
		dist_ang = error_steer/dist_corr						#angulo do erro em relacao a reta detectada
	else:
		dist_ang = ZERO
	#print "cat op sobre hip: ", error_steer/dist_corr

	if(abs(dist_ang)<=1 and velocity != ZERO):
		theta_error = np.arcsin(error_steer/dist_corr) * RAD_TO_ANGLE	#correcao normal
	elif (velocity != ZERO):
		theta_error = np.arcsin((error_steer/abs(error_steer))/(dist_corr/abs(dist_corr))) * RAD_TO_ANGLE	#se a distancia inicial for pequena para o erro
	else:
		theta_error = ZERO					#caso nao exista uma linha detectada
	
	if var_print:
		print "theta_error: ", theta_error

	#Calculando o PID--------------------------------------------------------------------------------------------------------------------------------------
	if abs(steer_integral) >= MAX_ERRO_INTEGRADOR or abs(theta_error) <= 0.1:		#se o integrador comacar a crescer demais, zera
		steer_integral = ZERO
	else:
		steer_integral = steer_integral + theta_error * dt							#incrementa o integrador
	if var_print:
		print "steer_integral: ", steer_integral

	steer_deriv = prev_theta - theta_error											#calcula a diferenca do erro
	#print "previous error:", (prev_theta - theta_error)

	#sistema de controle de angulacao
	if abs(slope) <= MIN_SLOPE and x-x1 > 10:
		control_error_steer = -(kp_steer_c*theta_error + ki_steer_c * steer_integral + kd_steer_c * steer_deriv )
		if var_print:
			print 'CURVA: ', slope
	else:
		control_error_steer = -(kp_steer_l*theta_error + ki_steer_l * steer_integral + kd_steer_l * steer_deriv )
		if var_print:
			print 'RETA: ', slope
	if var_print:
		print 'X2 - X1', x-x1

	
	#control_error_steer = calc_PID( theta_error, dt, slope, x, x1)
	#print 'Teste: ',control_error_steer

	prev_theta = theta_error							#atualiza o valor do erro
	
	if (obstacle == LIVRE):								# se nao houver obstaculo a frente
		angle = control_error_steer#*ANG_TO_RAD				#atualiza o angulo de controle
		#print 'LIVRE'
	else:												#atuacao em caso de obstaculo a frente
		angle = desvio
		prev_theta = 0
		steer_integral = ZERO
		velocity = VEL_FRENAGEM
		#print 'OBSTACLE'



	if angle > MAX_STEER:								#limita o angulo maximo
		angle = MAX_STEER
	if angle < -MAX_STEER:
		angle = -MAX_STEER
	#print "angle:",angle
	#--------------------------
	if var_print:
		print "-----------END Vehicle CONTROL ----------"

	#prepara msg para publicacao------------------------------------------------------------------------------------------------------------------------------
	msg = drive_param()
	msg.velocity = velocity
	msg.angle = angle
	pub_2.publish(msg)

	#msg_e = error_control()
	#msg_e.error_steer = error_steer
	#msg_e.control_error_steer = control_error_steer
	#msg_e.steer_integral = steer_integral
	#msg_e.steer_deriv = steer_deriv
	#msg_e.status = status
	#pub_3.publish(msg_e)

#-----------------------------------------------------------------------
#FUNCAO: listener()
#	inicializa o no
#	assina os topicos de leitura desejados
#-----------------------------------------------------------------------
def listener():
	rospy.init_node('PriusHybridPlugin', anonymous=True)			#inicializa o no
	rospy.Subscriber('X_Y', coords, vehicle_control)				#subscreve no topico e chama a funcao vehicle_control
	rospy.Subscriber('/car1/drive_parameters', drive_param, vel_and_angle)#subscreve no topico e chama a funcao vel_and_angle
	rospy.Subscriber('/car1/carINFO',CAM_simplified,car_info)		#subscreve no topico e chama a funcao car_info
	rospy.Subscriber('/car1/front_sonar_left_far_range',Sonar,sonar_front_LF)#subscreve ao topicoi e chama a funcao sonar_LF
	rospy.Subscriber('/car1/front_sonar_left_middle_range',Sonar,sonar_front_LM)#subscreve ao topicoi e chama a funcao sonar_LM
	rospy.Subscriber('/car1/front_sonar_right_middle_range',Sonar,sonar_front_RM)#subscreve ao topicoi e chama a funcao sonar_RM
	rospy.Subscriber('/car1/front_sonar_right_far_range',Sonar,sonar_front_RF)#subscreve ao topicoi e chama a funcao sonar_RF
	rospy.Subscriber('/car1/side_sonar_left_front_range',Sonar,sonar_side_L)#subscreve ao topicoi e chama a funcao sonar_RF
	rospy.Subscriber('/car1/side_sonar_right_front_range',Sonar,sonar_side_R)#subscreve ao topicoi e chama a funcao sonar_RF
	rospy.spin()													#mantem o listener aberto

if __name__ == '__main__':
	print "Simulation_Connector"
	if REDUCED_SPEED:
		print "REDUCED SPEED IN CURVES"
	listener()


