#!/usr/bin/env python
#-----------------------------------------------------------------------
#platooning.py TV (target Vehicle) SV
#Codigo responsavel pelo controle do veiculo seguidor (SV)
#v0
#   Le os dados do TV (local) (param0) - TV (target Vehicle)
#   Le os dados atuais do SV (param1) - SV (subject vehicle)
#   Atua sobre o SV
#v1
#   Atualizando termos e variaveis para nomenclatura padrao
#v2
#   Introduzindo controle de distancia/tempo
#v3
#   Introduzindo controle angulacao
#v4
#   Ajustando delay entre acoes de control 0.05
#v5
#   Adicionando lista de posicoes
#   Adicionando controle do Heading
#v6
#   Corrigindo erro no circulo trigonometrico
#v7 
#   Adjusting the PID Distance
#   Adding the Normal and Lost Mode
#v8 
#   Adding the bearing
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
import numpy as np

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

longitude_SV_old = 0.0
latitude_SV_old = 0.0

#variaveis de controle --------------------------------------------
kp_dis = 1.5        #0.9
ki_dis = 0.001       #0.01
kd_dis = 0.5        #0.3

dis_integral = 0.0
dist_deriv = 0.0
erro_dis_old = 0.0
erro_dis = 0.0

kp_theta = 5.0      #9.0    #6.0
ki_theta = 0.005     #0.1    #0.05
kd_theta = 1.0      #3.5    #3.0

theta_integral = 0.0
theta_deriv = 0.0
theta_erro_old = 0.0
theta_error = 0.0


#variaveis gerais -------------------------------------------------
TV_topic = 0                                           #topico do TV
SV_topic = 0                                         #topico do SV
publication_topic = 0                                      #topico a publicar dados do veiculo

time_distance_control=0.0
time_direction_control = 0.0
count = 0
lap = 0
count_lap = 0
error_steer = 0.0
control_error_steer  = 0.0
steer_integral  = 0.0
steer_deriv  = 0.0
status  = 0
mode = 1

TV_position_vector = np.empty([1,4])

#defines------------------------------------------------------------
MAX_SPEED = 30.0/3.6			#velocidade maxima do veiculo (m/s)
MIN_SPEED = 0.0/3.6				#velocidade minima do veiculo (m/s)
MAX_STEER = 19.5				#maxima angulacao da roda em graus
MIN_STEER = -19.5				#minima angulacao da roda em graus

ANG_TO_RAD	= np.pi/180			#converte graus para radiano
RAD_TO_ANGLE = 180/np.pi		#converte radiano para graus

KMH_TO_MS = 1/3.6				#converte de Km/H para m/s
MS_TO_KMH = 3.6					#converte de m/s para Km/h
EDGE_LOW_SPEED	= 0.05		    #ajuste para direcionar velocidade para zero (Erro do Gazebo) - m/s
EDGE_ANGLE = 0.01
MIN_VEL = 0.0				#
MAX_ACEL = 1.0				#
MAX_BRAKE = 1.0				#
MIN_ACEL = 0.0				#
MIN_BRAKE = 0.0				#

MINIMUM_DISTANCE = 5            #Safety distance - m
SAFETY_TIME = 0.5               #safety time - s
GPS_RANGE = 3.0                 #range to used in comparison between th SV position and the historical position of TV
MAX_ERRO_INTEGRADOR_DIS = 50    #UP limit for the integrator in distance
MAX_ERRO_INTEGRADOR_THETA = 0.5  #UP limit for the integrator in theta
BEARING_ANGLE_LIM = 0.15        #MIN limit for bearing check (angle)
BEARING_DIST_LIM = 0.5        #MIN limit for bearing check (DIST)

ZERO = 0
NORMAL_MODE = 1                 #platooning mode
LOST_MODE = 0                   #platooning mode

MINIMUM_DT = 0.05               #time between control actions
ON = 1
OFF = 0

WEST =      1
NW =        2
NORTH =     3
NE =        4
EST =       5
SE =        6
SOUTH =     7
SW =        8

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(desired_speed, real_speed)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(desired_speed, real_speed):
    
	erro_vel = desired_speed  - real_speed						#calcula o erro de velocidade -> "/3.6" converte para m/s
	erro_vel_norm = (erro_vel - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) 	#normaliza o erro (0 - 1)
	accel_control = min(erro_vel_norm,1.0)				                #acao de controle
	accel_control = max(accel_control,-1.0)				                #acao de controle

	if(abs(accel_control) < EDGE_LOW_SPEED and desired_speed < EDGE_LOW_SPEED):		#necessario para zerar a velocidade
		aceleracao = MIN_ACEL
		freio = MAX_BRAKE
	else:
		aceleracao = max(0.0,accel_control)				#acao de aceleracao
		freio = max(0.0,-accel_control)					#acao de frenagem
	return aceleracao, freio

#-----------------------------------------------------------------------
#FUNCAO: distance_control(v_TV, v_SV, distance_error)
#   Control the distance between the TV and the SV using the speed of the SV	
#   v_TV -> TV speed
#   v_SV -> SV Speed
#   d_error -> Error between the desired distance and the real distance
#   Return: control_vel (Data responsible for the new desired speed in m/s)
#-----------------------------------------------------------------------
def distance_control (d_error):
    #Global variables-----------------
    global dis_integral
    global dist_deriv
    global erro_dis_old
    global time_distance_control
    
    debug = ON       #defines if the text will be printed
    
    #Calcule dt------------------------------------------------------------
    #used to determine the time between interactions from the program
    time_now = tp.time()									#saves te time of the system
    dt = time_now - time_distance_control					#time between interactions
    if (dt >= MINIMUM_DT):                                  #time between control actions
        time_distance_control = time_now					#save the time for next interaction

        #determines the integrator component
        if (abs(dis_integral) >= MAX_ERRO_INTEGRADOR_DIS or dt>10):
            dis_integral = ZERO
        else:
            dis_integral = dis_integral + d_error * dt
        
        #derivative value
        if (dt < 1):
            dist_deriv = (erro_dis_old - d_error)/dt
        else:
            dist_deriv = (erro_dis_old - d_error)/dt
        erro_dis_old = d_error

        #calcula PID de distancia
        PID_dis_adjust = kp_dis * d_error + ki_dis * dis_integral + kd_dis * dist_deriv
    else:
        PID_dis_adjust = 0.0
    if debug:
        print ("PID_Dis_adjust: ", PID_dis_adjust)
    return PID_dis_adjust
    
#-----------------------------------------------------------------------
#FUNCAO: bearing_calculator()
#	calculates the bearing between SV and the desired position from TV
#-----------------------------------------------------------------------
def bearing_calculator(lat_TV, long_TV, head_TV):
    #global data
    global latitude_SV
    global longitude_SV
    global heading_SV
    E = 0.0
    
    #local
    debug = ON

    if debug:
        print "bearing calculator"
        print "TV: ", lat_TV, long_TV, head_TV
        print "SV: ", latitude_SV, longitude_SV, heading_SV

    d_lat = lat_TV - latitude_SV
    d_long = long_TV - longitude_SV

    if (heading_SV < 0.0):
        theta_SV = 2*math.pi + heading_SV
    else:
        theta_SV = heading_SV

    if ((heading_SV >= 5 * math.pi /4) and (heading_SV < 7 * math.pi /4)):
        if d_long > BEARING_DIST_LIM:
            E = math.atan(d_long/d_lat)
        else:
            E = ZERO
    elif ((heading_SV >= 3 * math.pi /4) and (heading_SV > 5 * math.pi /4)):
        if d_lat > BEARING_DIST_LIM:
            E = math.atan(d_lat/d_long)
        else:
            E = ZERO


    #print ("E: ", E)

    return E


#-----------------------------------------------------------------------
#FUNCAO: direction_control(heading_TV, heading_SV)
#   Control the direction of the SV using the heading of the TV	
#   h_TV -> TV speed
#   h_SV -> SV Speed
#   Return: theta_control 
#-----------------------------------------------------------------------
def direction_control (lat_TV, long_TV, h_TV, h_SV):
    #Global variables-----------------
    global theta_integral
    global theta_deriv
    global theta_erro_old
    global time_direction_control
    global theta_error
    global mode
    debug = ON
    E = 0.0
    #Calcule dt------------------------------------------------------------
    #used to determine the time between interactions from the program
    time_now = tp.time()									#armazena o tempo atual
    dt = time_now - time_direction_control								    #tempo entre iteracoes
    if (dt >= MINIMUM_DT):
        time_direction_control = time_now										#armazena o tempo atual para a proxima iteracao

        if debug:
            print "h_TV: ", h_TV
            print "h_SV: ", h_SV
        #-------------------------------------------------------------------------
        #solving the problemas with diferent positions in the trigonometric circle:
        #ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
        #to solve this problem, if the radian is negative, we add 2pi, in order to turn this number positive
        #However, if the comparison happens between numbers that are really near to zero, we have to 
        #fix the algorithm. If the difference between the angles is bigger then 3.1415, we define the theta_error as da sum of the angles
        #and use the signal from the TV
        # -------------------------------------------------------------------------  
        if (h_TV < 0.0):
            theta_TV = 2*math.pi + h_TV
        else:
            theta_TV = h_TV
        if (h_SV < 0.0):
            theta_SV = 2*math.pi + h_SV
        else:
            theta_SV = h_SV
        if debug:
            print "Theta_TV: ", theta_TV
            print "Theta_SV: ", theta_SV
        
        theta_error = theta_TV - theta_SV
        if (abs(theta_error) < BEARING_ANGLE_LIM): #0.025
            E = bearing_calculator(lat_TV, long_TV, h_TV)
            #E = 2 * math.pi + E
            print "E: ", E
            theta_error = (theta_TV + E) - theta_SV
            print "Theta_error (A): ", theta_error

        #theta_error = (theta_TV - E) - theta_SV
        if (abs(theta_error) > math.pi):
            theta_error = (abs(h_TV) + abs(h_SV))* (h_TV/abs(h_TV))
        
        if debug:
            print "Theta_error: ", theta_error

        if (abs(theta_error) >= EDGE_ANGLE):                #se erro de velocidade significativo
            if debug:
                print "ANGLE_DIFERENTE"
            #determina valor do integrador
            if (abs(theta_integral) >= MAX_ERRO_INTEGRADOR_THETA or dt>10):
                theta_integral = ZERO
            else:
                theta_integral = theta_integral + theta_error * dt
            
            #valor do derivativo
            theta_deriv = theta_erro_old - theta_error
            theta_erro_old = theta_error

            #control action
            theta_control = ( kp_theta * theta_error + ki_theta * theta_integral + kd_theta * theta_deriv)
            
            if debug:
                print "theta_control (A): ", theta_control
                print "Integral: ", theta_integral
                print "Deriv:", theta_deriv

            if (theta_control > MAX_STEER * ANG_TO_RAD):
                theta_control = MAX_STEER * ANG_TO_RAD
            elif (theta_control < - MAX_STEER * ANG_TO_RAD):
                theta_control = - MAX_STEER * ANG_TO_RAD
            if debug:
                print "theta_control: ", theta_control
        else:
            if debug:
                print "ANGLE_IGUAL"
            theta_error = 0.0
            theta_control = 0.0
        return theta_control
    else:
        return 0.0

#-----------------------------------------------------------------------
#FUNCAO: find_direction():
#Compares the current and the last position from SV to find its current direction 
#-----------------------------------------------------------------------
def find_direction(diff_x, diff_y):
    debug = ON
    direction = 0
    
    if (abs(diff_y) <= 0.1 and diff_x < 0.0):
        direction = WEST
        if debug:
            print "DIR: WEST - ", direction 
    elif (diff_y > 0.0 and diff_x < 0.0):
        direction = NW
        if debug:
            print "DIR: - NW", direction
    elif (diff_y > 0.0 and abs(diff_x) <= 0.1):
        direction = NORTH
        if debug:
            print "DIR: NORTH - ", direction
    elif (diff_y > 0.0 and diff_x > 0.0):
        direction = NE
        if debug:
            print "DIR: NE -", direction
    elif (abs(diff_y) <= 0.1 and diff_x > 0.0):
        direction = EST
        if debug:
            print "DIR: EST - ", direction
    elif (diff_y < 0.0 and diff_x > 0.0):
        direction = SE
        if debug:
            print "DIR: SE -", direction
    elif (diff_y < 0.0 and abs(diff_x) <= 0.1):
        direction = SOUTH
        if debug:
            print "DIR: S - ", direction
    elif (diff_y < 0.0 and diff_x < 0.0):
        direction = SW
        if debug:
            print "DIR: SW - ", direction
    else:
        print "IMPOSSIBLE TO DEFINE DIRECTION"
        print "delta_latitude: ", diff_x
        print "delta_longitude: ", delta_longitude_SV

    return direction    


#-----------------------------------------------------------------------
#FUNCAO: compare_lost_position():
#This function compares the SV position with the historical position from TV
#-----------------------------------------------------------------------
def compare_lost_position():
    global TV_position_vector
    global longitude_SV
    global latitude_SV
    global heading_SV
    global latitude_SV_old
    global longitude_SV_old
    global mode
    global count
    
    dist = 0
    dist_min = 999
    pos = 0

    dist_lost = 0
    dist_lost_min = 999
    pos_lost = 0

    direction = 0
    debug = OFF
    
    print "NORMAL MODE!"
    #print "count: ", count
    #count = count +1

    if latitude_SV < 0:
        debug = ON

    delta_latitude_SV = latitude_SV - latitude_SV_old
    delta_longitude_SV = longitude_SV - longitude_SV_old

    if debug:
        print "delta_latitude: ", delta_latitude_SV
        print "delta_longitude: ", delta_longitude_SV
    
    #Find SV direction
    direction = find_direction

    for i in range(TV_position_vector.shape[0]):
        if debug:
            print "i: ", i
            print "TV_position_vector: ", TV_position_vector[i,1], TV_position_vector[i,0], TV_position_vector[i,2], TV_position_vector[i,3]
            print "SV:                 ", longitude_SV, latitude_SV, heading_SV, speed_SV
            print "OLD_SV:             ", longitude_SV_old, latitude_SV_old
        
        #print "D_lat: ", delta_latitude_SV
        #print "D_long: ", delta_longitude_SV
        
        if (direction == WEST and TV_position_vector[i,0] < latitude_SV):                                             #1
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 1"
        elif (direction == NW and TV_position_vector[i,0] < latitude_SV and TV_position_vector[i,1] > longitude_SV):   #2
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 2"
        elif (direction == NORTH and TV_position_vector[i,1] > longitude_SV):                                           #3
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 3"
        elif (direction == NE and TV_position_vector[i,0] > latitude_SV and TV_position_vector[i,1] > longitude_SV):   #4
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 4"
        elif (direction == EST and TV_position_vector[i,0] > latitude_SV):                                            #5
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 5"
        elif (direction == SE and TV_position_vector[i,0] > latitude_SV and TV_position_vector[i,1] < longitude_SV):   #6
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 6"
        elif (direction == SOUTH and TV_position_vector[i,1] < longitude_SV):                                           #7
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 7"
        elif (direction == SW and TV_position_vector[i,1] < longitude_SV and TV_position_vector[i,0] < latitude_SV):   #8
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "OP 8"
        else:
            print "VALOR NAO A FRENTE"
            dist_lost = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if dist_lost_min > dist_lost and dist_lost > 0.0:
                pos_lost = i
                dist_lost_min = dist_lost


        if debug:
            print "Dist: ", dist
            
        #if dist > GPS_RANGE and dist_min > dist:
        if dist_min > dist and dist > 0.0:
            pos = i
            dist_min = dist             #else, update dist_min value
            #break
        elif dist_min < dist:
            break
        #    dist_min = dist             #else, update dist_min value
        elif (i == (TV_position_vector.shape[0] - 1)):
            pos = pos_lost

    
    print "TV: ", TV_position_vector[pos,0], TV_position_vector[pos,1] 
    print "CUrrent SV: ", latitude_SV, longitude_SV
    print "OLD_SV: ", latitude_SV_old, longitude_SV_old

    heading_out = TV_position_vector[pos,2]
    speed_out = TV_position_vector[pos,3]
    lat_out = TV_position_vector[pos,0]                 #update the value of TV_latitude to be compared in bearing
    long_out = TV_position_vector[pos,1]                #update the value of TV_longitude to be compared in bearing

    TV_position_vector = TV_position_vector[pos:,:]     #clean the vector, excluding the old positions


    return lat_out, long_out, heading_out,speed_out

#-----------------------------------------------------------------------
#FUNCAO: vehicle_general_control()
#	
#-----------------------------------------------------------------------
def vehicle_general_control():
    #TV data
    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    #SV data
    global longitude_SV
    global latitude_SV
    global heading_SV
    global speed_SV							                #chamada para a variavel global
    global steering_SV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo), speed_TV
    global throttle_SV                                       #percentual do acelerador (0 - 1)
    global brake_SV                                          #percentual do freio (0 - 1)

    global latitude_SV_old
    global longitude_SV_old

    global publication_topic
    global erro_vel
    global TV_position_vector

    debug = ON

    pub = rospy.Publisher(publication_topic, Control, queue_size=10)				#topico de controle do carro
    msg = Control()											                        #Define msg como um dado do tipo Control -> comando de controle do veiculo
    msg.header.stamp = rospy.Time.now();					                        #tempo atualizado da leitura
    msg.header.frame_id = "base_link";						
    msg.steer = 0 * ANG_TO_RAD						                                #angulacao desejada para as rodas (0-20) GRAUS

    print "------------------------------------------------------"
    #calcula distancia
    distance = ((longitude_TV - longitude_SV)**2 + (latitude_TV - latitude_SV)**2)**0.5                                 #real distance between TV and SV
    maximum_distance = MINIMUM_DISTANCE + speed_TV * SAFETY_TIME                                                        #desisred distance
    distance_error = distance - maximum_distance                                                                        #error between desired ditance and real distance

    if debug:
    #    print "Distance", distance
    #    print "maximum_distance: ", maximum_distance
        print "Distance Error: ", distance_error
    if (TV_position_vector[0,0]<=0.1 and TV_position_vector[0,1]<=0.1):                                                 #cleaning the first position of the vector
        TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV, speed_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
        TV_position_vector = TV_position_vector[1:,:]                                                                   #delete the first line with empty values
    else:
        TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV, speed_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
        
    if debug:
        print "TV_position_vector shape: ", TV_position_vector.shape
    #print TV_position_vector
    
    #ajusta velocidade e aceleracao
    if (speed_TV >= EDGE_LOW_SPEED and distance >= MINIMUM_DISTANCE):               #TV is moving and the distance is significative
        if debug:
            print "TV Moving!"
        PID_dist = distance_control (distance_error)                                                #determines de control action for the distance
        lat_compare, long_compare, heading_compare, speed_compare = compare_lost_position()         #compares the SV_position with the vector of positions
        new_speed_SV = speed_compare + PID_dist                                                     #defines the new speed for the SV
        msg.throttle, msg.brake = calc_throttle_brake(new_speed_SV,speed_SV)                        #determine throttle and brake
        msg.steer = direction_control (lat_compare, long_compare, heading_compare, heading_SV)
    else:
        print "Distance < MINIMUM_distance"

    latitude_SV_old = latitude_SV
    longitude_SV_old = longitude_SV

    #TODO in order to test, force the steer to be the same as the TV
    #msg.steer = steering_TV

    #print "throttle: ", msg.throttle
    #print "brake: ", msg.brake
    #print "steer: ", msg.steer
    #Atualiza topico carro
    pub.publish(msg)										#publica a mensagem desejada no topico 'car1/prius'


def SV_info(data):

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
    vehicle_general_control()

def TV_info(data):

    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    global lap
    global count_lap

    global TV_position_vector
    #print 'TV_INFO'

    latitude_TV = data.latitude
    longitude_TV = data.longitude
    heading_TV = data.heading_headingValue
    speed_TV = data.speed_speedValue						# salva o valor da velocidade no instante
    steering_TV = data.steeringWheelAngle_steeringWheelAngleValue
    throttle_TV = data.gasPedalPercent_Value
    brake_TV = data.brakePedalPercent_Value

    


def listener():
    global TV_topic
    global SV_topic
    #global TV_position_vector
    #TV_position_vector = np.resize(TV_position_vector,(1,4))
    #print "listener - TV_position_vector shape: ", TV_position_vector.shape
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print TV_topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(TV_topic,CAM_simplified,TV_info)
    rospy.Subscriber(SV_topic,CAM_simplified,SV_info)

    #print "Aqui"

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #global TV_topic
    #global SV_topic
    #global publication_topic
    param = sys.argv[1:]                                #le os dados passados como argumentos (TV SV)
                                                        #param[0] - TV
                                                        #param[1] - SV
    #TODO - tratamento de erros para parametros de entrada
    #TODO - Tornar generico parametros de leitura
    TV_topic = "/" + param[0] +"/carINFO"                   #cria a string do topico do TV a ser lido
    SV_topic = "/" + param[1] +"/carINFO"                   #cria a string do topico do SV a ser lido
    publication_topic = "/" + param[1] +"/prius"            #cria a string do topico do SV a ser publicado (controle)
    listener()
