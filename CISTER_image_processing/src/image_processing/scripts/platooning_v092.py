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
#   Adding the bearing (FIXED)
#   Adjusting the angle following (theta error)
#   adjustin the angles of the vehicles given the position of the road
#   Removing Normal and Lost mode - there is only one mode to find the most near position in front of the vehicle
#v9
#   Adjusting PID values
#   Removing "/dt" from PID calculation
#-----------------------------------------------------------------------
import rospy
import sys                                                  #biblioteca para leitura de param via linha de comando
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords
#from image_processing.msg import AckermannDriveStamped
#from image_processing.msg import error_control
from prius_msgs.msg import Control							#comandos de controle de aceleracao e frenagem do veiculo
from ros_its_msgs.msg import CAM_simplified 				#codigo adicionado. NECESSARIO adicionar no CMAKEList do pacote ros_its_msg
import math                                                 #library for math functions
import numpy as np
import time as tp
import csv
import numpy as np
#------------------------------------------------------------------------

#Publication Topic

#------------------------------------------------------------------------
#global variables  
#data from the TV --------------------------------------------
car_name_TV = 0                                          #TV name
longitude_TV = 0.0                                       #longitude
latitude_TV = 0.0                                        #latitude 
heading_TV = 0.0                                         #heading (angulacao global do veiculo)
speed_TV = 0.0											 #vehicle speed (m/s)
steering_TV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_TV = 0.0                                        #percentual do acelerador (0 - 1)
brake_TV = 0.0                                           #percentual do freio (0 - 1)

#data from the SV --------------------------------------------
longitude_SV = 0.0                                       #longitude
latitude_SV = 0.0                                        #latitude 
heading_SV = 0.0                                         #heading (angulacao global do veiculo)
speed_SV = 0.0											#vehicle speed (m/s)
steering_SV = 0.0                                        #steering (angulacao das rodas do veiculo em relacao ao veiculo)
throttle_SV = 0.0                                        #percentual do acelerador (0 - 1)
brake_SV = 0.0                                           #percentual do freio (0 - 1)

#variaveis de controle --------------------------------------------
kp_dis = 1.5        #0.9                                #Distance Proportional Controller
ki_dis = 0.001       #0.01                              #Distance Integral Controller
kd_dis = 0.5        #0.3                                #Distance Derivative Controller

dis_integral = 0.0                                      #Distance Integral Acumulator
dist_deriv = 0.0                                        #Distance derivative data (does not need to be global)
erro_dis_old = 0.0                                      #Distance previous error

kp_theta = 5.0      #9.0    #6.0                        #Angle Proportional Controller
ki_theta = 0.005     #0.1    #0.05                      #Angle Integral Controller
kd_theta = 1.0      #3.5    #3.0                        #Angle Derivative Controller

theta_integral = 0.0                                    #Angle Integral Acumulator
theta_deriv = 0.0                                       #Angle derivative data (does not need to be global)
theta_erro_old = 0.0                                    #Angle previous error
theta_error = 0.0

#variaveis gerais -------------------------------------------------
TV_topic = 0                                            #topico do TV
SV_topic = 0                                            #topico do SV
publication_topic = 0                                   #topico a publicar dados do veiculo

time_distance_control = 0.0                             #time of last distance control interaction 
time_direction_control = 0.0                            #time of last direction control interaction 

TV_position_vector = np.empty([1,4])                    #vector to store TV positions to be followed

#debug tools--------------------------------------------------------
count = 0                                               
lap = 0
count_lap = 0
mode = 1
LOST = 0
NORMAL = 1

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
BEARING_DIST_LIM = 1.5        #MIN limit for bearing check (DIST)
ANGLE_FRONT = math.pi/4         #angle to determine if the historical point is in fron or not
ANGLE_ADJUST = math.pi/2        #adjust provided by the difference between the trigonometric circle and the road

ZERO = 0

MINIMUM_DT = 0.02               #time between control actions
ON = 1                          #Debug mode
OFF = 0                         #Debug mode

#-----------------------------------------------------------------------
#FUNCAO: calc_throttle_brake(desired_speed, real_speed)
#	Calcula parametros de aceleracao e frenagem
#	Retorne a acelaracao e a frenagem do veiculo
#-----------------------------------------------------------------------
def calc_throttle_brake(desired_speed, real_speed):
    
	erro_vel = desired_speed  - real_speed						        #calcula o erro de velocidade -> "/3.6" converte para m/s
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
#FUNCAO: distance_control(d_error)
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
    global time_distance_control                            #time of last distance control interaction
    
    debug = OFF                                              #defines if the text will be printed
    
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
            dist_deriv = (erro_dis_old - d_error)
        else:
            dist_deriv = (erro_dis_old - d_error)
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
#   return bearing angle
#-----------------------------------------------------------------------
def bearing_calculator(d_lat, d_long, lat_TV, long_TV, head_TV, head_SV):
    
    #global data
    global latitude_SV                      #SV latitude
    global longitude_SV                     #SV longitude
    bearing_angle = 0.0                                 #bearing
    debug = OFF                             #modo de debug

    bearing_angle = math.atan2 (d_long, d_lat)          #bearing calculation

    if debug:
        print "bearing calculator"
        print "TV: ", lat_TV, long_TV, head_TV
        print "SV: ", latitude_SV, longitude_SV, head_SV
        print "d_long: ", d_long
        print "d_lat: ", d_lat
        print "bearing_angle: ", bearing_angle
    
    if (bearing_angle < 0.0 and head_SV > math.pi/4):   #bearing adjust if bearing is negative
        bearing_angle = bearing_angle + 2 * math.pi

    return bearing_angle

#-----------------------------------------------------------------------
#FUNCAO: direction_control(heading_TV, heading_SV)
#   Control the direction of the SV using the heading of the TV	
#   h_TV -> TV speed
#   h_SV -> SV Speed
#   Return: theta_control 
#-----------------------------------------------------------------------
def direction_control (lat_TV, long_TV, h_TV, h_SV):
    #Global variables-----------------
    global theta_integral                                   #Angle Integral Acumulator
    global theta_deriv                                      #Angle Derivative Acumulator
    global theta_erro_old                                   #Angle previous error
    global theta_error                                      #Angle current error
    global time_direction_control                           #time of last direction control interaction
    global latitude_SV                                      #SV latitude
    global longitude_SV                                     #SV Longitude
    debug = OFF                                              #modo debug
    bearing_angle = 0.0                                     #bearing Angle
    
    #Calcule dt------------------------------------------------------------
    #used to determine the time between interactions from the program
    time_now = tp.time()									#armazena o tempo atual
    dt = time_now - time_direction_control					#tempo entre iteracoes
    if (dt >= MINIMUM_DT):                                  #act only after new controller variables
        time_direction_control = time_now					#armazena o tempo atual para a proxima iteracao

        if debug:
            print "h_TV: ", h_TV
            print "h_SV: ", h_SV

        #------------------------------------------------------------------------
        #adjusting the angles given the rotation between the circle and the road (pi/2)
        #------------------------------------------------------------------------
        h_TV = h_TV - ANGLE_ADJUST
        h_SV = h_SV - ANGLE_ADJUST

        #-------------------------------------------------------------------------
        #solving the problemas with diferent positions in the trigonometric circle:
        #ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
        #to solve this problem, if the radian is smaller then -45, we add 2pi, in order to turn this number positive
        # -------------------------------------------------------------------------  
        if (h_TV < - math.pi/4):                #adjusting h_tv
            theta_TV = 2*math.pi + h_TV         #adding 2pi in order to change for positive
        else:
            theta_TV = h_TV                     #if not, keep the angle
        
        if (h_SV < - math.pi/4):                #adjusting h_sv
            theta_SV = 2*math.pi + h_SV         #adding 2pi in order to change for positive
        else:
            theta_SV = h_SV                     #if not, keep the angle
        
       
        theta_error = theta_TV - theta_SV       #calculate theta_error
        
        if debug:
            print "Theta_TV: ", theta_TV
            print "Theta_SV: ", theta_SV
            print "theta_error: ", theta_error

        #find the distance between the current SV point and the evaluated TV point
        d_lat = lat_TV - latitude_SV            #calculates the diference in latitude
        d_long = long_TV - longitude_SV         #calculates the diference in longitude
        dist = (d_lat**2 + d_long**2)**0.5      #calculates the distance

        #---------------------------------------------------------------------------
        #   Evaluate if the current position of SV is displaced from TV even if the angle theta_error is small
        #   Only checked if theta_error is small enough and if the distance between the evaluated points are not too small
        #   If the distance is very small, the atan2 function does not work properly, generating errors
        #---------------------------------------------------------------------------
        if (abs(theta_error) < BEARING_ANGLE_LIM and dist > BEARING_DIST_LIM):
            bearing_angle = bearing_calculator(d_lat, d_long, lat_TV, long_TV, theta_TV, theta_SV)  #calls the bearing angle function
            
            diff = abs(theta_TV - bearing_angle)                        #the difference between the bearing angle and the TV angle
                        
            if (bearing_angle > theta_SV):                              #determine if the bearing adjust is positive or negative
                theta_error = theta_TV - theta_SV + diff                #is positive if bearing angle is bigger then the SV angle
            else:
                theta_error = theta_TV - theta_SV - diff                #is negative if bearing angle is bigger then the SV angle
            
            if debug:
                print "bearing_angle: ", bearing_angle
                print "diff: ", diff
                print "Theta_error (A): ", theta_error

        #---------------------------------------------------------------------------
        #   Sometimes, SV angle and TV angle does not match the trigonometric adjust conditions at the same time
        #   In this situations, the difference between those angles is very big and out the trigonometric circle
        #   To adjust this, if the error outpass PI, the previous adjust is desconsidered and the simple error between the angles is considered
        #---------------------------------------------------------------------------
        if (abs(theta_error) > math.pi):                    
            theta_error = h_TV - h_SV
            #if debug or abs(theta_error) > 0.25:
            #    print "Theta_error: ", theta_error

        #---------------------------------------------------------------------------
        #   PID controller for the Angular Position
        #---------------------------------------------------------------------------
        if (abs(theta_error) >= EDGE_ANGLE):                            #If theta_error is significative
            if debug:
                print "ANGLE_DIFERENTE"
            
            #Update the angular integrator value
            if (abs(theta_integral) >= MAX_ERRO_INTEGRADOR_THETA or dt>10):
                theta_integral = ZERO
            else:
                theta_integral = theta_integral + theta_error * dt
            
            #Update the angular derivative value
            #theta_deriv = (theta_erro_old - theta_error)/dt
            theta_deriv = (theta_erro_old - theta_error)
            #if debug or abs(theta_error) > 0.25:
            #    print "theta_error_old: ", theta_erro_old
            #    print "theta_error: ", theta_error
            theta_erro_old = theta_error

            #PID Action
            theta_control = ( kp_theta * theta_error + ki_theta * theta_integral + kd_theta * theta_deriv)
            
            # if debug or abs(theta_control) > 0.25:
            #     print "theta_control (A): ", theta_control
            #     print "Integral: ", theta_integral
            #     print "Deriv:", theta_deriv
            #     print "dt: ",dt
            
            #---------------------------------------------------------------------------
            #   The maximum steering of the car is 20 degrees (0.34 radians)
            #---------------------------------------------------------------------------
            if (theta_control > MAX_STEER * ANG_TO_RAD):        #adjust the maximum control action
                theta_control = MAX_STEER * ANG_TO_RAD
            elif (theta_control < - MAX_STEER * ANG_TO_RAD):    #adjust the maximum control action
                theta_control = - MAX_STEER * ANG_TO_RAD
            # if debug or abs(theta_control) > 0.25:
            #     print "theta_control: ", theta_control
        else:                                                       #if theta_error is not significative
            if debug:
                print "ANGLE_IGUAL"
            theta_error = ZERO                                  #clear theta_error                                     
            theta_control = ZERO                                #clear theta control
            theta_integral = ZERO                               #clear theta integral
            theta_erro_old = ZERO                               #clear previous theta_error

        return theta_control
    else:                                                       #if theres not new values from the sensors controller variables
        return 0.0

#-----------------------------------------------------------------------
#FUNCAO: compare_lost_position():
#This function compares the SV position with the historical position from TV
#Uses the TV vector to find the most near position in front of SV, considering the current position of SV and
#historical positions from TV
#-----------------------------------------------------------------------
def compare_lost_position():
    global TV_position_vector                           #vector to store TV positions to be followed
    global longitude_SV                                 #SV Longitude
    global latitude_SV                                  #SV Latitude
    global heading_SV                                   #SV heading
    global mode
    
    dist = 0                                            #distance between the current SV position and TV vector
    dist_min = 999                                      #minimun distance value
    pos = 0                                             #position of the smallest distance in the vector

    dist_lost = 0                                       #if the position is not in front of SV, stores the current distance
    dist_lost_min = 999                                 #minimun distance value
    pos_lost = 0                                        #position of the smallest distance in the vector

    debug = OFF

    #------------------------------------------------------------------------
    #adjusting the angles given the rotation between the circle and the road (pi/2)
    #------------------------------------------------------------------------
    head_SV_adjusted = heading_SV - ANGLE_ADJUST
    
    #-------------------------------------------------------------------------
    #solving the problemas with diferent positions in the trigonometric circle:
    #ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
    #to solve this problem, if the radian is smaller then -45, we add 2pi, in order to turn this number positive
    # -------------------------------------------------------------------------  
    if (head_SV_adjusted < -math.pi/4):
        head_SV_adjusted = head_SV_adjusted + 2 * math.pi

    #-------------------------------------------------------------------------
    #   Compares the historical position from TV with current position of SV        
    #-------------------------------------------------------------------------    
    for i in range(TV_position_vector.shape[0]):            #for each position in TV_position_vector
        
        if debug:
            print "i: ", i
            print "TV_position_vector: ", TV_position_vector[i,1], TV_position_vector[i,0], TV_position_vector[i,2], TV_position_vector[i,3]
            print "SV:                 ", longitude_SV, latitude_SV, heading_SV, speed_SV
        
        d_lat = TV_position_vector[i,0] - latitude_SV               #calculates the distance in latitude (x)
        d_long = TV_position_vector[i,1] - longitude_SV             #calculates the distance in longitude (y)
        theta_SV_TV = math.atan2(d_long, d_lat)                     #calculate the angle between the current SV positin and the evaluated position of TV

        #-------------------------------------------------------------------------
        #solving the problemas with diferent positions in the trigonometric circle:
        #ROS considers the circle as 2 halfs: 0 to 3.1415 radians and 0 to -3.1414
        #to solve this problem, if the radian is smaller then -45, we add 2pi, in order to turn this number positive
        # -------------------------------------------------------------------------  
        if (theta_SV_TV < 0.0 and head_SV_adjusted > math.pi/4):
            theta_SV_TV = theta_SV_TV + 2 * math.pi
        
        diff_theta = head_SV_adjusted - theta_SV_TV     #calculates the diff between the current SV angle and the angle between TV and SV

        if (abs(diff_theta) < ANGLE_FRONT):             #if diff is to big, the TV position is not in front of SV
            dist = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if debug:
                print "VALOR A FRENTE"
        else:                                           #if the evaluated TV point is not in front of SV, calculate the distance to use in a "lost mode"
            if debug:
                print "VALOR NAO A FRENTE"
            dist_lost = ((TV_position_vector[i,1] - longitude_SV)**2 + (TV_position_vector[i,0] - latitude_SV)**2)**0.5
            if dist_lost_min > dist_lost and dist_lost > 0.0:
                pos_lost = i
                dist_lost_min = dist_lost

        if debug:
            print "Dist: ", dist
        #--------------------------------------------------------------------------
        #If dist is smaller then the previous saved value in dist_min, update dist_min
        #However, if dist min is smaller then current dist, breaks the repetition, keeping this position for the tests
        #If there are no positions in the vector that are in front of SV, uses the small distance between SV and the full vector
        #--------------------------------------------------------------------------    
        if dist_min > dist and dist > 0.0:
            pos = i
            dist_min = dist             #else, update dist_min value
            mode = NORMAL
        elif dist_min < dist:
            mode = NORMAL
            break
        elif (i == (TV_position_vector.shape[0] - 1)):
            pos = pos_lost
            mode = LOST

    
    #print "TV:          ", TV_position_vector[pos,0], TV_position_vector[pos,1] 
    #print "Current SV:  ", latitude_SV, longitude_SV

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

    global publication_topic
    global erro_vel
    global TV_position_vector                               #vector to store TV positions to be followed
    global mode

    debug = ON

    pub = rospy.Publisher(publication_topic, Control, queue_size=10)				#topico de controle do carro
    msg = Control()											                        #Define msg como um dado do tipo Control -> comando de controle do veiculo
    msg.header.stamp = rospy.Time.now();					                        #tempo atualizado da leitura
    msg.header.frame_id = "base_link";						
    msg.steer = 0 * ANG_TO_RAD						                                #angulacao desejada para as rodas (0-20) GRAUS

    #print "------------------------------------------------------"
    #calcula distancia
    distance = ((longitude_TV - longitude_SV)**2 + (latitude_TV - latitude_SV)**2)**0.5                                 #real distance between TV and SV
    maximum_distance = MINIMUM_DISTANCE + speed_TV * SAFETY_TIME                                                        #desisred distance
    distance_error = distance - maximum_distance                                                                        #error between desired ditance and real distance

    #if debug:
        #print "TV 1: ", longitude_TV, latitude_TV, heading_TV, speed_TV
        #print "SV: ", longitude_SV, latitude_SV, heading_SV, speed_SV
        #print "Distance", distance
        #print "maximum_distance: ", maximum_distance
        #print "Distance Error: ", distance_error
    if (TV_position_vector[0,0]<=0.1 and TV_position_vector[0,1]<=0.1):                                                 #cleaning the first position of the vector
        TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV, speed_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
        TV_position_vector = TV_position_vector[1:,:]                                                                   #delete the first line with empty values
    else:
        TV_position_vector = np.append(TV_position_vector,[[latitude_TV, longitude_TV, heading_TV, speed_TV]],axis=0)   #Numpy vector que armazena posicoes de TV
        
    #if debug:
        #print "TV_position_vector shape: ", TV_position_vector.shape
        #print "Speed_TV: ", speed_TV
        #print TV_position_vector
    
    #ajusta velocidade e aceleracao
    if (speed_TV >= EDGE_LOW_SPEED and distance >= MINIMUM_DISTANCE):               #TV is moving and the distance is significative
        #if debug:
        #    print "TV Moving!"
        PID_dist = distance_control (distance_error)                                                #determines de control action for the distance
        lat_compare, long_compare, heading_compare, speed_compare = compare_lost_position()         #compares the SV_position with the vector of positions
        new_speed_SV = speed_compare + PID_dist                                                     #defines the new speed for the SV
        msg.throttle, msg.brake = calc_throttle_brake(new_speed_SV,speed_SV)                        #determine throttle and brake
        msg.steer = direction_control (lat_compare, long_compare, heading_compare, heading_SV)
    else:
        msg.throttle, msg.brake = 0.0 , 0.0                        #determine throttle and brake
        msg.steer = 0.0
        # if debug:
        #     if speed_TV < EDGE_LOW_SPEED:
        #         print "Speed < EDGE_LOW_SPEED"
        #     elif distance >= MINIMUM_DISTANCE:
        #         print "Distance < MINIMUM_distance"
        #     else:
        #         print "ERROR!!!" 

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

    # global count_lap
    # global lap
    # if (abs(longitude_TV-180) < 1 and abs(latitude_TV-10) < 2):  # conta o numero de voltas
    #     if count_lap == 0:
    #         lap = lap +1
    #         count_lap = count_lap + 1
    #         print "Lap: ", lap
    #         print "TV_data: ", latitude_TV, longitude_TV, heading_TV, speed_TV
    #         print "SV_data: ", latitude_SV, longitude_SV, heading_SV, speed_SV
    # else:
    #     count_lap = 0

    vehicle_general_control()

def TV_info(data):

    global car_name_TV
    global longitude_TV
    global latitude_TV
    global heading_TV
    global speed_TV							                 #chamada para a variavel global
    global steering_TV                                       #steering (angulacao das rodas do veiculo em relacao ao veiculo)
    global throttle_TV                                       #percentual do acelerador (0 - 1)
    global brake_TV                                          #percentual do freio (0 - 1)

    global lap
    global count_lap

    global TV_position_vector                               #vector to store TV positions to be followed
    #print 'TV_INFO'
    if (car_name_TV == data.car_name):
        latitude_TV = data.latitude
        longitude_TV = data.longitude
        heading_TV = data.heading_headingValue
        speed_TV = data.speed_speedValue						# salva o valor da velocidade no instante
        steering_TV = data.steeringWheelAngle_steeringWheelAngleValue
        throttle_TV = data.gasPedalPercent_Value
        brake_TV = data.brakePedalPercent_Value
        print "TV 0: ", longitude_TV, latitude_TV, heading_TV, speed_TV, car_name_TV

    


def listener():
    global TV_topic
    global SV_topic
    global car_name_TV
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print "TV Topic: ", TV_topic
    print "SV Topic: ", SV_topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(TV_topic,CAM_simplified,TV_info)
    rospy.Subscriber(SV_topic,CAM_simplified,SV_info)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #global TV_topic
    #global SV_topic
    #global publication_topic
    param = sys.argv[1:]                                #le os dados passados como argumentos (TV SV)
                                                        #param[0] - TV
                                                        #param[1] - SV
    car_name_TV = "/" + param[0] +"/"
    #TODO - tratamento de erros para parametros de entrada
    #TODO - Tornar generico parametros de leitura
    #TV_topic = "/" + param[0] +"/carINFO"                   #cria a string do topico do TV a ser lido (INFO Padrao)
    TV_topic = "/" + param[1] +"/RXNetwork"                  #string to be readed using a simulation of communication
    #TV_topic = "/" + param[1] +"/RXNetwork_OMNET"              #string to be used with OMNET
    SV_topic = "/" + param[1] +"/carINFO"                   #cria a string do topico do SV a ser lido
    publication_topic = "/" + param[1] +"/prius"            #cria a string do topico do SV a ser publicado (controle)
    listener()
