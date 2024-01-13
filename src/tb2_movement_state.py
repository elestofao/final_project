#!/usr/bin/python
import rospy
import math
import time
from time import sleep
import random
import numpy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import smach_ros
from smach import State,StateMachine
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Variables globales
n_laps = 0                  # Numero de vueltas que lleva el robot
finish_line_pass = False    # Para saber si el robot pasa por meta 
finish_race = False         # Para saber si ha terminado la carrera
goal_laps = 5               # Numero de vueltas que tiene que completar el robot
cont = 0

# Nos fijamos en el laser que esta a 90 grados (el de la izquierda del robot) para detectar la meta
ANG_SCAN = 90*math.pi/180.0
# Para detectar el box, nos fijamos en los laseres a -75 y -105 grados (derecha)
ANG_SCAN_TOP = -80*math.pi/180.0
ANG_SCAN_BOTTOM = -100*math.pi/180.0


# Funciones auxiliares para suscribirnos/desuscrbirnos de los topics
def subcribe(self):
    self.dir_sub = rospy.Subscriber('/direction', Int32, self.dir_callback)
    #self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
    self.color_sub = rospy.Subscriber('/flag_color', String, self.color_callback)
    self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)

def unsubscribe(self):
    self.dir_sub.unregister()           # Desuscripcion al topic de la direccion
    #self.pose_sub.unregister()          # Desuscripcion al topic de la posicion del robot
    self.color_sub.unregister()         # Desuscripcion al topic del color
    self.scan_sub.unregister()          # Desuscripcion al topic del laser


# Funcion auxiliar que se encarga de mover el robot con una velocidad u otra segun el estado que la llame
def motion(direction,x_max,x_turn,z):
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    cmd = Twist()
    # Ir recto
    if(direction == 0):
        cmd.linear.x = x_max
        cmd.angular.z = 0
        #print("Going Straight")

    # Ir girando a la izquierda
    elif(direction == 1):
        cmd.linear.x = x_turn
        cmd.angular.z = z  #positivo es izquierda
        #print("Turning Left")

    # Ir girando a la derecha
    elif(direction == 2):
        cmd.linear.x = x_turn
        cmd.angular.z = -z  #negativo es derecha
        #print("Turning Right")

    # Ir girando a la izquierda muy lento
    elif(direction == 3):
        cmd.linear.x = x_turn-0.1
        cmd.angular.z = z  #positivo es izquierda
        #print("Turning Left Slowly")

    # Ir girando a la derecha muy lento
    elif(direction == 4):
        cmd.linear.x = x_turn-0.1
        cmd.angular.z = -z  #negativo es derecha
        #print("Turning Right Slowly")
    
    # Buscar linea
    elif(direction == 5):
        cmd.linear.x = 0
        cmd.angular.z = z
        #print("Searching...")
    
    else:
        print(direction)

    vel_pub.publish(cmd)


# Funcion auxiliar para contar las vueltas que lleva el robot
def finish_line(msg):
    global finish_line_pass, n_laps, finish_race, cont
    POS_SCAN = int((ANG_SCAN-msg.angle_min)/msg.angle_increment)
    laps_scan = msg.ranges[POS_SCAN]
    
    if laps_scan<0.5 and laps_scan!=cont:
        finish_line_pass = True
        cont = laps_scan
    elif laps_scan>0.5:
        cont = laps_scan
        
    if finish_line_pass:
        n_laps += 1
        print("Lap: ", n_laps)
        finish_line_pass = False

    # Si se cumplen las vueltas objetivo, se indica que la carrera debe terminar
    if n_laps >= goal_laps:
        finish_race = True


# Estado si la bandera es verde
class GreenFlag(State):
    def __init__(self):
        State.__init__(self, outcomes=['yellow_detected', 'red_detected', 'finish','aborted'])
        #subcribe(self)
    
    def execute(self, userdata):
        subcribe(self)

        self.yellow_detected = False    # flag para saber si se ha detectado bandera amarilla
        self.red_detected = False     # flag para saber si se ha detectado bandera roja
        print('Green Flag! Driving at full speed')
        rate = rospy.Rate(10)

        # Mientras no se detecte otra bandera ni acabe la carrera...
        while (not(self.yellow_detected) and not(finish_race) and not(self.red_detected)):
            rate.sleep()

        # Nos desuscribimos de los topics
        unsubscribe(self)

        # Si el while se ha interrumpido por bandera amarilla, pasaremos al estado amarillo
        if self.yellow_detected:
            return 'yellow_detected'
        # Si el while se ha interrumpido por bandera roja, pasaremos al estado rojo
        elif self.red_detected:
            return 'red_detected'
        # Si el while se ha interrumpido por finalizacion de la carrera, terminamos la maquina de estados
        elif finish_race:
            return 'finish'
        # Si el while se ha interrumpido por otra cosa, se aborta la maquina de estados (esto nunca pasara pero por si acaso)
        else:
            return 'aborted'


    def dir_callback(self,msg):
        # x_max = 0.5, x_turn = 0.25, z = 0.35
        motion(msg.data,0.5,0.25,0.35)
    
    def scan_callback(self,msg):
        finish_line(msg)

    def color_callback(self,msg):
        if msg.data == "Bandera amarilla":
            self.yellow_detected = True
        elif msg.data == "Bandera roja":
            self.red_detected = True


# Estado si la bandera es amarilla
class YellowFlag(State):
    def __init__(self):
        State.__init__(self, outcomes=['green_detected', 'red_detected', 'finish','aborted'])
        #subcribe(self)
    
    def execute(self, userdata):
        subcribe(self)

        self.green_detected = False     # flag para saber si se ha detectado bandera verde
        self.red_detected = False     # flag para saber si se ha detectado bandera roja
        print('Yellow Flag! Driving at limited speed')
        rate = rospy.Rate(10)

        # Mientras no se detecte otra bandera ni acabe la carrera...
        while (not(self.green_detected) and not(finish_race) and not(self.red_detected)):
            rate.sleep()

        # Nos desuscribimos de los topics
        unsubscribe(self)

        # Si el while se ha interrumpido por bandera verde, pasaremos al estado verde
        if self.green_detected:
            return 'green_detected'
        # Si el while se ha interrumpido por bandera roja, pasaremos al estado rojo
        elif self.red_detected:
            return 'red_detected'
        # Si el while se ha interrumpido por finalizacion de la carrera, terminamos la maquina de estados
        elif finish_race:
            return 'finish'
        else:
            return 'aborted'


    def dir_callback(self,msg):
        # x_max = 0.35, x_turn = 0.15, z = 0.25
        motion(msg.data,0.35,0.15,0.25)
    
    def scan_callback(self,msg):
        finish_line(msg)

    def color_callback(self,msg):
        if msg.data == "Bandera verde":
            self.green_detected = True
        elif msg.data == "Bandera roja":
            self.red_detected = True


# Estado si la bandera es roja
class RedFlag(State):
    def __init__(self):
        State.__init__(self, outcomes=['green_detected', 'yellow_detected', 'finish','aborted'])
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #subcribe(self)
    
    def execute(self, userdata):
        subcribe(self)

        self.green_detected = False     # flag para saber si se ha detectado bandera verde
        self.yellow_detected = False     # flag para saber si se ha detectado bandera amarilla
        self.stopMotion = False
        print('Red Flag! Red Flag! Returning to boxes')
        rate = rospy.Rate(10)

        # Mientras no se detecte otra bandera ni acabe la carrera...
        while (not(self.green_detected) and not(finish_race) and not(self.yellow_detected)):
            rate.sleep()

        # Nos desuscribimos de los topics
        unsubscribe(self)

        # Si el while se ha interrumpido por bandera verde, pasaremos al estado verde
        if self.green_detected:
            return 'green_detected'
        # Si el while se ha interrumpido por bandera amarilla, pasaremos al estado amarilla
        if self.yellow_detected:
            return 'yellow_detected'
        # Si el while se ha interrumpido por finalizacion de la carrera, terminamos la maquina de estados
        elif finish_race:
            return 'finish'
        else:
            return 'aborted'


    def dir_callback(self,msg):
        # x_max = 0.35, x_turn = 0.15, z = 0.25
        if not self.stopMotion:
            motion(msg.data,0.35,0.15,0.25)
        else:
            motion(msg.data, 0,0,0)

    def scan_callback(self,msg):
        TOP_SCAN = int((ANG_SCAN_TOP-msg.angle_min)/msg.angle_increment)
        BOTTOM_SCAN = int((ANG_SCAN_BOTTOM-msg.angle_min)/msg.angle_increment)
        box_scan_top = round(msg.ranges[TOP_SCAN],2)
        box_scan_bottom = round(msg.ranges[BOTTOM_SCAN],2)

        if box_scan_top>=0.8 and box_scan_top<=1.1 and box_scan_bottom>=0.8 and box_scan_bottom<=1.1 :
            self.stopMotion = True

    def color_callback(self,msg):
        if msg.data == "Bandera verde":
            self.green_detected = True
        if msg.data == "Bandera amarilla":
            self.yellow_detected = True


# Estado para empezar la carrera
class RaceStart(State):
    def __init__(self):
        State.__init__(self, outcomes=['green_flag', 'yellow_flag','aborted'])
    
    def execute(self, userdata):
        self.color_pub = rospy.Publisher('/flag_color', String, queue_size=10)
        self.go = False         # flag para saber comienzo de carrera
        self.countdown = 5     # cuenta atras de 5 segundos (cambiar si se quiere a 10s)
        print('Starting race...')
        rate = rospy.Rate(10)

        # Mientras no empiece la carrera
        while (not(self.go)):
            # Cuenta atras
            while self.countdown > 0:
                print(self.countdown)
                time.sleep(1)           # Esperar 1 segundo
                self.countdown -= 1     # Reducir cuenta atras            
            
            self.go = True
            print('Go!')

        if self.go:
            # Generar un numero aleatorio entre 0 y 1
            random_number = random.random()
            # Probabilidad de que la bandera sea verde
            green_odds = 0.95
            
            if random_number < green_odds:
                self.color_pub.publish('Bandera verde')
                return 'green_flag'
            else:
                self.color_pub.publish('Bandera amarilla')
                return 'yellow_flag'
        else:
            return 'aborted'



if __name__ == '__main__':
    rospy.init_node('tb2_motion')
    # Se crea la maquina de estados
    sm = StateMachine(outcomes=['end'])
    with sm:
        # Estado para empezar la carrera
        StateMachine.add('RaceStart', RaceStart(), transitions={'green_flag':'GreenFlag', 'yellow_flag':'YellowFlag', 'aborted':'end'})
        # Estado para correr bajo bandera verde
        StateMachine.add('GreenFlag', GreenFlag(), transitions={'yellow_detected':'YellowFlag', 'red_detected':'RedFlag', 'finish':'end', 'aborted':'end'})
        # Estado para correr bajo bandera amarilla
        StateMachine.add('YellowFlag', YellowFlag(), transitions={'green_detected':'GreenFlag', 'red_detected':'RedFlag', 'finish':'end', 'aborted':'end'})
        # Estado para la bandera roja
        StateMachine.add('RedFlag', RedFlag(), transitions={'green_detected':'GreenFlag', 'yellow_detected':'YellowFlag', 'finish':'end', 'aborted':'end'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()

    rospy.spin()   