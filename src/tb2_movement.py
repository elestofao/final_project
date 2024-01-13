#!/usr/bin/python
import rospy
import math
from time import sleep
import numpy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FollowLine:
    def __init__(self):
        self.dir_sub = rospy.Subscriber('/direction', Int32, self.dir_callback)
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd = Twist()
        self.vueltas = 0
        self.x_robot = 0.0
        self.y_robot = 0.0
        self.rate = rospy.Rate(10)
        self.cont = False

    def dir_callback(self,msg):
        direction = msg.data

        # Ir recto
        if(direction == 0):
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0
            #self.rate.sleep()
            #print("Going Straight")

        # Ir girando a la izquierda
        elif(direction == 1):
            self.cmd.linear.x = 0.25
            self.cmd.angular.z = 0.35  #positivo es izquierda
            #self.rate.sleep()
            #print("Turning Left")

        # Ir girando a la derecha
        elif(direction == 2):
            self.cmd.linear.x = 0.25
            self.cmd.angular.z = -0.35  #negativo es derecha
            #self.rate.sleep()
            #print("Turning Right")
        
        # Buscar linea
        elif(direction == 3):
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0.35
            #self.rate.sleep()
            #print("Searching...")
        
        else:
            print(direction)

        self.vel_pub.publish(self.cmd)

    def pose_callback(self, msg):
        x = round(msg.pose.pose.position.x,2)
        y = round(msg.pose.pose.position.y,2)
        nueva_vuelta = False

        if(x != self.x_robot):
            self.x_robot = x
            self.y_robot = y

        #print(x,y)
        #self.rate.sleep()
        if((self.x_robot>=0 and self.x_robot<=1) and (self.y_robot<=0.5 and self.y_robot>=-0.5) and not self.cont):
            self.vueltas += 1
            print("Vueltas dadas: ", self.vueltas)
            self.cont = True

        if self.x_robot > 1:
            self.cont = False


if __name__ == '__main__':
    rospy.init_node('tb2_motion')
    fl = FollowLine()
    rospy.spin()   