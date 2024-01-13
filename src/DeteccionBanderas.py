#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String


UMBRAL_PIXELS = 8000

class FlagDetection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.pub = rospy.Publisher('/flag_color', String, queue_size=10)
        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)
        self.rcont = False
        self.ycont = False
        self.gcont = False

    def image_callback(self, msg):
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        red_flag = False
        green_flag = False
        yellow_flag = False

        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        lower_green = np.array([50, 50, 50])
        upper_green = np.array([65, 255, 255])

        lower_yellow = np.array([25, 50, 50])
        upper_yellow = np.array([35, 255, 255])

        rmask = cv2.inRange(hsv, lower_red, upper_red)
        gmask = cv2.inRange(hsv, lower_green, upper_green)
        ymask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        rpixels = cv2.countNonZero(rmask)
        gpixels = cv2.countNonZero(gmask)
        ypixels = cv2.countNonZero(ymask)

        if rpixels > UMBRAL_PIXELS:
            red_flag = True
        if gpixels > UMBRAL_PIXELS:
            green_flag = True
        if ypixels > UMBRAL_PIXELS:
            yellow_flag = True


        if green_flag and not yellow_flag and not red_flag and not self.gcont:
            #print("Verde")
            self.pub.publish('Bandera verde')
            self.gcont = True
        if yellow_flag and not red_flag and not self.ycont:
            #print("Amarillo")
            self.pub.publish('Bandera amarilla')
            self.ycont = True
        if red_flag and not self.rcont:
            #print("Rojo")
            self.pub.publish('Bandera roja')
            self.rcont = True

        
        if not red_flag:
            self.rcont = False
        if not yellow_flag:
            self.ycont = False
        if not green_flag:
            self.gcont = False
        
        #en caso de querer que publique todo el rato el color de la bandera y no solo
        #una vez, eliminar todo lo relativo a los self.Xcont

if __name__ == '__main__':
    rospy.init_node('flag_color')
    cd = FlagDetection()
    rospy.spin()