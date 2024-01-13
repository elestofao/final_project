#!/usr/bin/python
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32

class LineDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.dir_pub = rospy.Publisher('/direction', Int32, queue_size=5)
        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)
        self.flag_sub = rospy.Subscriber('/flag_color', String, self.flag_callback)
        self.color = 'Starting race...'
        self.bgr = (0,0,0)
    
    # Color de la bandera
    def flag_callback(self, msg):
        if msg.data == "Bandera verde":
            self.color = 'GREEN FLAG'
            self.bgr = (25,200,40)
        if msg.data == "Bandera amarilla":
            self.color = 'YELLOW FLAG'
            self.bgr = (14,213,241)
        elif msg.data == "Bandera roja":
            self.color = 'RED FLAG'
            self.bgr = (0,0,240)

    # Callback de la camara del tb2
    def image_callback(self, msg):      
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        c_x = None  # centroide
        
        # Aplicar filtro Gaussiano
        cv_image_blur = cv2.GaussianBlur(cv_image,ksize=(3,3), sigmaX=0.1, sigmaY=0.1)

        blur_hsv = cv2.cvtColor(cv_image_blur, cv2.COLOR_BGR2HSV)

        h, w, _ = cv_image.shape    # Alto y ancho de la imagen

        # Detectar todos los objetos dentro del rango HSV (supuestamente sera la linea)
        lower_black = np.array([0,00,0])
        upper_black = np.array([180,255,40])

        mask = cv2.inRange(blur_hsv, lower_black, upper_black)

        search_y = int(h*0.6)
        mask[:search_y, ] = 0

        try:
            # Calcular momentos y calcular centroide
            moments = cv2.moments(mask)
            m10 = moments['m10']
            m00 = moments['m00']
            m01 = moments['m01']
            c_x = int(m10/m00)

            # Encontrar contornos en la mascara
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2:]

            # Coger contorno mas grande (area maxima)
            rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(max(contours, key=np.size))

            # Dibujar rectangulo sobre la linea (ver github)
            cv2.rectangle(cv_image, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (0, 0, 255), 3)
            cv2.putText(cv_image, 'Detected line', (rect_x - 2, rect_y - 8), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255))

            # Dibujar centroide 
            if (m00 > 0):
                i_m10=int(m10)
                i_m00=int(m00)
                i_m01=int(m01)
                
                p1 = (int(i_m10/i_m00), int(i_m01/i_m00))
                cv2.circle(cv_image, p1, 5, (155, 200, 0), -1)

            # Informacion sobre el color de la bandera en la camara del robot
            cv2.putText(cv_image, self.color, (20, 20), cv2.FONT_HERSHEY_COMPLEX, .7, self.bgr)

        except ZeroDivisionError:
            # Si no se detecta la linea...
            cv2.putText(cv_image, '[WARNING] No line found', (int(w/4), 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 0, 255))
            self.dir_pub.publish(5)

        # Pixeles de tolerancia
        tol = 45    # tolerancia

        # Dependiendo de donde este el centroide:
        # Si el centroide esta a la izquierda del centro de la imagen - tolerancia: ir girando a la izquierda (dir = 1)
        # Si el centroide esta a la derecha del centro de la imagen + tolerancia: ir girando a la derecha (dir = 2)
        # Si el centroide esta cerca del centro de la imagen: ir recto (dir = 0)

        if (c_x != None):
            if(c_x < w/2-tol and c_x > w/5):
                self.dir_pub.publish(1)  # izq
            elif(c_x > w/2+tol and c_x < 4*(w/5)):
                self.dir_pub.publish(2)  # dcha
            elif(c_x < w/5):
                self.dir_pub.publish(3)  # izq cerrada
            elif(c_x > 4*(w/5)):
                self.dir_pub.publish(4)  # dcha cerrada
            else:
                self.dir_pub.publish(0)  # centro
        
        
        cv2.imshow("Turtlebot View", cv_image)
        #cv2.imshow("Line Mask", mask)
        cv2.waitKey(3)


rospy.init_node('line_detector')
ld  = LineDetector()
rospy.spin()   