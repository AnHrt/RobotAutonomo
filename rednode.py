#!/usr/bin/env python3
import string
import cv2 
import rospy 
import numpy as np
from sensor_msgs.msg import Image
import time as t
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from statistics import mode


model = tf.keras.models.load_model('traffic_classifier_30.h5')

class rednn(): 
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        #Image size
        self.width = 640
        self.height = 480
        self.cvy = 0
        self.centro = 0
        self.minrec = 0
        self.maxrec = 0
        self.k = 0
        s = Int8()
        self.moda = []

        self.bridge_object = CvBridge() # create the cv_bridge object
        self.image_received = 0         #Flag to indicate that we have already received an image

        #~~~~~~~~~~~~~~~~~~~~~~~ SUBSCRIBERS & PUBLISHER ~~~~~~~~~~~~~~~~~~~~~~~
        #self.pub_img2 = rospy.Publisher("imagen2", Image, queue_size=1)
        self.pub_signal = rospy.Publisher("main_signal", Int8, queue_size=1)
        self.image_sub = rospy.Subscriber("video_source/raw", Image, self.image_cb)
        #~~~~~~~~~~~~~~~~~~~~~~~ INIT NODE ~~~~~~~~~~~~~~~~~~~~~~~
        freq=20
        rate = rospy.Rate(freq)         #20Hz  
        print("Node initialized " + str(freq) + " hz") 
        while not rospy.is_shutdown():
            
            if self.image_received:
                imagen = self.cv_image.copy()
                imagen = cv2.resize(imagen,(self.width,self.height))

                imgGray = cv2.cvtColor(imagen,cv2.COLOR_BGR2GRAY)
                imgGray = cv2.medianBlur(imgGray, 5)
                rows = imgGray.shape[0]

                circles = cv2.HoughCircles(imgGray,cv2.HOUGH_GRADIENT,1,rows/8, 
                         		param1 = 25, param2 = 29, 
                         		minRadius = 26, maxRadius = 50)

                if circles is not None:		#Condicional - si detecta circulos
                    circles = np.uint16(np.around(circles))
                    for i in circles[0, :]:
                        cy = i[0]
                        cyv = cy
                        for j in circles[0, :]:
                            cx = j[1]
                            self.center = (cx, cyv) 	
                            # centro del circulo
                            #cv.circle(framet, center, 1, (0, 100, 100), 3)
                            # esquema del circulo
                            radius = j[2]
                            #cv.circle(framet, center, radius, (255, 0, 255), 3)
                            #self.minrec = (cx - radius, cyv - radius)
                            #self.maxrec = (cx + radius, cyv + radius)
                            #rect = cv.rectangle(framet, minrec, maxrec, (255, 0, 255), 3)
                            startX = cx - radius
                            endX = cx + radius
                            startY = cy - radius
                            endY = cy + radius
                            if startX >= 0 and startY >= 0 and endX < self.width and endY < self.height:
                                frame = imagen[cx-radius:cx+radius,cyv-radius:cyv+radius]

                                frameNN = cv2.resize(frame,(30,30))

                                tests = []
                                tests.append(np.array(frameNN))
                                X = np.array(tests)
                                pred = np.argmax(model.predict(X))
                                self.k = 0
                                self.k = self.k +1
                                self.moda.append(pred)
                                # if(len(self.moda)%7 == 0):
                                #     moda = mode(self.moda)
                                s = Int8(pred)
                                print("El numero de la senal de transito es: ", s)
                                self.pub_signal.publish(s)
                                print("ya publique")
                            else:
                                s.data = -1
                                self.pub_signal.publish(s)
                                    #self.pub_img2.publish(self.bridge_object.cv2_to_imgmsg(frame, "passthrough"))
                else:
                    s.data = -1
                    self.pub_signal.publish(s)

                self.image_received = 0     
                rate.sleep()
                
        
        # cv2.destroyAllWindows()

    def image_cb(self, ros_image): 
        ## This function receives a ROS image and transforms it into opencv format  
        if not self.image_received:
            try:
                print("received ROS image, I will convert it to opencv")
                # We select bgr8 because it is the OpenCV encoding by defaul
                self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
                self.image_received = 1 #Turn the flag on
            except CvBridgeError as e:
                print(e)
    
    
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.
        self.image_received = 0   
        #self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(self.cropbw, "passthrough"))
        pass        
        
#~~~~~~~~~~~~~~~~~~~~~~~ MAIN PROGRAM ~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == "__main__": 
    rospy.init_node("rednode", anonymous=True) 
    rednn() 