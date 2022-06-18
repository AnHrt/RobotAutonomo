#!/usr/bin/env python 
import cv2 
import rospy 
import numpy as np

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 

class FollowLines(): 
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        #rospy.init_node("Follow_lines")   
        #~~~~~~~~~~~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~~~~~~~~~~~
        #wheel separation [m] 
        self.wr=0.0 
        self.wl=0.0
    
        #Image size
        self.width = 426
        self.height = 240

        self.bridge_object = CvBridge() # create the cv_bridge object
        self.image_received = 0         #Flag to indicate that we have already received an image

        #~~~~~~~~~~~~~~~~~~~~~~~ SUBSCRIBERS & PUBLISHER ~~~~~~~~~~~~~~~~~~~~~~~
        self.img_pub = rospy.Publisher("imagen", Image, queue_size=1)
        self.pub_control = rospy.Publisher('center', Int32, queue_size=1)

        self.center=60
        self.image_sub = rospy.Subscriber("video_source/raw", Image, self.image_cb)
        #~~~~~~~~~~~~~~~~~~~~~~~ INIT NODE ~~~~~~~~~~~~~~~~~~~~~~~
        freq=20 
        rate = rospy.Rate(freq)         #20Hz  
        Dt =1/float(freq)               #Dt is the time between one calculation and the next one 
        print("Node initialized 20hz") 


        while not rospy.is_shutdown():
            
            if self.image_received:
                print("Sigo linea")
                imagen = cv2.resize(self.cv_image,(self.width,self.height))

                grayFrame = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)  #GrayScale
                gaussian = cv2.GaussianBlur(grayFrame,(15,15),0)     #Blur

                #Binary transformation Image
                (thresh, blackAndWhiteFrame) = cv2.threshold(gaussian, 90, 255, cv2.THRESH_BINARY)
                self.cropbw = blackAndWhiteFrame[180:326, 120:240]

                x = np.sum(self.cropbw, axis=0)/15300 #Normalized values
                print(x)
                i = 0
                
                pointL = 0
                pointR = 0

                for i in range(len(x)):
                    if x[i] == 0 and x[i-1] == 1:
                        pointL = i
                    elif x[i] == 1 and x[i-1] == 0:
                        pointR = i-1
                
                if pointL > pointR:
                    pointL = 0
                    pointR = 0
                self.center = pointL + ((pointR-pointL)/2)
                

                print("Primer punto linea: ",pointL)
                print("Ultimo punto linea: ",pointR)
                print("Centro de la linea: ",self.center)
                print("Error", )

                self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(self.cropbw, "passthrough"))
                self.pub_control.publish(self.center)
                cv2.waitKey(1)
                self.image_received = 0
                

            rate.sleep() 
        
        cv2.destroyAllWindows()

    def image_cb(self, ros_image): 
            ## This function receives a ROS image and transforms it into opencv format  
        if not self.image_received:
            try:
                print("received ROS image, I will convert it to opencv")
                # We select bgr8 because it is the OpenCV encoding by default
                self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
                #self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="rgb8")
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
    rospy.init_node("Follow_lines", anonymous=True) 
    FollowLines() 