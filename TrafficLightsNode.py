#!/usr/bin/env python 
import cv2
import rospy 
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#This class will receive a ROS image and transform it to opencv format 
class detector():
	def __init__(self):

		self.bridge_object = CvBridge() # create the cv_bridge object
		self.image_received = 0 #Flag to indicate that we have already received an image
		self.img_pub = rospy.Publisher("imagen2", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("video_source/raw", Image, self.image_cb)
		self.green_pub = rospy.Publisher("LightGreen", Bool, queue_size=1)
		self.red_pub = rospy.Publisher("LightRed", Bool, queue_size=1)
		self.green = False
		self.red = False
		
       #********** INIT NODE **********###
		r = rospy.Rate(10) #10Hz 
		while not rospy.is_shutdown():

			if self.image_received:
				imagen = self.cv_image.copy()
				imagen = cv2.resize(imagen,(450,300))
				hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
				#Centro
				min_green1 = np.array([80,4,189])
				max_green1 = np.array([93,255,255])
				#Izquierda
				min_green2 = np.array([83,46,131])
				max_green2 = np.array([100,238,255])

				#izquierda
				min_red1 = np.array([156,18,139])
				max_red1 = np.array([175,140,255])
				#Centro
				min_red2 = np.array([0,75,168])
				max_red2 = np.array([179,122,248])

				mask_g1 = cv2.inRange(hsv, min_green1, max_green1)
				mask_r1 = cv2.inRange(hsv, min_red1, max_red1)
				mask_g2 = cv2.inRange(hsv, min_green2, max_green2)
				mask_r2 = cv2.inRange(hsv, min_red2, max_red2)

				mask_g = mask_g1 + mask_g2
				mask_r = mask_r1 + mask_r2

				kernel = np.ones((5,5),np.uint8)
				ret, thresh1 = cv2.threshold(mask_r,220,255,cv2.THRESH_BINARY_INV) 
				ret, thresh2 = cv2.threshold(mask_g,220,255,cv2.THRESH_BINARY_INV) 
				thresh1 = cv2.morphologyEx(thresh1,cv2.MORPH_OPEN,kernel)
				thresh2 = cv2.morphologyEx(thresh2,cv2.MORPH_OPEN,kernel)
				thresh1 = thresh1[0:150,0:300]
				thresh2 = thresh2[0:150,0:300]

				
				params = cv2.SimpleBlobDetector_Params()
				params.filterByArea = True
				params.minArea = 50
				#params.maxArea = 100
				params.filterByCircularity = False
				params.filterByConvexity = False
				params.filterByInertia = True
				params.minInertiaRatio = 0.1

				detector = cv2.SimpleBlobDetector_create(params)
				
				self.kp_g = detector.detect(thresh2)
				self.kp_r = detector.detect(thresh1)

				blank = np.zeros((1,1))

				self.im_w_kpr = cv2.drawKeypoints(thresh1, self.kp_r, blank, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
				self.im_w_kpg = cv2.drawKeypoints(thresh2, self.kp_g, blank, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
				
				for keypoints in self.kp_g:
					print("Green",keypoints.size)
					if (keypoints.size >= 13 and keypoints.size < 15.5) or (keypoints.size >= 22 and keypoints.size < 23.5):
						self.green = True
						self.red = False
						self.green_pub.publish(self.green)
					else:
						self.green = False

				print("Green",self.green)
				
				
				for keypoints in self.kp_r:
					print("Red",keypoints.size)
					if (keypoints.size > 12 and keypoints.size < 17):
						self.red = True
						self.green = False
						self.red_pub.publish(self.red)
					else:
						self.red = False
				print("Red",self.red)
				
				
				self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(self.im_w_kpg, "rgb8"))
				self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(self.im_w_kpr, "bgr8"))
				#img_final=thresh1+thresh2
				#self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(img_final, "mono8"))
				self.image_received = 0
				
				r.sleep() 
				#cv2.destroyAllWindows() 
	
	def image_cb(self, ros_image): 
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
		self.image_received = 0
		self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(self.im_w_kpg, "rgb8"))
		self.img_pub.publish(self.bridge_object.cv2_to_imgmsg(self.im_w_kpr, "bgr8"))
        
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("Lights", anonymous=True) 
    detector()