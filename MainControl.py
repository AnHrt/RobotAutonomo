#!/usr/bin/env python 
import rospy 
import cv2
import time as t
import numpy as np
from geometry_msgs.msg import Twist   
from std_msgs.msg import Float32, Bool, Int8, Int32

class Robot(): 
    #This class implements the differential drive model of the robot 
    def __init__(self): 
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 
        ############ Variables ############### 
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad] 

    def update_state(self, wr, wl, delta_t): 
        #This function returns the robot's state 
        #This functions receives the wheel speeds wr and wl in [rad/sec]  
        # and returns the robot's state 
        v=self.r*(wr+wl)/2 
        w=self.r*(wr-wl)/self.L 

        self.theta=self.theta + w*delta_t 
        #Crop theta_r from -pi to pi 
        self.theta=np.arctan2(np.sin(self.theta),np.cos(self.theta)) 

        vx=v*np.cos(self.theta) 
        vy=v*np.sin(self.theta) 

        self.x=self.x+vx*delta_t  
        self.y=self.y+vy*delta_t 

class ControlMain(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        self.robot=Robot()

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("center", Int32, self.center_cb) #Line following
        rospy.Subscriber("LightGreen", Bool, self.lightGreen_cb) #Traffic lights
        rospy.Subscriber("LightRed", Bool, self.lightRed_cb) #Traffic lights
        rospy.Subscriber("main_signal", Int8, self.signal_cb) #Traffic Signals

        self.x_target=0.0 #x position of the goal 
        self.y_target=0.0 #y position of the goal 
        #self.goal_received=0 #flag to indicate if the goal has been received 
        self.target_position_tolerance=0.07 

        ############################# CONSTANTES CALLBACK ############################

        self.vel_x = 0.0
        self.vel_z = 0.0
        self.center = 0.0
        
        self.received_center = False
        
        self.green =False
        self.red=False
        self.ed = 0.0
        
        self.vel = Twist()
        
        self.e_theta=0.0    #error 
        self.e_thetaAnterior=0.0
        self.e_thetaArray = []
        self.e_thetaIn = 0.0
    
        self.signal = 0
        self.received_signal = False
        main_signal = self.signal

        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 

        self.current_state='Stop'

        ############################# CONSTANTES ############################
 
        #~~~~~~~~~~~~~~~~~~~~~~~ INIT NODE ~~~~~~~~~~~~~~~~~~~~~~~
        freq=20 
        rate = rospy.Rate(freq)         #20Hz  
        Dt =1.0/float(freq)               #Dt is the time between one calculation and the next one 
        print("Node initialized " + str(freq) + " hz") 
        while not rospy.is_shutdown():
            self.robot.update_state(self.wr, self.wl, Dt)
            
            #print("Signal " + str(main_signal))
            print("Signal Recieve " + str(self.received_signal))
            print("Red", self.red)
            print("Green", self.green)
            """if self.red!=True:
                Velpred = 0.15
                if self.received_center:
                    velXpid, velZpid = self.pid_control(self.center, Velpred)
                    self.vel.linear.x = velXpid
                    self.vel.angular.z = velZpid
                    if (self.received_signal and main_signal != -1):
                        if main_signal == 0: #Go ahead
                            v_gtg, w_gtg = self.gtg_control(1.5, 0.0, self.robot.x, self.robot.y, self.robot.theta) 
                            self.vel.linear.x = v_gtg
                            self.vel.angular.z = w_gtg
                            if(self.ed<0.3):
                                main_signal=-1
                        elif main_signal == 1: #Turn right
                            v_gtg, w_gtg = self.gtg_control(1.5, 1.5, self.robot.x, self.robot.y, self.robot.theta) 
                            self.vel.linear.x = v_gtg
                            self.vel.angular.z = w_gtg
                        elif main_signal == 2: #No speed limit
                            Velpred += 0.05
                        elif main_signal == 3: #Stop
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = 0.0
                            break
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0"""

            if self.red!=True:
                Velpred = 0.15
                if self.received_center:
                    self.current_state='PID_Control'
                    if (self.received_signal and main_signal!=-1):
                        if main_signal == 0: #Go ahead
                            self.current_state='GO_Ahead'
                        elif main_signal == 1: #Turn right
                            self.current_state='Turn_Right'
                        elif main_signal == 2: #No speed limit
                            self.current_state='No_Speed_Limit'
                        elif main_signal == 3: #Stop
                            self.current_state='Stop'
                            break
                        else:
                            self.current_state='PID_Control'
            else:
                self.current_state='Stop'

            if self.current_state=='PID_Control':
                print("sigo linea rey")
                Velpred=Velpred
                velXpid, velZpid = self.pid_control(self.center, Velpred)
                self.vel.linear.x = velXpid
                self.vel.angular.z = velZpid

            elif self.current_state=='GO_Ahead':
                v_gtg, w_gtg = self.gtg_control(1.35, 0.001, self.robot.x, self.robot.y, self.robot.theta) 
                self.vel.linear.x = v_gtg
                self.vel.angular.z = w_gtg
                #self.signal = -1
                self.current_state='PID_Control'
            
            elif self.current_state=='Turn_Right':
                v_gtg, w_gtg = self.gtg_control(1.2, 1.2, 0.0, 0.0.y, self.robot.theta) 
                self.vel.linear.x = v_gtg
                self.vel.angular.z = w_gtg
                #self.signal = -1
                self.current_state='PID_Control'

            elif self.current_state=='No_Speed_Limit':
                Velpred += 0.02
                #self.signal = -1
                self.current_state='PID_Control'

            elif self.current_state=='Stop':
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                if(self.green == True):
                    self.red = False
                    self.current_state='PID_Control'

            if(self.vel.linear.x<0.030):
                self.signal = -1
                self.received_signal = False
                self.current_state='PID_Control'

            print(self.current_state)
            print(self.vel.linear.x)
            self.cmd_vel_pub.publish(self.vel) 
            rate.sleep()

    def gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 

        v=0.0 #Modify this line to change the robot's speed 
        w=0.0 #modify this line to change the robot's angular speed 

        #Calculate the error
        e_theta = np.arctan2(y_target - y_robot, x_target - x_robot) - theta_robot
        self.ed = np.sqrt(pow(x_target - x_robot, 2) + pow(y_target - y_robot, 2))

        #Gain
        Kv = 0.10
        Kw = 0.10

        vel_z = Kw * e_theta
        vel_x = Kv * self.ed

        #limits for the speed
        if (abs(vel_z) > 0.4):
            if (vel_z > 0):
                vel_z = 0.4
            else:
                vel_z = -0.4

        if (vel_x > 0.4):
            vel_x = 0.4

        #Control
        if abs(e_theta) >= 0.5:
            w = vel_z
            v = 0.0
        else:
            if abs(self.ed) <= 0.1:
                v = 0.0
                w = 0.0
            else:
                v = vel_x
                w = 0.0

        return v, w 

    def pid_control(self, center, vel):
        vel_x=0.2
        vel_z=0

        self.e_thetaAnterior = self.e_theta
        self.e_theta = 60 - center

        self.e_thetaArray.append(self.e_theta)
        self.e_thetaIn = sum(self.e_thetaArray)
        
        Kp = 0.000555
        Kd = 0.0#000005
        Ki = 0.0#00000006
        P = Kp * self.e_theta
        I = Ki*(self.e_thetaIn)
        D = Kd * (self.e_theta - self.e_thetaAnterior)
        print("P",P)
        print("I",I)
        print("D",D)
        #Left turn & Right turn
        if self.e_theta > 15:
            vel_z = P + I + D
            vel_x = 0.09

        elif self.e_theta < -15: 
            vel_z = -(P + I +D)
            vel_x = 0.09

        else:
            vel_z = 0

        if (abs(self.e_theta) > 33):
            vel_x = 0.05
        else:
            vel_x = vel

        """print('vel ang: ',self.vel.angular.z)
        print('vel lin: ',self.vel.linear.x)
        print('error theta', self.e_theta)"""

        

        return vel_x, vel_z
    
    def lightRed_cb(self, kp_r):
        self.red = kp_r.data
        #print("Red",self.red)

    def lightGreen_cb(self, kp_g):
        self.green = kp_g.data
        #print("Green",self.green)

    def center_cb(self, center): 
        self.center = center.data                 #Obtiene el punto detectado
        #print("Centro",self.center)
        self.received_center = True

    def signal_cb(self, signal):
        self.signal = signal.data                 #Obtiene el punto detectado
        self.received_signal = True
        
    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data  

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  
    
    def cleanup(self): 
    #Funcion para termino del nodo / detiene el robot 
    #Modifica velocidades a 0 y las publica
        cleanup = Twist()
        self.cmd_vel_pub.publish(cleanup)            
        print("\nBye bye :)")

#~~~~~~~~~~~~~~~~~~~~~~~ MAIN PROGRAM ~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == "__main__":
    rospy.init_node("MainControl", anonymous=True) 
    ControlMain()