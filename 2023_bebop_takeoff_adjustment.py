#!/usr/bin/env pythonimport cv2
import sys
import numpy as np
import time
import math
import cv2, cv_bridge
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from pid import PID

class LineDetection :
    
    def __init__(self) :
        self.angle_vector = list(np.ones(10))
        self.norm_angle = 45
        self.counter = 0    
  
        self.error = []
        self.angle = []
        self.lower_hsv = np.array([80, 150, 100]) 
        self.upper_hsv = np.array([179, 255, 255]) 
          
        self.Pyaw = 0.01
        self.Iyaw = 0.001
        self.Dyaw = 0

        self.bridge = cv_bridge.CvBridge()        
        self.twist = Twist()       
        rospy.init_node('takeoff_adjustment', anonymous=True)        
        self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.callback)  
        self.camera_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)   
        self.pub_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)

        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.5, 0.5, -0.1, 0.1) 
        self.autonomous_flag = False
                 
   
    def get_angle(self, x_orig, y_orig, x_des, y_des):
        deltaY = y_des - y_orig
        deltaX = x_des - x_orig   
        return math.atan2(deltaY, deltaX)*180/math.pi    

    def closest_line(self):
        pass               
    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.lineDetect(cv_image)
            cv2.imshow("window", cv_image )        
        
            k = cv2.waitKey(1)  
            if k == ord('o'):
                print(1)
                self.autonomous_flag = not self.autonomous_flag
            if k == 27:  # close on ESC key
                cv2.destroyAllWindows()
                rospy.signal_shutdown('interrupt')      
        except CvBridgeError as e:
            print(e)

                    
    def lineDetect (self, cv_image):        
        h, w = cv_image.shape[:2]      
         
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)        
        mask = cv2.inRange( hsv , self.lower_hsv , self.upper_hsv)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=9)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=5)  
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)        
        contours,_ = cv2.findContours(mask, 1, 2) 
        
        if len(contours) != 0:
            biggest_contour = max(contours, key = cv2.contourArea)

            [vx,vy,x,y] = cv2.fitLine(biggest_contour, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((w-x)*vy/vx)+y)           
            cv2.line(cv_image,(w-1,righty),(0,lefty),(0,255,0),2)

            angle = self.get_angle(0, lefty, w, righty)  
            self.angle_vector.append(angle)
            self.norm_angle = sum(self.angle_vector)/len(self.angle_vector)
            self.angle_vector.pop(0)
            self.norm_angle = round(self.norm_angle, 3)

            if abs(self.norm_angle) <= 0.5:
                self.counter  = self.counter  + 1
                print('counter=', self.counter )
            if self.counter  > 10:
                cv2.destroyAllWindows()
                rospy.signal_shutdown("adjusted")
                exit("adjusted")   

            self.twist.linear.x = 0
            self.twist.angular.z = -self.pid_yaw.update(angle) 
            if self.autonomous_flag:
                self.pub_vel.publish(self.twist)

            print(self.norm_angle)           
            print('angle = ', angle)
            print(lefty, righty)
            print('                         ')                                  
            cv2.putText(cv_image, text='angle : ' +str( self.norm_angle), org=(10, 20), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='Left   : ' +str( lefty), org=(10, 40), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='Right : ' +str( righty), org=(10, 60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='flag  ' +str( self.autonomous_flag), org=(10, 80), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.65, color=(0, 0, 0),thickness=2)
            cv2.circle(cv_image, (w, righty), 10, (0,100,0), thickness=-1) 
            cv2.circle(cv_image, (0, lefty), 10, (0,100,0), thickness=-1) 
        else:
            self.counter =0
            print('cant see the line')
        
    def cam_down(self):
        cam = Twist()
        cam.angular.y = -80
        self.camera_pub.publish(cam)

def main():
    try:                   
        LD = LineDetection()   
        time.sleep(1)       
        LD.cam_down()
        time.sleep(3)                  
        rospy.spin()
    except (KeyboardInterrupt, EOFError):
        cv2.destroyAllWindows()
        rospy.signal_shutdown('keyboard')
        sys.exit()        


if __name__ == '__main__':   
   main()


