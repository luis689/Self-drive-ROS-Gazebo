#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt

class camara(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/kobuki/camera/image_raw",Image,self.camera_callback)

    def camera_callback(self,data):
        try:
            img = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except:
            print(e)

        img_2 = np.copy(img)
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])

        

        blur = cv2.GaussianBlur(gray,(5,5),0)
        canny = cv2.Canny(blur,50,150)
        height =canny.shape[0]
        width = canny.shape[1]
        triangle = np.array([[(0,height),(width,height),(300,250)]])
        mask = np.zeros_like(canny)
        cv2.fillPoly(mask,triangle,255)
        img_mask = cv2.bitwise_and(canny,mask)
        minLineLength = 300
        maxLineGap = 5
        lines = cv2.HoughLines(canny,1,np.pi/180,100)
        
        if lines is not None:
            
            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                print(a,b)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv2.line(img_2,(x1,y1),(x2,y2),(0,0,255),2)
        cv2.imshow("Mask",canny)
        cv2.imshow("Resultado",img_2)
        cv2.waitKey(1)

"""def region_de_interes(image):
    width = image.shape[1]
    triangle = np.array([[(0,300),(width,300),(300,250)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask,triangle,255)
    img_mask = cv2.bitwise_and(image,mask)
    return img_mask

def edges(image):
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(blur,50,150)
    return canny"""

def main():
    camera_viwer = camara()
    rospy.init_node("camera_node",anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__== '__main__':
    main()

