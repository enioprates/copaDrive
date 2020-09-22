#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.signal import argrelextrema

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pickle
import glob
from numpy.linalg import inv


# Define conversions in x and y from pixels space to meters
ym_per_pix = 3.0/720 # meters per pixel in y dimension
xm_per_pix = 0.37/700 # meters per pixel in x dimension

# prepare object points
nx = 4#TODO: enter the number of inside corners in x
ny = 4#TODO: enter the number of inside corners in y

objpoints = []
imgpoints = []

objp = np.zeros((nx*ny, 3), np.float32)
objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1, 2)


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/car3/front_camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    
    car_cascade = cv2.CascadeClassifier('src/image_processing/scripts/cars.xml')
    # convert to gray scale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Detects cars of different sizes in the input image
    cars = car_cascade.detectMultiScale(gray, 1.1, 2)
    print(cars)
    # To draw a rectangle in each cars
    for (x,y,w,h) in cars:
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)


    cv2.imshow("cv_image", cv_image)


    #cv2.imwrite('04.png',cv_image)
    cv2.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
