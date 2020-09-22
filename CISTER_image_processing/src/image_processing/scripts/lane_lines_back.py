#!/usr/bin/env python
#--------------------------------
#Bibliotecas
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.signal import argrelextrema

from image_processing.msg import coords

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pickle
import glob
from numpy.linalg import inv
import os



#--------------------------
#Topico a publicar
pub=rospy.Publisher('X_Y_back', coords, queue_size=10)
lines_old = (0,0,0)
class image_converter:
  
  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/image_topic_3",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/car1/back_camera/image_raw",Image,self.callback)

  def callback(self,data):
    global lines_old
    def select_rgb_white_yellow(image): 
        # white color mask
        lower = np.uint8([250, 250, 250])
        upper = np.uint8([255, 255, 255])
        white_mask = cv2.inRange(image, lower, upper)
        # yellow color mask
        lower = np.uint8([170, 190,   0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(image, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked = cv2.bitwise_and(image, image, mask = mask)
        return masked
    def convert_hsv(image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    def convert_hls(image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    def select_white_yellow(image):
        converted = convert_hsv(image)
        # white color mask
        #lower = np.uint8([ 70, 20, 85])
        #upper = np.uint8([80, 30, 100])
        #white_mask = cv2.inRange(converted, lower, upper)
        # yellow color mask

        lower = np.uint8([0, 120, 0])
        upper = np.uint8([170, 180, 150])

        #lower = np.uint8([0, 120, 0])
        #upper = np.uint8([170, 255, 130])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = yellow_mask # cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask = mask)
    def convert_gray_scale(image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    def apply_smoothing(image, kernel_size=15):
        """
        kernel_size must be postivie and odd
        """
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
    def detect_edges(image, low_threshold=100, high_threshold=150): #50
        return cv2.Canny(image, low_threshold, high_threshold)
    def filter_region(image, vertices):
        """
        Create the mask using the vertices and apply it to the input image
        """
        mask = np.zeros_like(image)
        if len(mask.shape)==2:
            cv2.fillPoly(mask, vertices, 255)
        else:
            cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
        return cv2.bitwise_and(image, mask)  
    def select_region(image):
        """
        It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
        """
        # first, define the polygon by vertices
        bottom_left  = [0, 500]
        top_left     = [0, 370]
        bottom_right = [800, 500]
        top_right    = [800, 370] 
        # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return filter_region(image, vertices)
    def hough_lines(image):
        """
        `image` should be the output of a Canny transform.
        
        Returns hough lines (not the image with lines)
        """
        return cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)
    def draw_lines(image, lines, color=[255, 0, 0], thickness=2, make_copy=True):
        # the lines returned by cv2.HoughLinesP has the shape (-1, 1, 4)
        if make_copy:
            #print ("ok_2")
            image = np.copy(image) # don't want to modify the original
            #print ("ok_2.1")
        #print ("Lines: ",lines.shape)
        for line in lines:
            #print ("ok_3")
            for x1,y1,x2,y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)
        return image
    def average_slope_intercept(lines):
        left_lines    = [] # (slope, intercept)
        left_weights  = [] # (length,)
        right_lines   = [] # (slope, intercept)
        right_weights = [] # (length,)
        #print ("Line: ", lines)
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2==x1:
                    continue # ignore a vertical line
                slope = (y2-y1)/(x2-x1)
                intercept = y1 - slope*x1
                length = np.sqrt((y2-y1)**2+(x2-x1)**2)
                #For PID calculations ( steering )
                msg = coords()
                msg.Dist = slope
                msg.X = x2
                msg.Y = y2		
                pub.publish(msg)
                if slope < 0: # y is reversed in image
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))
        
        # add more weight to longer lines    
        left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
        right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
        print("left_lane", left_lane)
        print("right_lane", right_lane)
        return left_lane, right_lane # (slope, intercept), (slope, intercept)
    def four_point_transform(image):
        # obtain a consistent order of the points and unpack them
        # individually
        rect = np.float32([(10, 370), (790, 370), (800, 500), (0, 500)] )
        (tl, tr, br, bl) = rect

        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")

        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

        # return the warped image
        return warped, M
    ######
  

    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    masked = select_rgb_white_yellow(cv_image)
    chsv = convert_hsv(cv_image)
    #chls = convert_hls(cv_image)
    s_white_yellow = select_white_yellow(cv_image)
    gscale = convert_gray_scale(s_white_yellow)
    #a_smooth_5 = apply_smoothing(gscale, 5) #3, 5, 9, 11, 15, 17 (positive and odd)
    edges = detect_edges(gscale) #a_smooth_5
    roi = select_region(edges)
    lines = hough_lines(roi)
    if (lines is not None):
        lines_old = lines
    else: 
        lines = lines_old
    #print("Lines len: ", lines)
    lines_colored = draw_lines(cv_image, lines)
    average_slope_intercept(lines)
    perspective_transformed, M = four_point_transform(roi)

    cv2.imshow("cv_image", cv_image)
    #cv2.imshow("masked", masked)
    #cv2.imshow("chsv", chsv)
    #cv2.imshow("chls", chls)
    #cv2.imshow("s_white_yellow", s_white_yellow)
    #cv2.imshow("convert_gscale", gscale)
    #cv2.imshow("a_smooth_5", a_smooth_5)
    #cv2.imshow("edges", edges)
    #cv2.imshow("roi", roi)
    cv2.imshow("lines_colored", lines_colored)
    cv2.imshow("perspective_transformed", perspective_transformed)


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
