#!/usr/bin/env python2
from __future__ import print_function

import sys
import rospy
import cv2


import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class obstacle_segmenter():



  def __init__(self):
    self.image_pub = rospy.Publisher("/robot_obstacle_seg_output",Image, queue_size = 1)

    self.image_wh = [640,360]
    self.movement_th = 30
    self.vertical_cropping = np.asarray([0.0,2.0,1.0,1.0,1.0,2.0]) * self.image_wh[0] * (1.0/7)
    self.horizontal_cropping = np.asarray([0.0,2.0,1.0]) * (self.image_wh[1] - self.movement_th) * (1.0/3)

    self.vertical_cropping = map(int,self.vertical_cropping)
    self.horizontal_cropping = map(int,self.horizontal_cropping)
    

    self.occupancy_grid = [[0 for i in range(len(self.vertical_cropping))] for j in range(len(self.horizontal_cropping))]


    self.bridge = CvBridge()


  def get_obstacle_mask(self):
    try:
      data = rospy.wait_for_message("/robot1/camera1/image_raw",Image)

      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    
    # converting from BGR to HSV color space
    hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

    # Range for lower red
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    # Range for upper range
    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsv,lower_red,upper_red)

    # Generating the final mask to detect red color
    mask = mask1+mask2
    mask_grid = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    #vert [0 182, 91, 91, 91, 182]
    #hor [0 300 150]
    x0 = self.vertical_cropping[0]
    x1 = x0 + self.vertical_cropping[1]
    x2 = x1 + self.vertical_cropping[2]
    x3 = x2 + self.vertical_cropping[3]
    x4 = x3 + self.vertical_cropping[4]
    x5 = x4 + self.vertical_cropping[5]
    y0 = self.horizontal_cropping[0]
    y1 = y0 + self.horizontal_cropping[1]
    y2 = y1 + self.horizontal_cropping[2]
    

    cv2.rectangle(mask_grid, (x0, y0), (x1, y1), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x1, y0), (x2, y1), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x2, y0), (x3, y1), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x3, y0), (x4, y1), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x4, y0), (x5, y1), (255,0,0), 2)
    
    cv2.rectangle(mask_grid, (x0, y1), (x1, y2), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x1, y1), (x2, y2), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x2, y1), (x3, y2), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x3, y1), (x4, y2), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x4, y1), (x5, y2), (255,0,0), 2)
    
    cv2.rectangle(mask_grid, (x0, y2), (x1, self.image_wh[1]), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x1, y2), (x2, self.image_wh[1]), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x2, y2), (x3, self.image_wh[1]), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x3, y2), (x4, self.image_wh[1]), (255,0,0), 2)
    cv2.rectangle(mask_grid, (x4, y2), (x5, self.image_wh[1]), (255,0,0), 2)
    



    #print(mask)
    


    ros_image = self.bridge.cv2_to_imgmsg(mask_grid)
    self.image_pub.publish(ros_image)
    


def main(args):
  rospy.init_node('obstacle_segmenter', anonymous=True)
  ic = obstacle_segmenter()
  while not rospy.is_shutdown():
    try:
      ic.get_obstacle_mask()
    except KeyboardInterrupt:
      print("Shutting down")
      break

if __name__ == '__main__':
    main(sys.argv)


