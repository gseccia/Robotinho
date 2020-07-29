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

    


    #print(mask)
    


    ros_image = self.bridge.cv2_to_imgmsg(mask)
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
