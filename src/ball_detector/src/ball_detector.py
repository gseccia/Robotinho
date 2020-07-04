#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('mytest')
import sys
import rospy
import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox

import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class ball_detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/robot_detector_output",Image, queue_size = 1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.callback, queue_size=1)

    #self.ball_cascade = cv2.CascadeClassifier('/home/mivia/robotinho_ws/src/ball_detector/src/ball_cascade.xml')




  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #print("cv_image: ", cv_image.shape)
    
    bbox, label, conf = cv.detect_common_objects(cv_image, confidence=0.05)
    print(len(bbox))
    for obj in label:
      if obj == "sports ball":
          index = label.index(obj)
          x1 = int(bbox[index][0])
          y1 = int(bbox[index][1])
          x2 = int(bbox[index][2])
          y2 = int(bbox[index][3])
          cv2.rectangle(cv_image, (x1,y1), (x2,y2), (0,255,0), 2)

    #cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #print("cv_gray: ", cv_gray.shape)

    #balls = self.ball_cascade.detectMultiScale(cv_gray, 1.5,2,8, minSize=(8,8))
    #print(len(balls))
    #for (x,y,w,h) in balls:
    #    cv_image = cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)

    #print("cv_image_mod: ", cv_image.shape)
      
    ros_image = self.bridge.cv2_to_imgmsg(cv_image)
    self.image_pub.publish(ros_image)
    


def main(args):
  rospy.init_node('ball_detector', anonymous=True)
  ic = ball_detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    ic.stop()
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)