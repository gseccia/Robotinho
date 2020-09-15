#!/usr/bin/env python
from __future__ import print_function

import json
import sys
import os

import cv2
import numpy as np
import rospy
from PIL import Image as PilImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolo import YOLO

class ball_detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("/robot_detector_output", Image, queue_size=1)
        self.ball_position_pub = rospy.Publisher("/robot_ball_position", String, queue_size=1)
        self.bridge = CvBridge()
        
        dir_path = os.path.dirname(os.path.realpath(__file__))

        self.yolo = YOLO(model_path=os.path.join(dir_path,'weights.h5'),
                         anchors_path=os.path.join(dir_path,'tiny_yolo_anchors.txt'),
                         classes_path=os.path.join(dir_path,'soccer_ball_classes.txt'),
                         score=0.2,
                         iou=0.3,
                         model_image_size=(320, 640), )

    def detect_soccer_ball(self):
        try:
            data = rospy.wait_for_message("/robot1/camera1/image_raw", Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            pil_image = PilImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        except CvBridgeError as e:
            print(e)

        ball_position = {"detected": False}

        # bbox, label, conf = detect_common_objects(cv2.blur(cv_image,(3,3)), confidence=0.25, model='yolov3-tiny')
        # print(len(bbox))
        # print(label)
        # for obj in label:
        #  #if obj == "sports ball":
        #      index = label.index(obj)
        #      x1 = int(bbox[index][0])
        #      y1 = int(bbox[index][1])
        #      x2 = int(bbox[index][2])
        #      y2 = int(bbox[index][3])
        #      cv2.rectangle(cv_image, (x1,y1), (x2,y2), (0,255,0), 2)

        bbox, label, conf = self.yolo.detect_image(pil_image)
        label = list(label)
        # print(len(label))
        msg = String()
        for obj in label:
            index = label.index(obj)
            y1 = max(0, np.floor(bbox[index][0] + 0.5).astype('int32'))
            x1 = max(0, np.floor(bbox[index][1] + 0.5).astype('int32'))
            y2 = min(cv_image.shape[0], np.floor(bbox[index][2] + 0.5).astype('int32'))
            x2 = min(cv_image.shape[1], np.floor(bbox[index][3] + 0.5).astype('int32'))
            print(x1, y1, x2, y2)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            ball_position["detected"] = True
            ball_position["x1"] = str(x1)
            ball_position["x2"] = str(x2)
            ball_position["y1"] = str(y1)
            ball_position["y2"] = str(y2)

        msg.data = json.dumps(ball_position)
        self.ball_position_pub.publish(msg)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image)
        self.image_pub.publish(ros_image)

def main(args):
    rospy.init_node('ball_detector', anonymous=True)
    ic = ball_detector()
    while not rospy.is_shutdown():
        try:
            ic.detect_soccer_ball()
        except KeyboardInterrupt:
            ic.stop()
            print("Shutting down")
            break

if __name__ == '__main__':
    main(sys.argv)