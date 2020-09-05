#!/usr/bin/env python2
from __future__ import print_function
import rospy
from std_msgs.msg import String,Float32MultiArray
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from robotinhoMotionModule import RobotinhoController, Pose
from math import pow,sqrt,pi,floor,atan2,cos,sin,tan,exp
from cv_bridge import CvBridge, CvBridgeError
import json
import numpy as np
import cv2
from robotinhoMapRepresentation import Map,Tile


LEFT = -1
RIGHT = 1



class RobotinhoPlanner:
    def __init__(self):
        print("Planner Initialization...")
        self.port = [ ((6.1,-1.0),(6.1,1.0)) , ((-6.1,-1.0),(-6.1,1.0))]

        self.ball_position_subscriber = rospy.Subscriber("/robot_ball_position",String,self.ballPositionUpdate)
        self.desired_Position = rospy.Publisher('/robot1/desired_position', Odometry, queue_size= 1)
        
        self.map = rospy.Subscriber("/map_image",Image,self.mapUpdate)
        self.bridge = CvBridge()

        self.map = None
        
        self.desired_Position = Odometry()

        self.bridge = CvBridge()
        self.ball_position = None
        self.is_goal = False
        self.ball_attached = None
        self.ball_center = None
        self.ball_last_position = None

        self.occupancy_grid = None
        self.obstaclePresence = False


        print("DONE!")

    def initialize(self):
        while not self.controller.init_complete:
            pass
    
    def mapUpdate(self,msg):
        try:
          data = rospy.wait_for_message("/robot1/camera1/image_raw",Image)

          self.map = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

    def ballPositionUpdate(self,data):
        ball_dict = json.loads(data.data)
        if ball_dict["detected"]:
            x1 = ball_dict["x1"]
            x2 = ball_dict["x2"] 
            y1 = ball_dict["y1"] 
            y2 = ball_dict["y2"] 
            # print("{0},{1} - {2},{3} ".format(x1,y1,x2,y2))
            if self.ball_center is None:
                self.controller.forceStop()
            self.ball_center = (int(x2) + int(x1)) /2
            if int(y2) == 360:
                self.ball_attached = True
            else:
                self.ball_attached = False
            self.ball_last_position = (self.controller.pose.x,self.controller.pose.y)
        else:
            if self.ball_center is not None:
                self.controller.forceStop()
            self.ball_center = None
            self.ball_attached = None
    
    def searchBallBehaviour(self):
        print("SEARCH BALL BEHAVIOUR")

        
        

    
    def planningOnTheFly(self):
        print("RUN")
        # Add a better State Management
        meanPortPose = Pose((self.port[0][0][0] + self.port[0][1][0])/2,(self.port[0][0][1] + self.port[0][1][1])/2)
        while not rospy.is_shutdown() and not self.is_goal:           
            if self.ball_center is None:
                self.searchBallBehaviour()
            elif self.ball_attached is not None:
                print("BALL FOUND!")
        rospy.spin()


if __name__ == "__main__":
    robotPlanner = RobotinhoPlanner()
    robotPlanner.initialize()
    robotPlanner.planningOnTheFly()