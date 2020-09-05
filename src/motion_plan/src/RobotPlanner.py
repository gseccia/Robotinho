#!/usr/bin/env python2
from __future__ import print_function
import rospy
from std_msgs.msg import String,Float32MultiArray,Bool
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from math import pow,sqrt,pi,floor,atan2,cos,sin,tan,exp
from cv_bridge import CvBridge, CvBridgeError
import json
import numpy as np
import cv2
import itertools


LEFT = -1
RIGHT = 1

class RobotinhoPlanner:
    def __init__(self):
        print("Planner Initialization...")
        self.port = [ ((6.1,-1.0),(6.1,1.0)) , ((-6.1,-1.0),(-6.1,1.0))]

        self.current_x = 0
        self.current_y = 0

        self.ball_position_subscriber = rospy.Subscriber("/robot_ball_position",String,self.ballPositionUpdate)
        self.desired_Position = rospy.Publisher('/robot1/desired_position', Odometry, queue_size= 1)
        self.positionUpdateSub = rospy.Subscriber("/robot1/odom",Odometry,self.positionUpdate)
        
        self.explorePositionPub = rospy.Publisher("/robot1/des_position", Odometry, queue_size= 1)
        self.exploreResponse = rospy.Subscriber("/robot1/path_exist", Bool, self.respUpdate) 
        
        self.map = rospy.Subscriber("/map_image",Image,self.mapUpdate)
        self.bridge = CvBridge()

        self.map = None

        yPoint = [-2.5,-1.5,0.0,1.5,2.5]
        rev_yPoint = [2.5,1.5,0.0,-1.5,-2.5]

        xPoint = [-5.5,-4.5,-3.5,-2.5,-1.5,0.0,1.5,2.5,3.5,4.5,5.5]
        self.explorationPoints = []

        for i,x in enumerate(xPoint):
            if i % 2 == 0:
                self.explorationPoints += list(itertools.product([x],yPoint))
            else:
                self.explorationPoints += list(itertools.product([x],rev_yPoint))

        

        self.bridge = CvBridge()
        self.ball_position = None
        self.is_goal = False
        self.ball_attached = None
        self.ball_center = None
        self.ball_last_position = None

        self.occupancy_grid = None
        self.obstaclePresence = False
        self.availablePosition = None

        self.distance_error = 0.3


        print("DONE!")
    
    def respUpdate(self,msg):
        self.availablePosition = msg.data
    
    def positionUpdate(self,msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
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
            self.ball_center = (int(x2) + int(x1)) /2
            if int(y2) == 360:
                self.ball_attached = True
            else:
                self.ball_attached = False
            self.ball_last_position = (self.current_x,self.current_y)
        else:
            self.ball_center = None
            self.ball_attached = None
    
    def searchBallBehaviour(self):
        print("SEARCH BALL BEHAVIOUR")
        minIndex = 0
        minDistance = float("inf")

        for i,(x,y) in enumerate(self.explorationPoints):
            distance = sqrt(pow(x - self.current_x,2) + pow(y - self.current_y,2))
            if minDistance > distance:
                minDistance = distance
                minIndex = i
        
        exploredPoint = 0
        nextPoint = minIndex

        while exploredPoint < len(self.explorationPoints):
            x,y = self.explorationPoints[nextPoint]

            msg = Odometry()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            
            self.availablePosition = None
            self.explorePositionPub.publish(msg)
            self.current_distance = sqrt(pow(x - self.current_x,2) + pow(y - self.current_y,2))

            print("Going to: ",x,y)

            while not rospy.is_shutdown() and self.current_distance > self.distance_error and (self.availablePosition is None or self.availablePosition) and self.ball_center is None:
                self.desired_Position.publish(msg)
                self.explorePositionPub.publish(msg)
                self.current_distance = sqrt(pow(x - self.current_x,2) + pow(y - self.current_y,2))

                print("Current Distance: ",self.current_distance, " Available Position: ",self.availablePosition, "Current DesPosition ",msg.pose.pose.position)
                
            if rospy.is_shutdown() or self.ball_center is not None:
                break
            
            exploredPoint += 1
            nextPoint = (nextPoint + 1) % len(self.explorationPoints)
            
            print("End ")

    
    def planningOnTheFly(self):
        print("RUN")
        # Add a better State Management
        while not rospy.is_shutdown() and not self.is_goal:           
            if self.ball_center is None:
                self.searchBallBehaviour()
            elif self.ball_attached is not None:
                print("BALL FOUND!")
            else:
                print("BALL FOUND, NOT REACHED!")
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("Robot_Planner")

    robotPlanner = RobotinhoPlanner()
    robotPlanner.planningOnTheFly()