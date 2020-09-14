#!/usr/bin/env python2
from __future__ import print_function

import itertools
import json
from math import pow, sqrt, cos, sin, atan2,pi
import time

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from tf import transformations
from std_srvs.srv import *


class RobotinhoPlanner:
    def __init__(self):
        print("Planner Initialization...")
        # Goal Position
        self.port = [((6.1, -1.0), (6.1, 1.0)), ((-6.1, -1.0), (-6.1, 1.0))]

        # Odometry
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        # Ros Topic Initialization
        self.ball_position_subscriber = rospy.Subscriber("/robot_ball_position", String, self.ballPositionUpdate)
        self.desired_Position = rospy.Publisher('/robot1/desired_position', Odometry, queue_size=1)
        self.exploration_desired_Position = rospy.Publisher('/robot1/bug2_desired_position', Odometry, queue_size=1)
        self.positionUpdateSub = rospy.Subscriber("/robot1/odom", Odometry, self.positionUpdate)
        self.controlRobot = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)

        self.startSignalSub = rospy.Subscriber("/robot1/start", Bool, self.startUpdate)

        # Ros Service Initialization
        rospy.wait_for_service('/go_to_point_switch')
        self.go_to_point_service = rospy.ServiceProxy('/go_to_point_switch', SetBool)
        
        # Exploration Point
        yPoint = [-2.5,2.5]
        xPoint = [-5.5, -3.5, -1.5, 0.5, 2.5, 4.5, 5.5, 3.5 , 1.5 , -0.5, -2.5, -4.5]
        
        self.explorationPoints = []

        for i, x in enumerate(xPoint):
            self.explorationPoints.append((x, yPoint[i % 2 ]))
        
        self.rate = rospy.Rate(50)

        # Robot Status
        self.is_goal = False
        self.reachDone = False
        self.start = False

        # Ball Status
        self.ball_position = None
        self.ball_attached = None
        self.ball_center = None
        self.ball_last_position = None

        # Parameters
        self.distance_error = 0.1
        self.rate = rospy.Rate(10)

        # Others
        self.waitchar = 0
        self.waitSignals = "\|/-"

        print("DONE!")
    
    def startUpdate(self,msg):
        self.start = msg.data

    def positionUpdate(self, msg):
        # Odometry update
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_theta = euler[2]

    def ballPositionUpdate(self, data):
        # Ball update
        ball_dict = json.loads(data.data)
        if ball_dict["detected"]:
            x1 = ball_dict["x1"]
            x2 = ball_dict["x2"]
            y1 = ball_dict["y1"]
            y2 = ball_dict["y2"]
            # print("{0},{1} - {2},{3} ".format(x1,y1,x2,y2))
            self.ball_center = (int(x2) + int(x1)) / 2
            self.ball_low_y = y2
            if int(y2) > 360 - 60:
                self.ball_attached = True
            else:
                self.ball_attached = False
            
            angleValue = (float(self.ball_center) - 320) * pi / (8*640)

            ball_angle = self.current_theta + angleValue
            self.ball_last_position = (self.current_x, self.current_y,ball_angle)
        else:
            self.ball_center = None
            self.ball_attached = None

    ## Searching Ball Behaviour
    def searchBallBehaviour(self):
        print("SEARCH BALL BEHAVIOUR")
        self.reachDone = False
        if self.ball_last_position is not None:
            # Exists a last ball position ==> Search near last ball position
            print("LAST BALL POSITION")
            current_time = time.time()
            last_time = current_time
            current_timeout = 0
            timeout = 5

            vel_msg = Twist()
            startingPoint = (self.current_x,self.current_y)

            print("Last Ball: ",self.ball_last_position[:2])
            print("Angle Ball: ",self.ball_last_position[2] * 180 / pi)
            
            print("Angle : ",self.current_theta*180/pi)
            while not rospy.is_shutdown() and self.ball_center is None \
                and abs(self.current_theta - self.ball_last_position[2]) > 0.1 \
                and current_timeout < timeout:
                # print("Ruota!")

                angular = -(self.ball_last_position[2] - self.current_theta) / pi

                vel_msg.linear.x = 0
                vel_msg.angular.z = 6.0 * angular

                self.controlRobot.publish(vel_msg)
                self.rate.sleep()

                last_time = current_time
                current_time = time.time()
                current_timeout += current_time - last_time
            
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.controlRobot.publish(vel_msg)
            print("Angle : ",self.current_theta*180/pi)
            last_time = current_time
            current_timeout = 0
            while not rospy.is_shutdown() and self.ball_center is None \
                and sqrt(pow(startingPoint[0] - self.current_x,2) + pow(self.current_y - startingPoint[1],2)) < 2.0 \
                and current_timeout < timeout:
                # print("Avanti!")

                vel_msg.angular.z = 0
                vel_msg.linear.x = 0.7

                self.controlRobot.publish(vel_msg)
                self.rate.sleep()

                last_time = current_time
                current_time = time.time()
                current_timeout += current_time - last_time
            
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.controlRobot.publish(vel_msg)
            self.rate.sleep()
            self.ball_last_position = None
        else:
            # Exhaustive search of the ball
            minIndex = 0
            minDistance = float("inf")

            # Take min-distance point
            if  -pi/2 <= self.current_theta <= pi/2:
                searchMinList = self.explorationPoints[:8]
                forwardCondition = lambda x: x >= self.current_x
            else:
                searchMinList = self.explorationPoints[8:]
                forwardCondition = lambda x: x <= self.current_x
            
            for i, (x, y) in enumerate(searchMinList):
                    distance = sqrt(pow(x - self.current_x, 2) + pow(y - self.current_y, 2))
                    if minDistance > distance and forwardCondition(x):
                        minDistance = distance
                        minIndex = i

            exploredPoint = 0
            markedExploredPoint = {}
            nextPoint = minIndex
            timeout = 60

            # Explore each point
            while exploredPoint < len(self.explorationPoints):
                
                while nextPoint in markedExploredPoint:
                    nextPoint = (nextPoint + 1) % len(self.explorationPoints)
                
                x, y = self.explorationPoints[nextPoint]
                markedExploredPoint[nextPoint] = True

                msg = Odometry()
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y

                self.current_distance = sqrt(pow(x - self.current_x, 2) + pow(y - self.current_y, 2))

                print("Going to: ", x, y)
                current_time = time.time()
                last_time = current_time
                current_timeout = 0

                while not rospy.is_shutdown() and self.current_distance > self.distance_error and current_timeout < timeout and self.ball_center is None:
                    self.exploration_desired_Position.publish(msg)
                    self.current_distance = sqrt(pow(x - self.current_x, 2) + pow(y - self.current_y, 2))
                    current_time = time.time()
                    current_timeout += current_time - last_time
                    last_time = current_time

                    # print("Current Distance: ", self.current_distance, "Current DesPosition ", msg.pose.pose.position, end="\r")

                if rospy.is_shutdown() or self.ball_center is not None:
                    break
                if current_timeout >= timeout:
                    print("TimeOut!")
                
                exploredPoint += 1

                print("End ")

            # STOP
            msg.pose.pose.position.x = self.current_x
            msg.pose.pose.position.y = self.current_y
            self.desired_Position.publish(msg)

    ## Reaching ball behaviour
    def reachTheBallBehaviour(self):
        # Ball at center of screen
        print("REACH THE BALL")

        if self.ball_center is not None:
            # Estimation Ball Position Point
            msg = Odometry()

            # Reach The Ball
            ball_center = self.ball_center
            y_low = self.ball_low_y

            vel_msg = Twist()
            while not rospy.is_shutdown() and not self.ball_attached and ball_center is not None:
                angular = - (float(ball_center) - 320) / 640 if ball_center is not None and not 320 - 5 < ball_center < 320 + 5 else 0

                vel_msg.angular.z = 6.0 * angular
                vel_msg.linear.x = 0.1 * (1.0 - abs(float(ball_center) - 320) / (640)) + 0.25 * float(y_low)/320

                self.controlRobot.publish(vel_msg)
                self.rate.sleep()

                ball_center = self.ball_center
                y_low = self.ball_low_y

            # STOP
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.controlRobot.publish(vel_msg)

            print("STOP")

            if self.ball_attached:
                # Adjust Ball center
                ball_center = self.ball_center
                while not rospy.is_shutdown() and self.ball_attached and ball_center is not None and not 320 - 5 < ball_center < 320 + 5:
                    vel_msg.angular.z = 1.0 if 320 - ball_center > 0 else -1.0
                    vel_msg.linear.x = 0
                    self.controlRobot.publish(vel_msg)
                    self.rate.sleep()
                    ball_center = self.ball_center
                
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0
                self.controlRobot.publish(vel_msg)
            
            self.reachDone = True

    def goalBehavior(self):
        print("GOAL Behaviour!")
        # Ball Position Estimation
        ballEstimationPosition = Point()
        ballEstimationPosition.x = self.current_x + 0.3 * cos(self.current_theta)
        ballEstimationPosition.y = self.current_y + 0.3 * sin(self.current_theta)

        print("Ball Estimation: ",ballEstimationPosition.x,ballEstimationPosition.y)

        # Goal Position
        meanPortPose = Point()
        meanPortPose.x = (self.port[0][0][0] + self.port[0][1][0]) / 2
        meanPortPose.y = (self.port[0][0][1] + self.port[0][1][1]) / 2

        msg = Odometry()

        kickPose = Point()
        middleStepKickPose = Point()
        middleStepKickPose1 = Point()
        middleStepKickPose2 = Point()
        

        kickAngle = atan2(meanPortPose.y - ballEstimationPosition.y,meanPortPose.x - ballEstimationPosition.x)
        print("Kick Angle: ",kickAngle * 180 / pi)

        m = (ballEstimationPosition.y - meanPortPose.y) / (ballEstimationPosition.x - meanPortPose.x)

        # Kick Pose Estimation
        kickPose.x = ballEstimationPosition.x - 0.35*cos(kickAngle)
        kickPose.y = meanPortPose.y + m * (kickPose.x  - meanPortPose.x) 

        # Middle Pose Estimation
        middleStepKickPose1.x = ballEstimationPosition.x + 0.35*sin(kickAngle)
        middleStepKickPose1.y = ballEstimationPosition.y - (1/m) * (middleStepKickPose1.x - ballEstimationPosition.x)

        middleStepKickPose2.x = ballEstimationPosition.x - 0.35*sin(kickAngle)
        middleStepKickPose2.y = ballEstimationPosition.y - (1/m) * (middleStepKickPose2.x - ballEstimationPosition.x)

        if sqrt(pow(middleStepKickPose1.x - self.current_x, 2) + pow(middleStepKickPose1.y - self.current_y,2)) > \
                sqrt(pow(middleStepKickPose2.x - self.current_x, 2) + pow(middleStepKickPose2.y - self.current_y,2)):
            middleStepKickPose = middleStepKickPose2
        else:
            middleStepKickPose = middleStepKickPose1
        
        distancePose = sqrt(pow(middleStepKickPose.x - self.current_x, 2) + pow(middleStepKickPose.y - self.current_y,2))
        distanceKickPose = sqrt(pow(kickPose.x - self.current_x, 2) + pow(kickPose.y - self.current_y,2))
        print("Middle pose: ",middleStepKickPose.x,middleStepKickPose.y)
        print("Kick pose: ",kickPose.x,kickPose.y)

        print("Distance from middle pose: ",distancePose)
        # Reaching Middle Pose
        if distanceKickPose > distancePose and distancePose > 0.1:
            self.go_to_point_service(True)
            msg.pose.pose.position.x = middleStepKickPose.x
            msg.pose.pose.position.y = middleStepKickPose.y
            while not rospy.is_shutdown() and sqrt(pow(msg.pose.pose.position.x - self.current_x, 2) + pow(msg.pose.pose.position.y - self.current_y,
                                                                           2)) > self.distance_error:
                self.desired_Position.publish(msg)
        

        distancePose = sqrt(pow(kickPose.x - self.current_x, 2) + pow(kickPose.y - self.current_y,2))
        print("Distance from kick pose: ",distancePose)
        # Reaching Kick Pose
        if distancePose > 0.10:
            self.go_to_point_service(True)
            msg.pose.pose.position.x = kickPose.x
            msg.pose.pose.position.y = kickPose.y
            while not rospy.is_shutdown() and sqrt(pow(msg.pose.pose.position.x - self.current_x, 2) + pow(msg.pose.pose.position.y - self.current_y,
                                                                           2)) > self.distance_error:
                self.desired_Position.publish(msg)
        
        print("init rotation")
        # Adjust kick angle 
        velMsg = Twist()
        velMsg.linear.x = 0
        velMsg.angular.z = 1.8
        desid_angle = kickAngle
        while not rospy.is_shutdown() and abs(self.current_theta - desid_angle) > 0.05:
            if (self.current_theta - desid_angle)/abs(self.current_theta - desid_angle) > 0:
                velMsg.angular.z = -1.8
            else:
                velMsg.angular.z = 1.8
            self.controlRobot.publish(velMsg)
        velMsg.angular.z = 0 
        self.controlRobot.publish(velMsg)
        print("End Rotation")

        print("Reach Ball")
        # Go near the ball
        vel_msg = Twist()
        ball_center = self.ball_center
        while not rospy.is_shutdown() and not self.ball_attached and ball_center is not None:
            angular = - (float(ball_center) - 320) / 640 if ball_center is not None and not 320 - 5 < ball_center < 320 + 5 else 0

            vel_msg.angular.z = 6.0 * angular
            vel_msg.linear.x = 0.25 * (1.0 - abs(float(ball_center) - 320) / (640))

            self.controlRobot.publish(vel_msg)
            self.rate.sleep()

            ball_center = self.ball_center
        vel_msg.angular.z = 0
        vel_msg.linear.x = 0
        self.controlRobot.publish(vel_msg)
        print("End reach")

        print("Goal Pose")
        # Goal Movement 
        msg.pose.pose.position.x = meanPortPose.x
        msg.pose.pose.position.y = meanPortPose.y
        self.go_to_point_service(True)
        while not rospy.is_shutdown() and sqrt(pow(msg.pose.pose.position.x - self.current_x, 2) + pow(msg.pose.pose.position.y - self.current_y,
                                                                           2)) > self.distance_error and (self.ball_center is not None): # and 320 - 220 < self.ball_center < 320 + 220) :
            self.desired_Position.publish(msg)
        
        # STOP
        msg.pose.pose.position.x = self.current_x
        msg.pose.pose.position.y = self.current_y
        self.desired_Position.publish(msg)

        if sqrt(pow(meanPortPose.x - self.current_x, 2) + pow(meanPortPose.y - self.current_y,2)) <= self.distance_error:
            # Is Goal
            self.is_goal = True
            self.start = False
        else:
            # Ball Lost
            self.reachDone = False
    
    def idleBehaviour(self):
        self.is_goal = False
        print("Waiting start signal "+self.waitSignals[self.waitchar],end = "\r")
        self.waitchar = (self.waitchar + 1) % 4
        self.rate.sleep()

    ## Status Management 
    def planningOnTheFly(self):
        print("RUN")
        while not rospy.is_shutdown():
            if self.is_goal or not self.start:
                self.idleBehaviour()
            elif self.ball_center is None:
                self.searchBallBehaviour()
            elif self.reachDone:
                self.goalBehavior()
            else:
                self.reachTheBallBehaviour()
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("Robot_Planner")

    robotPlanner = RobotinhoPlanner()
    robotPlanner.planningOnTheFly()
