#!/usr/bin/env python2
from __future__ import print_function

import itertools
import json
from math import pow, sqrt, cos, sin, atan2,pi

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from tf import transformations

LEFT = -1
RIGHT = 1


class RobotinhoPlanner:
    def __init__(self):
        print("Planner Initialization...")
        self.port = [((6.1, -1.0), (6.1, 1.0)), ((-6.1, -1.0), (-6.1, 1.0))]

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.ball_position_subscriber = rospy.Subscriber("/robot_ball_position", String, self.ballPositionUpdate)
        self.desired_Position = rospy.Publisher('/robot1/desired_position', Odometry, queue_size=1)
        self.positionUpdateSub = rospy.Subscriber("/robot1/odom", Odometry, self.positionUpdate)
        self.controlRobot = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)

        self.explorePositionPub = rospy.Publisher("/robot1/des_position", Odometry, queue_size=1)
        self.exploreResponse = rospy.Subscriber("/robot1/path_exist", Bool, self.respUpdate)

        self.map = rospy.Subscriber("/map_image", Image, self.mapUpdate)
        self.bridge = CvBridge()

        self.map = None

        yPoint = [-2.5, -1.5, 0.0, 1.5, 2.5]
        rev_yPoint = [2.5, 1.5, 0.0, -1.5, -2.5]
        xPoint = [-5.5, -4.5, -3.5, -2.5, -1.5, 0.0, 1.5, 2.5, 3.5, 4.5, 5.5]

        self.explorationPoints = []

        for i, x in enumerate(xPoint):
            if i % 2 == 0:
                self.explorationPoints += list(itertools.product([x], yPoint))
            else:
                self.explorationPoints += list(itertools.product([x], rev_yPoint))

        self.rate = rospy.Rate(50)

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

    def respUpdate(self, msg):
        self.availablePosition = msg.data

    def positionUpdate(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_theta = euler[2]

    def mapUpdate(self, msg):
        try:
            data = rospy.wait_for_message("/robot1/camera1/image_raw", Image)

            self.map = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def ballPositionUpdate(self, data):
        ball_dict = json.loads(data.data)
        if ball_dict["detected"]:
            x1 = ball_dict["x1"]
            x2 = ball_dict["x2"]
            y1 = ball_dict["y1"]
            y2 = ball_dict["y2"]
            # print("{0},{1} - {2},{3} ".format(x1,y1,x2,y2))
            self.ball_center = (int(x2) + int(x1)) / 2
            if int(y2) > 360 - 60:
                self.ball_attached = True
            else:
                self.ball_attached = False
            ball_angle = self.current_theta + (float(self.ball_center) - 320) / 640
            self.ball_last_position = (self.current_x, self.current_y,ball_angle)
        else:
            self.ball_center = None
            self.ball_attached = None

    def searchBallBehaviour(self):
        print("SEARCH BALL BEHAVIOUR")
        if self.ball_last_position is not None:

            vel_msg = Twist()
            startingPoint = (self.current_x,self.current_y)
            while not rospy.is_shutdown() and self.ball_center is None \
                and sqrt(pow(startingPoint[0] - self.current_x,2) + pow(self.current_y - startingPoint[1],2)) < 2.0:
                angular = (self.current_theta - self.ball_last_position[2])

                vel_msg.angular.z = 6.0 * angular
                vel_msg.linear.x = 0.7

                self.controlRobot.publish(vel_msg)
            
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.controlRobot.publish(vel_msg)
            self.ball_last_position = None
        else:

            minIndex = 0
            minDistance = float("inf")

            for i, (x, y) in enumerate(self.explorationPoints):
                distance = sqrt(pow(x - self.current_x, 2) + pow(y - self.current_y, 2))
                if minDistance > distance:
                    minDistance = distance
                    minIndex = i

            exploredPoint = 0
            nextPoint = minIndex

            while exploredPoint < len(self.explorationPoints):
                x, y = self.explorationPoints[nextPoint]

                msg = Odometry()
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y

                self.availablePosition = None
                self.explorePositionPub.publish(msg)
                self.current_distance = sqrt(pow(x - self.current_x, 2) + pow(y - self.current_y, 2))

                print("Going to: ", x, y)

                while not rospy.is_shutdown() and self.current_distance > self.distance_error and (
                        self.availablePosition is None or self.availablePosition) and self.ball_center is None:
                    self.desired_Position.publish(msg)
                    self.explorePositionPub.publish(msg)
                    self.current_distance = sqrt(pow(x - self.current_x, 2) + pow(y - self.current_y, 2))

                    print("Current Distance: ", self.current_distance, " Available Position: ", self.availablePosition,
                        "Current DesPosition ", msg.pose.pose.position, end="\r")

                if rospy.is_shutdown() or self.ball_center is not None:
                    break

                exploredPoint += 1
                nextPoint = (nextPoint + 1) % len(self.explorationPoints)

                print("End ")

            # STOP
            msg.pose.pose.position.x = self.current_x
            msg.pose.pose.position.y = self.current_y
            self.desired_Position.publish(msg)

    def reachTheBallBehaviour(self):
        # Ball at center of screen
        print("REACH THE BALL")
        """msg.pose.pose.position.x = self.current_x
        msg.pose.pose.position.y = self.current_y
        self.desired_Position.publish(msg)"""

        if self.ball_center is not None:
            # Estimation Ball Position Point
            msg = Odometry()

            # Reach The Ball
            ball_center = self.ball_center

            vel_msg = Twist()
            while not rospy.is_shutdown() and not self.ball_attached and ball_center is not None:
                angular = - (float(ball_center) - 320) / 640 if ball_center is not None and not 320 - 5 < ball_center < 320 + 5 else 0

                vel_msg.angular.z = 6.0 * angular
                vel_msg.linear.x = 0.25 * (1.0 - abs(float(ball_center) - 320) / (640))

                self.controlRobot.publish(vel_msg)

                ball_center = self.ball_center

            """distance = 0.3
            msg.pose.pose.position.x = self.current_x + (distance) * cos(
                self.current_theta - (float(ball_center) - 320) / 640)
            msg.pose.pose.position.y = self.current_y + (distance) * sin(
                self.current_theta - (float(ball_center) - 320) / 640)

            while not rospy.is_shutdown() and not self.ball_attached and ball_center is not None:
                self.desired_Position.publish(msg)
                ball_center = self.ball_center
                if sqrt(pow(msg.pose.pose.position.x - self.current_x, 2) + pow(
                        msg.pose.pose.position.y - self.current_y,
                        2)) < self.distance_error and ball_center is not None:
                    msg.pose.pose.position.x = self.current_x + (distance) * cos(
                        self.current_theta - (float(ball_center) - 320) / 640)
                    msg.pose.pose.position.y = self.current_y + (distance) * sin(
                        self.current_theta - (float(ball_center) - 320) / 640)"""

            # STOP
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.controlRobot.publish(vel_msg)

            print("STOP")

            if self.ball_attached:
                ball_center = self.ball_center
                while not rospy.is_shutdown() and self.ball_attached and ball_center is not None and not 320 - 10 < ball_center < 320 + 10:
                    vel_msg.angular.z = 1.0 if 320 - ball_center > 0 else -1.0
                    vel_msg.linear.x = 0
                    self.controlRobot.publish(vel_msg)
                    ball_center = self.ball_center
                
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0
                self.controlRobot.publish(vel_msg)
    
    def reachPose(self,pose,distance_error = 0.1):
        velMsg = Twist()
        velMsg.linear.x = 0
        velMsg.angular.z = 0

        desid_angle = atan2(pose.y - self.current_y,pose.x - self.current_x)
        print("Kick Pose Angle")
        velMsg.angular.z = 1.2
        while not rospy.is_shutdown() and abs(self.current_theta - desid_angle) > 0.05:
            if (self.current_theta - desid_angle)/abs(self.current_theta - desid_angle) > 0:
                velMsg.angular.z = -1.2
            else:
                velMsg.angular.z = 1.2
            print("Current Angle: {0} Desid angle: {1} Angular Error: {2} ABS Angular Error: {3} Control Vel: {4}".format(self.current_theta,desid_angle,self.current_theta - desid_angle,abs(self.current_theta - desid_angle),velMsg.angular.z),end = "\r")
            self.controlRobot.publish(velMsg)

        print("Kick Pose Linear")
        velMsg.linear.x = 0.4
        velMsg.angular.z = 0
        while not rospy.is_shutdown() and sqrt(pow(pose.x - self.current_x, 2) + pow(pose.y - self.current_y,2)) > distance_error:
            print("Error Pose: {0} Pose: {1} {2} Current: {3} {4}".format(sqrt(pow(pose.x - self.current_x, 2) + pow(pose.y - self.current_y,2)),pose.x,pose.y,self.current_x,self.current_y),end = "\r")
            if self.current_theta - desid_angle > 0.1:
                velMsg.angular.z = -1.2
                velMsg.linear.x = 0
            elif self.current_theta - desid_angle < -0.1:
                velMsg.angular.z = 1.2
                velMsg.linear.x = 0
            else:
                velMsg.angular.z = 0
                velMsg.linear.x = 0.4
            self.controlRobot.publish(velMsg)
            # self.desired_Position.publish(msg)
    
        velMsg.linear.x = 0
        velMsg.angular.z = 0
        self.controlRobot.publish(velMsg)

    def goalBehavior(self):
        print("GOAL Behaviour!")
        ballEstimationPosition = Point()
        ballEstimationPosition.x = self.current_x + 0.2 * cos(self.current_theta)
        ballEstimationPosition.y = self.current_y + 0.2 * sin(self.current_theta)

        meanPortPose = Point()
        meanPortPose.x = (self.port[0][0][0] + self.port[0][1][0]) / 2
        meanPortPose.y = (self.port[0][0][1] + self.port[0][1][1]) / 2

        msg = Odometry()
        # Senza Considerare Ostacoli lungo il tragitto

        kickPose = Point()
        middleStepKickPose = Point()
        middleStepKickPose1 = Point()
        middleStepKickPose2 = Point()
        
        kickAngle = atan2(meanPortPose.y - ballEstimationPosition.y,meanPortPose.x - ballEstimationPosition.x)
        print("Kick Angle: ",kickAngle * 180 / pi)
        
        kickPose.y = ballEstimationPosition.y + 0.4*sin(kickAngle)
        kickPose.x = ballEstimationPosition.x + -0.4*cos(kickAngle)

        middleStepKickPose1.x = ballEstimationPosition.x + 0.4*cos(kickAngle + pi/2)
        middleStepKickPose1.y = ballEstimationPosition.y + 0.4*sin(kickAngle + pi/2)
        middleStepKickPose2.x = ballEstimationPosition.x + 0.4*cos(kickAngle - pi/2)
        middleStepKickPose2.y = ballEstimationPosition.y + 0.4*sin(kickAngle - pi/2)

        if sqrt(pow(middleStepKickPose1.x - self.current_x, 2) + pow(middleStepKickPose1.y - self.current_y,2)) > \
                sqrt(pow(middleStepKickPose2.x - self.current_x, 2) + pow(middleStepKickPose2.y - self.current_y,2)):
            middleStepKickPose = middleStepKickPose2
        else:
            middleStepKickPose = middleStepKickPose1
        

        """kickPose.x = ballEstimationPosition.x - 0.45
        kickPose.y = 0.1 + meanPortPose.y + (ballEstimationPosition.y - meanPortPose.y) * \
                                   (kickPose.x  - meanPortPose.x) / (ballEstimationPosition.x - meanPortPose.x)"""
        

        distancePose = sqrt(pow(middleStepKickPose.x - self.current_x, 2) + pow(middleStepKickPose.y - self.current_y,2))
        distanceKickPose = sqrt(pow(kickPose.x - self.current_x, 2) + pow(kickPose.y - self.current_y,2))
        print("Middle pose: ",middleStepKickPose.x,middleStepKickPose.y)
        print("Kick pose: ",kickPose.x,kickPose.y)

        print("Distance from middle pose: ",distancePose)
        if distanceKickPose > distancePose and distancePose > 0.1:
            self.reachPose(middleStepKickPose, distance_error = 0.1)
        
        distancePose = sqrt(pow(kickPose.x - self.current_x, 2) + pow(kickPose.y - self.current_y,2))
        print("Distance from kick pose: ",distancePose)
        if distancePose > 0.15:
            self.reachPose(kickPose, distance_error = 0.15)

        print("Goal Pose")
    
        msg.pose.pose.position.x = meanPortPose.x
        msg.pose.pose.position.y = meanPortPose.y
        while not rospy.is_shutdown() and sqrt(pow(msg.pose.pose.position.x - self.current_x, 2) + pow(msg.pose.pose.position.y - self.current_y,
                                                                           2)) > self.distance_error:
            self.desired_Position.publish(msg)
        
        self.is_goal = True

    def planningOnTheFly(self):
        print("RUN")
        # Add a better State Management
        while not rospy.is_shutdown() and not self.is_goal:
            if self.ball_center is None:
                self.searchBallBehaviour()
            elif self.ball_attached:
                self.goalBehavior()
            else:
                self.reachTheBallBehaviour()
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("Robot_Planner")

    robotPlanner = RobotinhoPlanner()
    robotPlanner.planningOnTheFly()
