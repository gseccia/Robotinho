#!/usr/bin/env python2
from __future__ import print_function
import rospy
from std_msgs.msg import String,Float32MultiArray
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from robotinhoMotionModule import RobotinhoController, Pose
from math import pow,sqrt,pi,floor,atan2,cos,sin,tan
from cv_bridge import CvBridge, CvBridgeError
import json
import numpy as np
import cv2


LEFT = -1
RIGHT = 1

class RobotinhoPlanner:
    def __init__(self):
        print("Planner Initialization...")
        self.port = [ ((6.1,-1.0),(6.1,1.0)) , ((-6.1,-1.0),(-6.1,1.0))]

        self.occupancy_grid_subscriber = rospy.Subscriber("/occupancy_grid",Float32MultiArray,self.occupancyGridUpdate)
        self.ball_position_subscriber = rospy.Subscriber("/robot_ball_position",String,self.ballPositionUpdate)
        self.bridge = CvBridge()
        self.controller = RobotinhoController()
        self.ball_position = None # (0.0,0.0) # Real init None
        self.is_goal = False
        self.ball_attached = None
        self.ball_center = None
        self.ball_last_position = None
        self.explored_cell = {}

        self.occupancy_grid = None
        self.obstaclePresence = False

        for i in range(-6,6):
            self.explored_cell[i] = {}
            for j in range(-3,3):
                self.explored_cell[i][j] = False
        print("DONE!")

    def printExploredMap(self):
        out = "----------MAP EXPLORED-------------\n"
        for i in range(-6,6):
            for j in range(-3,3):
                out += "|" + str(self.explored_cell[i][j]) + "|"
            out += "\n"
        out += "\n\n"
        print(out)

    def initialize(self):
        while not self.controller.init_complete:
            pass

    def occupancyGridUpdate(self,data):
        self.occupancy_grid = np.reshape(data.data,(3,5))
        
        self.obstaclePresence =  self.occupancy_grid[2][0] > 0 or self.occupancy_grid[2][1] > 0 or self.occupancy_grid[2][2] > 0

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
            self.ball_last_position = (self.controller.pose.x,self.controller.pose.y)
        else:
            self.ball_center = None
            self.ball_attached = None

    def genericLinePlanner(self,initialPosition,targetPosition,limit_lenght_perc = None):
        curve_lenght = sqrt(pow((targetPosition.x - initialPosition.x),2) + pow((targetPosition.y - initialPosition.y),2))
        if limit_lenght_perc is None:
            limit_lenght = curve_lenght
        else:
            limit_lenght = limit_lenght_perc * curve_lenght
        p_s = []
        x = float(curve_lenght) / 100
        while x < limit_lenght:
            p_s.append(initialPosition + (targetPosition - initialPosition) * (x /curve_lenght))
            x += float(curve_lenght) / 100
        return p_s

    def searchBallBehaviour(self):
        print("SEARCH BALL BEHAVIOUR")
        current_x = floor(self.controller.pose.x)
        current_y = floor(self.controller.pose.y)
        points = self.genericLinePlanner(self.controller.pose,Pose(current_x + 0.5,current_y +0.5))

        next_point = 0
        while next_point < len(points) and self.ball_center is None:
            self.controller.move2goal(points[next_point])
            next_point += 1
        

        if self.ball_last_position is None:
            current_x = int(floor(self.controller.pose.x))
            current_y = int(floor(self.controller.pose.y))
        else:
            next_cell_x = int(floor(self.ball_last_position[0]))
            current_y = int(floor(self.ball_last_position[1]))

        next_cell_x = current_x
        next_cell_y = current_y

        while self.ball_center is None:
            self.printExploredMap()
            print("NEXT CELL: ",next_cell_x," - ",next_cell_y)

            points = self.genericLinePlanner(self.controller.pose,Pose(next_cell_x + 0.5,next_cell_y + 0.5))
            next_point = 0
            while next_point < len(points) and self.ball_center is None:
                self.controller.move2goal(points[next_point])
                next_point += 1
            
            current_x = int(floor(self.controller.pose.x))
            current_y = int(floor(self.controller.pose.y))
            next_cell_x = current_x
            next_cell_y = current_y
            for y in self.explored_cell[current_x]:
                if not self.explored_cell[current_x][y]:
                    next_cell_y = y
                    break
            if next_cell_y == current_y:
                for x in range(current_x,6):
                    for y in self.explored_cell[x]:
                        if not self.explored_cell[x][y]:
                            next_cell_y = y
                            next_cell_x = x
                            break
            
    def mantainTarget(self,th = 5):
        if self.ball_center is not None and not 320 - th < self.ball_center < 320 + th:
            print("BALL CENTER: ",self.ball_center)
            angular_vel = - (float(self.ball_center) - 320) / 640
        else:
            angular_vel = 0
        return angular_vel

    def reachTheBall(self,th = 5):
        vel_msg = Twist()
        angular = self.mantainTarget(th)
        print("ANGULAR: ",angular)
        vel_msg.angular.z = 6.0 * angular
        vel_msg.linear.x = 0.25 * (1.0 - abs(float(self.ball_center) - 320) / (640))
        
        self.controller.velocity_publisher.publish(vel_msg)
        self.controller.rate.sleep()
        print(vel_msg)

    def freeSpaceBehaviour(self):
        print("FREE SPACE BEHAVIOUR")
        # Reset Speed
        self.controller.stop()

        # Reach the ball
        while self.ball_center is not None and not self.ball_attached and not self.obstaclePresence:
            self.reachTheBall()
        
        # STOP
        self.controller.stop()

        if not self.obstaclePresence:

        
            vel_msg = Twist()
            if self.ball_attached:
                # Adjust ball - robot angle
                angular = self.mantainTarget()
                while self.ball_center is not None and self.ball_attached and angular != 0:
                    print("ANGULAR: ",angular)
                    vel_msg.angular.z = 6.0 * angular
                    self.controller.velocity_publisher.publish(vel_msg)
                    self.controller.rate.sleep()
                    angular = self.mantainTarget()
                
                # STOP
                self.controller.stop()
                
                self.ballToGoal()


    def ballToGoal(self):
        print("BALL TO GOAL")
        # Estimation Ball Position
        distance = 0.3
        estimate_ball_position = [self.controller.pose.x + distance*cos(self.controller.pose.theta),self.controller.pose.y + distance*sin(self.controller.pose.theta)]

        print("ESTIMATED BALL POSITION: ",estimate_ball_position)

        # Estimation Kick Position
        meanPortPose = Pose((self.port[0][0][0] + self.port[0][1][0])/2,(self.port[0][0][1] + self.port[0][1][1])/2)
        x = estimate_ball_position[0] - 0.3 
        y = meanPortPose.y + (estimate_ball_position[1] - meanPortPose.y)*(x - meanPortPose.x)/(estimate_ball_position[1] - meanPortPose.x)
        ballKickPose = Pose(x,y)

        # Planning Path to Kick Position
        points = self.genericLinePlanner(self.controller.pose,ballKickPose)
        for point in points:
            self.controller.move2goal(point)
        theta = atan2(meanPortPose.y - self.controller.pose.y ,meanPortPose.x - self.controller.pose.x)
        self.controller.rotate(theta)

        # Go to GOAL!
        points = self.genericLinePlanner(self.controller.pose,meanPortPose)
        while len(points) > 1 and self.ball_center is not None:
            # Move to GOAL!
            self.controller.move2goal(points[0])

            # Adjust Ball Position if you miss it!
            angular = self.mantainTarget(80)
            if angular != 0:
                estimate_ball_position = [self.controller.pose.x + distance*cos(self.controller.pose.theta),self.controller.pose.y + distance*sin(self.controller.pose.theta)]
                x = estimate_ball_position[0] - 0.3 
                y = meanPortPose.y + (estimate_ball_position[1] - meanPortPose.y)*(x - meanPortPose.x)/(estimate_ball_position[1] - meanPortPose.x)
                ballKickPose = Pose(x,y) 
                points = self.genericLinePlanner(self.controller.pose,ballKickPose)
                for point in points:
                    self.controller.move2goal(point)
                theta = atan2(meanPortPose.y - self.controller.pose.y ,meanPortPose.x - self.controller.pose.x)
                self.controller.rotate(theta)

            # Regenerate Plan
            points = self.genericLinePlanner(self.controller.pose,meanPortPose)
        
        if self.ball_center is not None:
            # GOAL!!!!!
            self.is_goal = True
            print("GOAL!")
    
    def freeCell(self,value,th = 0.5):
        return value < th
    
    def freeDirection(self, row):
        return np.dot(self.occupancy_grid[row],[-1,-1,0,1,1])

    def obstacleAviodanceBehaviour(self):
        print("OBSTACLE AVOIDANCE BEHAVIOUR")
        if self.occupancy_grid is not None:
            offset_angle = 0

            for i in range(3):
                for j in range(5):
                    region = chr(ord("A")+i) + str(j+1)

                    print("{0} : {1:.3f}".format(region,self.occupancy_grid[i][j]))

            # Evaluate Grid 
            if not self.freeCell(self.occupancy_grid[0][3],th = 0.3):
                offset_angle =  (0.1*self.freeDirection(0) + 0.5*self.freeDirection(1) / 0.6) * pi/2
                
                print("OFFSET ANGLE: ",offset_angle)

                current_x = floor(self.controller.pose.x)
                current_y = floor(self.controller.pose.y)
                meanPortPose = Pose((self.port[0][0][0] + self.port[0][1][0])/2,(self.port[0][0][1] + self.port[0][1][1])/2)

                middlePoint_x = (current_x + meanPortPose.x) / 2
                middlePoint_y = (current_y + meanPortPose.y) / 2

                distance = sqrt( pow(middlePoint_x - current_x,2) +  pow(middlePoint_y - current_y,2))
                x = middlePoint_x + distance*tan(offset_angle)*sin(offset_angle)
                y = middlePoint_y + distance*tan(offset_angle)*cos(offset_angle)
                obstacleAvoidancePoint = Pose(x,y)

                print("AVOID POINT:",obstacleAvoidancePoint)

                # Add Kick pose!
                # !!!!!!!!!!!!!

                points = self.genericLinePlanner(self.controller.pose,obstacleAvoidancePoint)
                return points

            # Pure Rotation
            elif not self.freeCell(self.occupancy_grid[2][3],th = 0.3):
                offset_angle = pi/2 if self.freeDirection(2) < 0 else pi/2

                # Add Kick pose!
                # !!!!!!!!!!!!!!

                self.controller.rotate(offset_angle)
                return None
        return None
            

    def planningOnTheFly(self):
        print("RUN")
        # Add a better State Management
        meanPortPose = Pose((self.port[0][0][0] + self.port[0][1][0])/2,(self.port[0][0][1] + self.port[0][1][1])/2)
        while not rospy.is_shutdown() and not self.is_goal:
            
            points = self.obstacleAviodanceBehaviour()
            if points is None:
                finalPoints = self.genericLinePlanner(self.controller.pose,meanPortPose)
                self.obstaclePresence = False
                print("Vado in Porta: ")
                i = 0
                while i < len(finalPoints) and not self.obstaclePresence:
                    self.controller.move2goal(finalPoints[i])
                    self.obstaclePresence = not self.freeCell(self.occupancy_grid[1][3],0.2)
                    print(finalPoints[i])
                    i += 1
            else:
                finalPoints = points
                self.obstaclePresence = True
                print("Evito ostacolo: ")
                i = 0
                while i < len(finalPoints) and self.obstaclePresence:
                    self.controller.move2goal(finalPoints[i])
                    self.obstaclePresence = not self.freeCell(self.occupancy_grid[1][3],0.2)
                    print(finalPoints[i])
                    i += 1
            
            self.controller.stop()
            
            if False: #self.ball_center is None:
                self.searchBallBehaviour()
            elif False:
                self.freeSpaceBehaviour()
            else:
                pass
        rospy.spin()

        


if __name__ == "__main__":
    robotPlanner = RobotinhoPlanner()
    robotPlanner.initialize()
    robotPlanner.planningOnTheFly()
