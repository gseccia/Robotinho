#!/usr/bin/env python2
import rospy
from robotihnoMotionModule import RobotinhoController, Pose
from math import pow,sqrt,pi

class RobotinhoPlanner:
    def __init__(self):
        self.port = [ ((6.0,-1.0),(6.0,1.0)) , ((-6.0,-1.0),(-6.0,1.0))]
        # self.ball_position_subscriber = rospy.Subscriber("/ball_detector", )
        self.controller = RobotinhoController()
        self.ball_position = None # (0.0,0.0) # Real init None
        self.is_goal = False

    def genericLinePlanner(self,initialPosition,targetPosition):
        curve_lenght = sqrt(pow((targetPosition.x - initialPosition.x),2) + pow((targetPosition.y - initialPosition.y),2))
        p_s = []
        x = 0
        while x < curve_lenght:
            p_s.append(initialPosition + (targetPosition - initialPosition) * (x /curve_lenght))
            x += curve_lenght / 100
        return p_s

    def searchBallBehaviour(self):
        print("SEARCH BALL BEHAVIOUR")
        print("Rotate -pi/2")
        self.controller.rotate(pi/2)
        print("Rotate +pi/2")
        self.controller.rotate(-pi/2)
        
    def freeSpaceBehaviour(self):
        print("FREE SPACE BEHAVIOUR")
        self.controller.move2goal(Pose(self.ball_position[0] - 0.2,self.ball_position[1]))
        meanPortPose = Pose((self.port[0][0][0] + self.port[0][1][0])/2,(self.port[0][0][1] + self.port[0][1][1])/2)

        points = self.genericLinePlanner(self.controller.pose,meanPortPose)

        for point in points:
            print("Move to ",point)
            self.controller.move2goal(point)

        self.is_goal = True
        
    def obstacleAviodanceBehaviour(self):
        print("OBSTACLE AVOIDANCE BEHAVIOUR")

    def planningOnTheFly(self):
        while not rospy.is_shutdown() and not self.is_goal:
            if self.ball_position is None:
                self.searchBallBehaviour()
            elif False:
                self.obstacleAviodanceBehaviour()
            else: 
                self.freeSpaceBehaviour()
        rospy.spin()

        


if __name__ == "__main__":
    robotPlanner = RobotinhoPlanner()
    robotPlanner.planningOnTheFly()
