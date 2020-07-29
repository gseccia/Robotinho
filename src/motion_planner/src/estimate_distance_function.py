#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
from math import pow,sqrt
import time
import cv2
import json

class EstimateFunction:

    def __init__(self):
        rospy.init_node('estimator_node', anonymous=True)

        self.ball_position_sub = rospy.Subscriber("/football/bumper_states",ContactsState,self.realBallPositionUpdate)
        self.robot_position_sub = rospy.Subscriber("/robot1/odom",Odometry,self.robotPositionUpdate)
        self.detector_sub = rospy.Subscriber("/robot_ball_position",String,self.ballPositionUpdate)
        
        self.collector_data = []
        self.robot_pose = [0,0]
        self.ball_position = [0,0]

    def robotPositionUpdate(self, data):
        self.robot_pose[0] = data.pose.pose.position.x
        self.robot_pose[1]= data.pose.pose.position.y
        # roll,pitch,theta = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        # self.pose.theta = theta # self.odom.pose.pose.orientation.z * pi # pi / (3*45*180)

    
    def ballPositionUpdate(self,data):
        ball_dict = json.loads(data.data)
        if ball_dict["detected"]:
            x1 = ball_dict["x1"]
            x2 = ball_dict["x2"] 
            y1 = ball_dict["y1"] 
            y2 = ball_dict["y2"] 
            print("{0},{1} - {2},{3} ".format(x1,y1,x2,y2))
            area = (int(x2) - int(x1)) * (int(y2) - int(y1))
            distanza =  sqrt(pow((self.robot_pose[0] - self.ball_position[0]),2) + pow((self.robot_pose[1] - self.ball_position[1]),2))

            self.collector_data.append((area,distanza))
            
    
    def realBallPositionUpdate(self,real_pose):
        self.ball_position[0] = real_pose.states[0].contact_positions[0].x
        self.ball_position[1] = real_pose.states[0].contact_positions[0].y

    def save(self,filename):
        with open(filename,"w") as f:
            for data in self.collector_data:
                f.write(str(data[0])+" "+str(data[1])+"\n")
            f.close()

if __name__ == "__main__":
    estimator = EstimateFunction()
    k = 0
    while not rospy.is_shutdown() and k != 27:
        print("Collected data: ",len(estimator.collector_data))
        k = cv2.waitKey(100)
    
    estimator.save("./collection.txt")
    print("File saved!")