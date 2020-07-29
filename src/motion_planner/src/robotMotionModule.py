#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import numpy as np
import sys
import roslib


topic = '/robot1/cmd_vel'



def brutal_planner(point,tf = 2.0):
    Ax = [[0,0,0,1],[tf*tf*tf,tf*tf,tf,1],[0,0,1,0],[3*tf*tf,2*tf,1,0]]
    px = [current_x,point.x,0,0]
    coef_x = np.dot(np.linalg.pinv(Ax),px)

    Ay = [[0,0,0,1],[tf*tf*tf,tf*tf,tf,1],[0,0,1,0],[3*tf*tf,2*tf,1,0]]
    py = [current_y,point.y,0,0]
    coef_y = np.dot(np.linalg.pinv(Ay),py)

    At = [[0,0,0,1],[tf*tf*tf,tf*tf,tf,1],[0,0,1,0],[3*tf*tf,2*tf,1,0]]
    pt = [current_theta,0,0,0]
    coef_t = np.dot(np.linalg.pinv(At),pt)

    t = 0
    sampling_time = tf / 100
    while t < tf:
        vx = 3*coef_x[0]*t*t + 2*coef_x[1]*t + coef_x[2]
        vy = 3*coef_y[0]*t*t + 2*coef_y[1]*t + coef_y[2]
        
        v = math.sqrt(vx*vx + vy*vy)
        w = 3*coef_t[0]*t*t + 2*coef_t[1]*t + coef_t[2]

        print(v,w)
        sendCommand(v,w)
        t += sampling_time
        time.sleep(sampling_time)




def reach(point,Kp = 0.1 ,Ki = 0.01 , Kd = 0.1,th = 0.01):
    cumulative_e = 0
    last_e = 0
    eps = sys.float_info.epsilon
    v = 1.0

    dx = point.x - current_x
    dy = point.y - current_y
    desid_z = math.atan2(dy,(dx + eps))

    print("POINT: ",point.x," ",point.y)
    print("DZ: ",desid_z)

    while abs(dx) > th or abs(dy) > th:
        print("DISTANCE ",dx," ",dy)
        
        alpha = desid_z  - current_theta * (3*45) * math.pi /180
        e = math.atan2(math.sin(alpha),math.cos(alpha)+ eps)

        e_P = e
        e_I = cumulative_e + e
        e_D = e - last_e

        w = Kp*e_P + Ki*e_I + Kd*e_D

        w = math.atan2(math.sin(w),math.cos(w)+ eps)
        v = math.sqrt(dx*dx+dy*dy)

        if v > 2.0:
            v = 2.0
        elif v < -2.0:
            v = -2.0
        print("SPEED ",v," ",w)

        cumulative_e += e
        last_e = e

        dx = point.x - current_x
        dy = point.y - current_y
        desid_z = math.atan2(dy,(dx + eps))

        sendCommand(v,w)
        time.sleep(1)

if __name__=="__main__":
    rospy.init_node('control_module')

    pub = rospy.Publisher(topic, Twist, queue_size=10)
    sub = rospy.Subscriber('/robot1/odom',Odometry,readFromSensor)

    initComplete = False

    while not initComplete:
        pass
    
    stopRobot()
    try:
        print "RuotoBene"
        moveTo(0.0,0.0)
        # forwardMove(0,0)
        
        #sendCommand(0,1)
        # goAhead(1)
        

    except Exception as e:
        print e

    finally:
        stopRobot()

    rospy.spin()
