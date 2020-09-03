#! /usr/bin/env python

# import ros stuff
import rospy
#from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import numpy as np
import math


pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
active_ = True

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_occupancy_grid(msg):
    global regions_
    occupancy_grid = np.reshape(msg.data,(3,5))
    max_value=20
    regions_ = {
        'right':  max_value,
        'fright': min(max(0, max_value-int(occupancy_grid[1][4]*max_value)), max_value),
        'front':  min(max(0, max_value-int(np.sum(occupancy_grid[2][:])*max_value)), max_value),
        'fleft':  min(max(0, max_value-int(occupancy_grid[1][0]*max_value)), max_value),
        'left':   max_value,
    }
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 15

    if regions['front'] >= d and regions['fleft'] >= d and regions['fright'] >= d:
        state_description = 'case 1 - nothing' #nulla -> giro a dx
        change_state(0)
    elif regions['front'] <= d and regions['fleft'] >= d and regions['fright'] >= d:
        state_description = 'case 2 - front' #ostacolo davanti -> giro a sx
        change_state(1)
    elif regions['front'] >= d and regions['fleft'] >= d and regions['fright'] <= d:
        state_description = 'case 3 - fright' #ostacolo sulla destra -> seguo il muro
        change_state(2)
    elif regions['front'] >= d and regions['fleft'] <= d and regions['fright'] >= d:
        state_description = 'case 4 - fleft' #ostacolo sulla sinistra -> giro a dx
        change_state(0)
    elif regions['front'] <= d and regions['fleft'] >= d and regions['fright'] <= d:
        state_description = 'case 5 - front and fright' #centro-destra occupato -> giro a sx
        change_state(1)
    elif regions['front'] <= d and regions['fleft'] <= d and regions['fright'] >= d:
        state_description = 'case 6 - front and fleft' #centro-sinistra occupato -> giro a sx
        change_state(1)
    elif regions['front'] <= d and regions['fleft'] <= d and regions['fright'] <= d:
        state_description = 'case 7 - front and fleft and fright'  #ostacolo davanti -> giro a sx
        change_state(1) 
    elif regions['front'] >= d and regions['fleft'] <= d and regions['fright'] <= d:
        state_description = 'case 8 - fleft and fright' #ostacolo dx e sx -> giro a sx
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(regions)
    rospy.loginfo(state_description)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.6
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.6
    return msg


def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.2
    return msg


def main():
    global pub_, active_

    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/occupancy_grid', Float32MultiArray, clbk_occupancy_grid)
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)


    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
