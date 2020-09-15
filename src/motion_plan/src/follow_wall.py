#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import *

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
active_ = False


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_occupancy_grid(msg):
    global regions_
    occupancy_grid = msg.data
    max_value = 20
    regions_ = {
        'left': max_value,
        'fleft': min(max(0, max_value - int(occupancy_grid[1] * max_value)), max_value),
        'front': min(max(0, max_value - int(np.sum(occupancy_grid[0]) * max_value * 3)), max_value),
        'fright': min(max(0, max_value - int(occupancy_grid[2] * max_value)), max_value),
        'right': max_value
    }
    take_action()


def change_state(state):
    global state_, state_dict_, watchdog_
    if state is not state_:
        if state_ < 2:
            watchdog_ += 1
        else:
            watchdog_ = 0
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state


def take_action():
    global regions_, watchdog_, state_description_
    
    regions = regions_
    d = 15

    #CASO 1
    if regions['front'] >= d and regions['fleft'] >= d and regions['fright'] >= d:
        state_description_ = 'case 1 - nothing'  # nulla -> giro a dx
        change_state(0)
    #CASO 2
    elif regions['front'] <= d and regions['fleft'] >= d and regions['fright'] >= d:
        state_description_ = 'case 2 - front'  # ostacolo davanti -> giro a sx
        change_state(1)
    #CASO 3
    elif regions['front'] >= d and regions['fleft'] >= d and regions['fright'] <= d:
        state_description_ = 'case 3 - fright'  # ostacolo sulla destra -> seguo il muro
        change_state(2)
    #CASO 4
    elif regions['front'] >= d and regions['fleft'] <= d and regions['fright'] >= d:
        state_description_ = 'case 4 - fleft'  # ostacolo sulla sinistra -> giro a dx
        change_state(0)
    #CASO 5
    elif regions['front'] <= d and regions['fleft'] >= d and regions['fright'] <= d:
        state_description_ = 'case 5 - front and fright'  # centro-destra occupato -> giro a sx
        change_state(1)
    #CASO 6
    elif regions['front'] <= d and regions['fleft'] <= d and regions['fright'] >= d:
        state_description_ = 'case 6 - front and fleft'  # centro-sinistra occupato -> giro a sx
        change_state(1)
    #CASO 7
    elif regions['front'] <= d and regions['fleft'] <= d and regions['fright'] <= d:
        state_description_ = 'case 7 - front and fleft and fright'  # ostacolo davanti -> giro a sx
        change_state(1)
    #CASO 8
    elif regions['front'] >= d and regions['fleft'] <= d and regions['fright'] <= d:
        state_description_ = 'case 8 - fleft and fright'  # ostacolo dx e sx -> giro a sx
        change_state(2)
    else:
        state_description_ = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(regions)
    rospy.loginfo(state_description_)


def find_wall():
    msg = Twist()
    msg.angular.z = -1.8
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 2.0
    return msg


def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.4
    return msg


def main():
    global pub_, active_, state_description_, watchdog_

    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/visual_range_grid', Float32MultiArray, clbk_occupancy_grid)
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    watchdog_ = 0
    state_description_ = ''

    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        print(watchdog_)

        msg = Twist()
        if watchdog_ > 8:
            print('O SONO IN WATCHDOG')
            for i in range(30):
                if i > 20:
                    msg = follow_the_wall()
                else:
                    msg = turn_left()
                pub_.publish(msg)
                rate.sleep()
            watchdog_ = 0
        else:
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