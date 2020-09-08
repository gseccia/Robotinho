#! /usr/bin/env python

import math

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import *
from tf import transformations

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
initial_position_ = None
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0


# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
    global position_, yaw_, initial_position_

    # position
    position_ = msg.pose.pose.position

    if initial_position_ is None:
        initial_position_ = position_

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_desired_position(msg):
    global desired_position_

    desired_position_ = msg.pose.pose.position


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


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)


def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_

    # here goes the equation
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance


def normalize_angle(angle):
    if (math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_

    print("Start Node")

    rospy.init_node('bug2')

    sub_occupancy_grip = rospy.Subscriber('/occupancy_grid', Float32MultiArray, clbk_occupancy_grid)
    sub_odom = rospy.Subscriber('/robot1/odom', Odometry, clbk_odom)

    sub_desPosition = rospy.Subscriber('/robot1/desired_position', Odometry, clbk_desired_position)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    # rospy.wait_for_service('/gazebo/set_model_state')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    # srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # set robot position
    # model_state = ModelState()
    # model_state.model_name = 'm2wr'
    # model_state.pose.position.x = initial_position_.x
    # model_state.pose.position.y = initial_position_.y
    # resp = srv_client_set_model_state(model_state)

    # initialize going to the point
    change_state(0)
    print("Start")

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if regions_ == None or initial_position_ == None:
            print("Regions         : ", regions_)
            print("Initial Position: ", initial_position_)
            continue
        print("State: ", state_)

        distance_position_to_line = distance_to_line(position_)

        if state_ == 0 and regions_['front'] < 15:
            change_state(1)
        elif state_ == 1 and count_state_time_ > 5 and distance_position_to_line < 1.0:
            change_state(0)

        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x,
                      position_.y)
        rate.sleep()


if __name__ == "__main__":
    main()