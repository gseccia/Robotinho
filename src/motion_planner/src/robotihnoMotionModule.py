#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi
from tf.transformations import euler_from_quaternion

class Pose():
    def __init__(self,x = 0,y = 0,theta = 0,vel = 0,w = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.linear_vel = vel
        self.radial_vel = w
    
    def __add__(self,point):
        return Pose(self.x + point.x,self.y + point.y,self.theta + point.theta, self.linear_vel + point.linear_vel, self.radial_vel + point.radial_vel )
    
    def __sub__(self,point):
        return Pose(self.x - point.x,self.y - point.y,self.theta - point.theta, self.linear_vel - point.linear_vel, self.radial_vel - point.radial_vel )

    def __mul__(self,value):
        return Pose(self.x * value,self.y * value,self.theta * value, self.linear_vel * value, self.radial_vel * value )
    
    def __str__(self):
        return 'Pose:\n\tx : {0}\n\ty : {1}\n\tz : {2}\n\tvel : {3}\n\tw : {4}\n'.format(self.x,self.y,self.theta,self.linear_vel,self.radial_vel)
    
    def __repr__(self):
        return str(self)

class RobotinhoController:
    def __init__(self):
        rospy.init_node('robotinho_controller', anonymous=True)
        
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/robot1/cmd_vel',Twist, queue_size=10)
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/robot1/odom',Odometry, self.update_pose)
        self.pose = Pose()
        self.odom = Odometry()
        self.rate = rospy.Rate(100000000)


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.pose.x = self.odom.pose.pose.position.x
        self.pose.y = self.odom.pose.pose.position.y
        roll,pitch,theta = euler_from_quaternion([self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w])
        self.pose.theta = theta # self.odom.pose.pose.orientation.z * pi # pi / (3*45*180)
        self.pose.linear_vel = self.odom.twist.twist.linear.x
        self.pose.radial_vel = self.odom.twist.twist.angular.z
        
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
  
    def linear_vel(self, goal_pose, constant=1.5):
        vel = constant * self.euclidean_distance(goal_pose)
        return vel if vel < 1.0 else 1.0
   
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
 
    def angular_vel(self, goal_pose, constant=6):
        steering_angle = self.steering_angle(goal_pose)
        w = constant * (steering_angle - self.pose.theta)
        if (pi/2 - 0.1 < abs(steering_angle) < pi/2 + 0.1  or abs(steering_angle) < 0.1) and abs(w) > 0.1:
            return w,True
        return w,False
 
    def move2goal(self,goal_pose):
        """Moves the turtle to the goal."""
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.1 # input("Set your tolerance: ")

        
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z,pure_rotation = self.angular_vel(goal_pose)
            
            if pure_rotation:
                vel_msg.linear.x = 0
            else:
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            print(self.pose)
            print(vel_msg)
            print(self.euclidean_distance(goal_pose))

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        print(self.pose)
        print(self.euclidean_distance(goal_pose))
    
    def rotate(self,deg):
        vel_msg = Twist()

        wz = 2*(deg - self.pose.theta)
        while abs(deg - self.pose.theta) > 0.01:
            vel_msg.linear.x = 0
            vel_msg.angular.z = wz
            self.velocity_publisher.publish(vel_msg)

            wz = 1.5*(deg - self.pose.theta)
            self.rate.sleep()
        
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == "__main__":
    robot = RobotinhoController()

    # Get the input from the user.
    goal_pose = Pose()

    while not rospy.is_shutdown():
        goal_pose.x = input("Set your x goal: ")
        goal_pose.y = input("Set your y goal: ")
        
        robot.move2goal(goal_pose)

        rospy.spin()
