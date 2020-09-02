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
        self.desidered_velocity = Pose()
        self.odom = Odometry()
        self.rate = rospy.Rate(100000000)

        self.init_complete = False
    
    def isStuck(self):
        return False # self.pose.linear_vel < 0.1 and self.desidered_velocity.linear_vel > 0.0 and self.pose.radial_vel < 0.1

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
        
        self.stopTrigger = False
        self.init_complete = True
    
    def forceStop(self):
        self.stopTrigger = True

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
        if (pi/2 - 0.1 < abs(steering_angle) < pi/2 + 0.1  or pi - 0.1 <= abs(steering_angle) <= pi + 0.1) and abs(w) > 0.1:
            return w,True
        return w,False

    def basicMove2tile(self,goal_pose):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Tolleranza
        print("ROTATE    : ",self.steering_angle(goal_pose) * 180 / pi)

        self.rotate(self.steering_angle(goal_pose))

        print("END ROTATE: ",self.pose.theta * 180 / pi)
        
        distance_tolerance = 0.1
        self.stopTrigger = False
        initPose = Pose(self.pose.x,self.pose.y,self.pose.theta)
        velocity = sqrt( pow(goal_pose.x - initPose.x,2) + pow(goal_pose.y - initPose.y,2))
        velocity = velocity if velocity > 0.2 else 0.2       

        euclidean_distance = self.euclidean_distance(goal_pose)
        cycles = 10000
        totalTime = 0

        print("DISTANCE : ",euclidean_distance)
        while euclidean_distance >= distance_tolerance and not self.stopTrigger:
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            
            vel_msg.linear.x = velocity
            
            # print("Pure Rotation   : ",pure_rotation)
            
            # print("Velocity        : ",vel_msg.linear.x)
            # print("Angular Velocity: ",vel_msg.linear.z)
            # print(self.pose)
            self.desidered_velocity.linear_vel = vel_msg.linear.x
            self.desidered_velocity.radial_vel = vel_msg.angular.z
            
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

            totalTime += self.rate.sleep_dur.to_nsec()

            euclidean_distance = self.euclidean_distance(goal_pose)

            if totalTime > self.rate.sleep_dur.to_nsec() * cycles:
                self.stopTrigger = True
                self.stop()
                print("Force to STOP")
        print("DISTANCE REACHED : ",euclidean_distance)


    def move2tile(self,goal_pose):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Tolleranza
        distance_tolerance = 0.1
        self.stopTrigger = False
        initPose = Pose(self.pose.x,self.pose.y,self.pose.theta)
        velocity = sqrt( pow(goal_pose.x - initPose.x,2) + pow(goal_pose.y - initPose.y,2))
        velocity = velocity if velocity > 0.75 else 0.75


        euclidean_distance = self.euclidean_distance(goal_pose)
        cycles = 10000
        totalTime = 0
        


        while euclidean_distance >= distance_tolerance and not self.stopTrigger:
            last_euclidean = euclidean_distance

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z,pure_rotation = self.angular_vel(goal_pose)
            
            if pure_rotation:
                vel_msg.linear.x = 0
                vel_msg.angular.z = vel_msg.angular.z if vel_msg.angular.z < 1.0 else 1.0
            else:
                # Linear velocity in the x-axis.
                vel_msg.linear.x = velocity
            
            # print("Pure Rotation   : ",pure_rotation)
            
            # print("Velocity        : ",vel_msg.linear.x)
            # print("Angular Velocity: ",vel_msg.linear.z)
            # print(self.pose)
            self.desidered_velocity.linear_vel = vel_msg.linear.x
            self.desidered_velocity.radial_vel = vel_msg.angular.z

            print("Linear  Velocity: ",velocity)
            print("Angular Velocity: ",vel_msg.angular.z)
            print("Distance        : ",euclidean_distance)
                        
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # print(self.pose)
            # print(vel_msg)
            # print(self.euclidean_distance(goal_pose))

            # Publish at the desired rate.
            self.rate.sleep()

            totalTime += self.rate.sleep_dur.to_nsec()

            euclidean_distance = self.euclidean_distance(goal_pose)

            if totalTime > self.rate.sleep_dur.to_nsec() * cycles:
                self.stopTrigger = True
                self.stop()
                print("Force to STOP")


        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal(self,goal_pose):
        """Moves the robot to the goal."""
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Tolleranza
        distance_tolerance = 0.2
        self.stopTrigger = False

        
        while self.euclidean_distance(goal_pose) >= distance_tolerance and not self.stopTrigger:
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

            # print(self.pose)
            # print(vel_msg)
            # print(self.euclidean_distance(goal_pose))

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # print(self.pose)
        # print(self.euclidean_distance(goal_pose))
    
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
    
    def stop(self):
        vel_msg = Twist()
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
