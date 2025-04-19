# adapted from UGV given code and my old 1567 project


#!/usr/bin/env python
# encoding: utf-8
import sys, select, termios, tty, math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quanternion
from nav_msgs.msg import Odometry 



msg = ""
# not so fast! Don't want speed to go 0 -> 1 etc
A_SMOOTHER = 0.04
L_SMOOTHER = 0.008
# max speed, hard 1 for now. maybe a bit more efficent to 
A_SPEED = 1.0
L_SPEED = 1.0
# robot current position, we get these from robot odometry

class ugv_Move(Node):
    def __init__(self, name):
        # initalize
        super().__init__(name)
        # create ros2 publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        # like and subscribe!
        # odom gives robot positioning
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.x,self.y = 0,0
        self.degree = 0
        self.yaw = 0
        # create a Twist to send to robot
        self.twist = Twist()

    def odom_callback(self, msg):
        # Get the orientation of the robot
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w

        # Calculate the yaw of the robot
        # quaternion magic
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        
        # Store the distance and yaw of the robot
        self.distance = msg.pose.pose.position
        self.yaw = math.atan2(siny_cosp, cosy_cosp) 
    
    def move_the_robot(self, distance):
        if distance > .05:
            self.twist.linear.x = .2
        else:
            self.twist.linear.x = 0

    def turn_the_robot(self, turning_degree_in_rad):
        if 	self.yaw < turning_degree_in_rad:
            self.twist.angular.z = 0.3
        else:
            self.twist.angular.z = -0.3
        
        if abs(self.yaw - turning_degree_in_rad) < .05:
            self.twist.angular.z = 0

    def calculate_angle_and_distance(self, coordinate):
        x1 = coordinate[0]
        y1 = coordinate[1]

        angle = math.atan2(y1-self.y, x1-self.x)
        
        distance = math.sqrt(math.pow((x1-self.x),2) + math.pow((y1-self.y),2))

        return angle, distance

    

def main():
    # initialize ROS 2 
    rclpy.init()

    # initialize our mover
    ugv_mover = ugv_Move("path_scanner")

    coord_list = []
    wait_list = []

    # our path is predetermined, saved in a file of (x,y) coords
    # for nodes where we stop and scan, any other fun characters in there
    # to denote a stoppage
    with open("final_path.txt", "r") as file:
        line_count = 0
        for line in file:
            splat = line.split("()")
            x_str, y_str = splat[0].split(",")
            x1 = int(x_str.strip())
            y1 = int(y_str.strip())
            coord_list.append((x1,y1))
            if len(line.split("()")) > 1:
                 wait_list.append(line_count)
            line_count += 1

    # for every coord in the list, go to it!
    # basically turns then moves, always straight line
    # if the turns arent too sharp (aka not big gap)
    # it should be fairly smooth
    coord_count = 0
    for coord in coord_list:
        turning_rad, distance_to_move = ugv_mover.calculate_angle_and_distance(coord)
        # turn to coord
        while abs(ugv_mover.yaw - turning_rad) > .05:
            turning_rad, distance_to_move = ugv_mover.calculate_angle_and_distance(coord)
            ugv_mover.turn_robot(turning_rad)
            ugv_mover.pub.publish(ugv_mover.twist)
            # SLEEEEP
        # travel to coord
        while distance_to_move > 0.03:
            turning_rad, distance_to_move = ugv_mover.calculate_angle_and_distance(coord)
            # may need to adjust on the fly
            ugv_mover.turn_the_robot(turning_rad)
            ugv_mover.move_the_robot(distance_to_move)
            ugv_mover.pub.publish(ugv_mover.twist)
            # SLEEEEP
        ugv_mover.twist.linear.x = 0
        coord_count += 1
    
    # end of coord_list, stop bot
    ugv_mover.twist.linear.x = 0
    ugv_mover.twist.angular.z = 0
    ugv_mover.pub.publish(ugv_mover.twist)
    # destroy build destroy
    ugv_mover.destroy_node()
    # shutdown
    rclpy.shutdown()

if __name__ == "__main__":
    main()