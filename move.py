# adapted from UGV given code and my old 1567 project


#!/usr/bin/env python
# encoding: utf-8
import sys, select, time,termios, tty, math, threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 



msg = ""
# not so fast! Don't want speed to go 0 -> 1 etc
A_SMOOTHER = 0.04
L_SMOOTHER = 0.008
# max speed, hard 1 for now. maybe a bit more efficent to 

A_SPEED = .4
L_SPEED = .4
# robot current position, we get these from robot odometry

class ugv_Move(Node):
    def __init__(self, name):
        # initalize
        super().__init__(name)
        # create ros2 publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        # like and subscribe!
        # odom gives robot positioning
        self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.x,self.y = 0,0
        self.degree = 0
        self.yaw = 0
        self.make_coord_list()
        # create a Twist to send to robot
        self.twist = Twist()
        self.executor_thread = threading.Thread(target=self.move_bot)
        self.executor_thread.start()
    def move_bot(self):
   # basically turns then moves, always straight line
    # if the turns arent too sharp (aka not big gap)
    # it should be fairly smooth
        coord_count = 0
        for coord in self.coord_list:
            turning_rad, distance_to_move = self.calculate_angle_and_distance(coord)
        # turn to coord

            while abs(self.yaw - turning_rad) > .15:
                turning_rad, distance_to_move = self.calculate_angle_and_distance(coord)
                self.turn_the_robot(turning_rad)
                self.pub.publish(self.twist)
                time.sleep(0.1)
                print(f"goal:{coord}")
            # SLEEEEP
        # travel to coord
            print("passed turn")
            while distance_to_move > 0.10:
                turning_rad, distance_to_move = self.calculate_angle_and_distance(coord)
            # may need to adjust on the fly
                self.turn_the_robot(turning_rad)
                self.move_the_robot(distance_to_move)
                self.pub.publish(self.twist)
                time.sleep(0.1)
                print(f"goal:{coord}\tx:{self.x}\ty:{self.y}")
            print("passed move")

            if coord_count in self.wait_list:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
            self.twist.linear.x = 0.0
            coord_count += 1

    def make_coord_list(self):
        self.coord_list = []
        self.wait_list = []
    # our path is predetermined, saved in a file of (x,y) coords
    # for nodes where we stop and scan, any other fun characters in there
    # to denote a stoppage
        with open("/home/ws/cs1699-EcoBot/final_path.txt", "r") as file:
            line_count = 0
            for line in file:
                splat = line.strip("( )")
                x_str, y_str = splat.split(",")
                x1 = -1 * float(x_str.strip()) / 4.0
                y1 = float(y_str[:-1].strip("). ")) / 4.0
                self.coord_list.append((x1,y1))
                if len(line.split("()")) > 1:
                    self.wait_list.append(line_count)
                line_count += 1

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
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = math.atan2(siny_cosp, cosy_cosp) 
    
    def move_the_robot(self, distance):
        if distance > .05:

            self.twist.linear.x = L_SPEED
        else:
            self.twist.linear.x = 0.0

    def turn_the_robot(self, turning_degree_in_rad):
        if 	self.yaw < turning_degree_in_rad:

            self.twist.angular.z = A_SPEED
        else:
            self.twist.angular.z = -A_SPEED
        
        if abs(self.yaw - turning_degree_in_rad) < .15:
            self.twist.angular.z = 0.0

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
    rclpy.spin(ugv_mover)

    ugv_mover.pub.publish(ugv_mover.twist)
    # destroy build destroy
    ugv_mover.destroy_node()
    # shutdown
    rclpy.shutdown()

if __name__ == "__main__":
    main()
