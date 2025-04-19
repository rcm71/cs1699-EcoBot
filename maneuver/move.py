# adapted from UGV given code and my old 1567 project


#!/usr/bin/env python
# encoding: utf-8
import sys, select, termios, tty, math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = ""
A_SMOOTHER = 0.04
L_SMOOTHER = 0.008

class movement: #used for both linear and angular
	def __init__(self, curr, limit, goal):
		self.curr = curr
		self.limit = limit
		self.goal = goal

def calculate_angle_and_distnace(coordinate):
    global x,y
    x1 = coordinate[0]
    y1 = coordinate[1]
#    print(x1,y1)
    angle = math.atan2(y1-y, x1-x)
    
    distance = math.sqrt(math.pow((x1-x),2) + math.pow((y1-y),2))

    return angle, distance

def turn_the_robot(turning_degree_in_rad):

    global twist, yaw
    if 	yaw < turning_degree_in_rad:
        twist.angular.z = 0.3
    else:
	    twist.angular.z = -0.3   
    if abs(yaw - turning_degree_in_rad) < .05:
	    twist.angular.z = 0
         
def move_the_robot(distance):
    print("moving...")
    #code
    global twist, x
    if distance > .05:
	    twist.linear.x = .2
    else:
	    twist.linear.x = 0
    

class ugv_Move(Node):
    def __init__(self, name):
        # initalize
        super().__init__(name)
        # create ros2 publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        # declare speed limits
        self.declare_parameter("linear_speed_limit",1.0)
        self.declare_parameter("angular_speed_limit",1.0)
		# Get the parameter values
        self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value

    

def main():
    # initialize ROS 2 
    rclpy.init()

    # initialze our smoother objects
    angular = movement(0,1,0) # max speed 1.0
    linear  = movement(0,1,0)

    # initialize our mover
    ugv_mover = ugv_Move("path_scanner")

    # set initial values for speed/turn
    xspeed_switch = True
    (speed, turn) = (0.2, 0.5)

    # set initial positioning
    (x, th) = (0,0)

    #set intial status
    status = 0
    stop = False
    count = 0

    # create a Twist
    twist = Twist()
    try:
          
          #linear smoothing
        linear_diff = abs(linear.goal - linear.curr)
            #floats suck. if close, it's there
        if linear_diff <= 0.001:
                linear.curr = linear.curr
                if linear.goal == 0:
                    linear.curr = 0
            # wont stop on 0 eek, was going -.0000000006
        elif (linear.curr < linear.goal):#not there yet
            linear.curr += commands[6]*L_SMOOTHER
            #array is mode, eco=1, sport=2 (sport double fast!)
        elif(linear.curr > linear.goal):#slow down!
            linear.curr -= commands[6]*L_SMOOTHER
        #again? yes.        
        linear_diff = abs(linear.goal - linear.curr)
            if linear_diff <= 0.001:
                linear.curr = linear.curr
                if linear.goal == 0:
                    linear.curr = 0# wont stop on 0 eek
    twist.angular.z =
    twist.linear.x = 

            ugv_mover.pub.publish(twist)

    except Exception as e: print(e)

    # destroy build destroy
    ugv_mover.destroy_node()
    # shutdown
    rclpy.shutdown()