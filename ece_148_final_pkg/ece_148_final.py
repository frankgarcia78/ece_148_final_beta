import rclpy
# import the ROS2 python libraries
from rclpy.node import Node

# TODO: confirm package name with team 2
#from planning_interface.msg import PathMsg, SensorMsg, PathObject, SensorObject # Dummy Objects
# from team02_interface.msg import TrackedObjects # Saved for later

from rclpy.qos import ReliabilityPolicy, QoSProfile

import sys
import os
os.environ['OPENBLAS_NUM_THREADS'] = str(1)
import numpy as np
import datetime
import json
import time
import configparser
import graph_ltpl

##from planning_interfaces.msg import SensorMsg
#from planning_interfaces.msg import SensorObject


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Exercise31(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('exercise31')
        # create the publisher object
        self.path_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # both subscribers need to be modified/fixed
        self.perc_sub = self.create_subscription(LaserScan, '/scan', self.perception, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.vel_pos_sub = self.create_subscription(LaserScan, '/scan', self.vel_pos, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def perception(self,msg):
        #this is the perception subscriber
    
    def vel_pos(self,msg):
        #this is the velocity and position subscriber
        
        

    def motion(self):
        # print the data
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # Logic of move
        if self.laser_forward > 5:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        elif self.laser_forward <5 and self.laser_forward>=0.5:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
         
            
            
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    exercise31 = Exercise31()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(exercise31)
    # Explicity destroy the node
    exercise31.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
