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


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ECE_148_final(Node):
    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('ece_148_final')
        # create the publisher object
        self.path_pub = self.create_publisher(Path, 's', 10)
        # both subscribers need to be modified/fixed
        #self.perc_sub = self.create_subscription(LaserScan, '/scan', self.perception, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        #self.vel_pos_sub = self.create_subscription(LaserScan, '/scan', self.vel_pos, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        #insert original code under
        # ----------------------------------------------------------------------------------------------------------------------
        # IMPORT (should not change) -------------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        # top level path (module directory)
        # toppath = os.path.dirname(os.path.sssssssssssssssssssssssssssssssss                               
        toppath = "/home/projects/ros2_ws/src/ece_148_final/ece_148_final_pkg"                                                                 

        track_param = configparser.ConfigParser()
        if not track_param.read("/home/projects/ros2_ws/src/ece_148_final/ece_148_final_pkg/params/driving_task.ini"):
            raise ValueError('Specified online parameter config file does not exist or is empty!')

        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

        # define all relevant paths
        path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
                    'graph_store_path': toppath + "/inputs/stored_graph.pckl",
                    'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                    'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                    'log_path': toppath + "/logs/graph_ltpl/",
                    'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                    }

        # ----------------------------------------------------------------------------------------------------------------------
        # INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        # intialize graph_ltpl-class
        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,
                                                    visual_mode=False,
                                                    log_to_file=False)

        # calculate offline graph
        self.ltpl_obj.graph_init()

        # set start pose based on first point in provided reference-line
        refline = graph_ltpl.imp_global_traj.src.import_globtraj_csv.\
            import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]
        pos_est = refline[0, :]
        heading_est = np.arctan2(np.diff(refline[0:2, 1]), np.diff(refline[0:2, 0])) - np.pi / 2
        vel_est = 0.0

        # set start pos
        self.ltpl_obj.set_startpos(pos_est=pos_est,
                            heading_est=heading_est)

        #this is only for testing, obj_list_dummy replaced by our car
        # obj_list_dummy = graph_ltpl.testing_tools.src.objectlist_dummy.ObjectlistDummy(dynamic=True,vel_scale=0.3,s0=250.0)

        #used to store sensor message variables
        # self.obj_list = obj_list_dummy.get_objectlist()
        
        #end of original code
        
        self.path = Path()

    #def perception(self,msg):
        #this is the perception subscriber
    
    #def vel_pos(self,msg):
        #this is the velocity and position subscriber
        
    def send_path(self):
        
        self.path.header.frame_id = 'map'
        for row in self.traj_set[self.behavior][0]:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = row[1]
            pose_msg.pose.position.y = row[2]

            self.path.poses.append(pose_msg)

        self.path_pub.publish(self.path)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    ece_148_final = ECE_148_final()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(ece_148_final)
    # Explicity destroy the node
    ece_148_final.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
