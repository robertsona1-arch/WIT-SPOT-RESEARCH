"""
varying_height.py

python3 varying_height.py <USERNAME> <PASSWORD> --end_n <END_N>

THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
Minimal AI was used to aid in syntax and structure
"""


"""
Written by Geoffrey Siebert, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Adam Robertson
Date Created: 3/13/2026
Last Updated: 3/13/2026
"""

import argparse
import logging
import os
import sys
import time
import struct #added for ply conversion
import traceback
import math

import google.protobuf.timestamp_pb2
#import graph_nav_util
import grpc
from google.protobuf import wrappers_pb2 as wrappers

import bosdyn.client.channel 
import bosdyn.client.util
from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2
import bosdyn.client.graph_nav 
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient #check this
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2
from bosdyn.api.gps import gps_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2

#following not in sdk examples
import bosdyn.client
from bosdyn.client import map_processing
from bosdyn.client.robot import Robot

from bosdyn.client.lease import LeaseKeepAlive
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_se2_a_tform_b
# 1. CLIENTS (The "Doing" part)

#from bosdyn.client.graph_nav_recording import GraphNavRecordingClient # Standalone in 5.x
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.api import geometry_pb2
from bosdyn.client.map_processing import MapProcessingServiceClient

# 2. APIS/PROTOS (The "Data" part)
from bosdyn.api import robot_command_pb2 as generic_robot_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

from bosdyn.client import math_helpers

ROBOT_IP ="192.168.80.3"
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import check_batt_perc, control_height

def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument('username',help='Spot Username')
    parser.add_argument('password',help='Spot Password')
    parser.add_argument('--end_n',type=int,help='Number of final step amount to perform', default=10)
    

    options=parser.parse_args(argv)
    

    #2. create sdk & authenticate
    sdk = bosdyn.client.create_standard_sdk('RotatingMapExample')

    #create robot object since
    robot=sdk.create_robot(ROBOT_IP)
    robot.authenticate(options.username,options.password)

    print("Authenticating...")
    robot.time_sync.wait_for_sync()

    #3. create clients
    lease_client=robot.ensure_client('lease')
    # recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
    # graph_nav_client=robot.ensure_client(GraphNavClient.default_service_name)
    command_client=robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state_client=robot.ensure_client('robot-state')
    # map_processing_client = robot.ensure_client(MapProcessingServiceClient.default_service_name)


    if not robot.is_powered_on():
        print("\n robot is powered off, exiting...\n")
        sys.exit(1)

    #4. acquire lease & execution

    #forcefully take the lease:
    lease_client.take()
    with LeaseKeepAlive(lease_client, must_acquire=False, return_at_exit=True):
        print("\nbeginning\n")
        time.sleep(2)

        #Command the robot to stand
        print("\nCommanding robot to stand...\n")
        stand=RobotCommandBuilder.synchro_stand_command()
        command_client.robot_command(stand)
        time.sleep(3)

        
        for a in range(1, options.end_n + 1):
            control_height(command_client,-0.1,robot_state_client)
            #battery check, won't run if less than 20%
            if not check_batt_perc(robot_state_client,limit=20.0):
                print(f"\nBattery below 20%. Stopping at N={a}.")
                break
                
            fold_name=f"test_n_{a:02d}"
            full_path=os.path.join(options.map_dir,fold_name)


            if not os.path.exists(full_path):
                os.makedirs(full_path)
                
            print("\nStarting...\n")
            time.sleep(0.1)
            low=-0.1
            high=0.1
            step=(high-low)/a
            cur_h=low
            for b in range(a):
                if a!=1:
                    time.sleep(3)#need to have this so it goes on when its ready
                    print("\nCreating Waypoint\n")
                    time.sleep(3)
                    cur_h+=step
                    control_height(command_client,cur_h,robot_state_client)
                    time.sleep(2)
                else:
                    time.sleep(3)#need to have this so it goes on when its ready
                    print("\nCreating Waypoint\n")
                    time.sleep(3)

            time.sleep(0.5)
            
    print("\nScript finished\n")

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)