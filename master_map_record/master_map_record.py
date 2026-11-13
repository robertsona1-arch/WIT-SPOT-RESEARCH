"""
master_map_record.py

mac - python3 master_map_record.py <USERNAME> <PASSWORD> --dir "DIRECTORY" --int <NUM> --float <FLOAT>

windows - python master_map_record.py <USERNAME> <PASSWORD> --dir "DIRECTORY" --int <NUM> --float <FLOAT>

This script is used to record a new master map of the environment. 
Start with the tablet holding the lease. Run this script, then select hijack on tablet. 
Select walk, then press enter on the script to start the 30s of recording. After 30s, the script will stop recording and download the map to the specified directory.

Use ctrl+c to stop the script any time, the robot will stop and sit safetly, and any completed maps will be saved
Please maintian a safe distance from the robot at all time
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET READY TO STOP THE ROBOT IF NEEDED

This script pulls significant portions of code from the Boston Dynamics <blank> files and Google API <blank> files

Minimail AI was used in syntax and structure
"""

"""
Written by Adam Robertson, Wentworth Institude of Technology, Douglas D. Schumman School of Engineerning
WIT SPOT Research Group
Advisors: Prof. Tahmid Latif, Prof. Afsaneh Ghanavati
Contributors: 
Date Created: 02/28/2026
Last Updated: 11/12/2026
"""

import time
import os
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.math_helpers import SE3Pose #new
from bosdyn.api import world_object_pb2 #new

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
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
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
from requests import options

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import check_batt_perc 

ROBOT_IP ="192.168.80.3"

def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument('username',help='Spot Username')
    parser.add_argument('password',help='Spot Password')
    parser.add_argument('--map_dir', help='Directory example')
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
    recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
    graph_nav_client=robot.ensure_client(GraphNavClient.default_service_name)
    command_client=robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state_client=robot.ensure_client('robot-state')
    map_processing_client = robot.ensure_client(MapProcessingServiceClient.default_service_name)
    # In section #3: create clients
    world_object_client = robot.ensure_client(bosdyn.client.world_object.WorldObjectClient.default_service_name)
   
    # 1. Start the script. It will TAKE the lease.
    lease_client.take()
    with LeaseKeepAlive(lease_client, must_acquire=False, return_at_exit=True):
        # Power and Stand
        robot.power_on()
        command_client.robot_command(RobotCommandBuilder.synchro_stand_command())
        time.sleep(3)

    # 2. THE FIX: Give the lease BACK to the tablet temporarily
    print("\n--- LEASE HANDOFF ---\n")
    print("\nOn the Tablet, select 'HIJACK'. You should now have control.\n")
    input("\nOnce you are driving the robot, press ENTER here to start recording...\n")

    # 3. Start Recording (The script can send commands even if it doesn't 'own' the body)
    recording_client.start_recording()
        
    print("\nRECORDING... Drive around the fiducial now.\n")

    
    time.sleep(30)

    # We use a loop to handle the 'NotReadyYetError'
    max_retries = 10
    for i in range(max_retries):
        try:
            recording_client.stop_recording()
            print("Recording stopped successfully.")
            print("\nRecording stopped. Downloading...\n")
            break # Exit the loop if it works
        except bosdyn.client.recording.NotReadyYetError:
            print(f"Robot is still processing map (Attempt {i+1}/{max_retries}). Waiting 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"Critical error stopping recording: {e}")
            break       
    print("Recording stopped. Downloading...")
    graph_nav_client.write_graph_and_snapshots(options.map_dir)

    print("Stopping recording...")
    
if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
