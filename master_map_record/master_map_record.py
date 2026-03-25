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


ROBOT_IP ="192.168.80.3"

def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument("username",help="Username for Spot")
    parser.add_argument("password",help="Password for Spot")
    parser.add_argument("map_dir",help="Directory where the map is stored on the robot")
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
    print("\n--- LEASE HANDOFF ---")
    print("On the Tablet, select 'HIJACK'. You should now have control.")
    input("Once you are driving the robot, press ENTER here to start recording...")

    # 3. Start Recording (The script can send commands even if it doesn't 'own' the body)
    recording_client.start_recording()
        
    print("RECORDING... Drive around the fiducial now.")
    time.sleep(30)

    # We use a loop to handle the 'NotReadyYetError'
    max_retries = 10
    for i in range(max_retries):
        try:
            recording_client.stop_recording()
            print("Recording stopped successfully.")
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
