"""
varying_height_test.py

python varying_height_test.py <USERNAME> <PASSWORD> --end_n <END_N>

This is a test script for the varying_height.py script, designed to validate the scripts function without accessing the robot hardware that the vary_height script runs on.
It allows us to test the logic using MagicMock to simulate the robot's client and responses.
This script changes the height of the robot.

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

from unittest.mock import MagicMock
from bosdyn.client import math_helpers

ROBOT_IP ="192.168.80.3"

# Create the params object
params = spot_command_pb2.MobilityParams()


# Example: setting a specific parameter like stairs mode
params.stair_hint = True

def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument('username',help='Spot Username')
    parser.add_argument('password',help='Spot Password')
    parser.add_argument('--end_n',type=int,help='Number of final step amount to perform', default=10)

    options=parser.parse_args(argv)
    

    #2. create sdk & authenticate
    #sdk = bosdyn.client.create_standard_sdk('RotatingMapExample')

    #create robot object since
    #robot=sdk.create_robot(ROBOT_IP)
    robot=MagicMock()
    #robot.authenticate(options.username,options.password)

    print("Authenticating...")
    #robot.time_sync.wait_for_sync()

    #3. create clients
    #lease_client=robot.ensure_client('lease')
    lease_client=MagicMock()
    #command_client=robot.ensure_client(RobotCommandClient.default_service_name)
    command_client=MagicMock()
    #robot_state_client=robot.ensure_client('robot-state')
    robot_state_client=MagicMock()
    # 1. Create the mock state client
    robot_state_client = MagicMock()
    
    # 2. Build a fake state object with a hardcoded battery float
    mock_state = MagicMock()
    mock_state.power_state.locomotion_charge_percentage.value = 95.00
    
    # 3. Tell the client to return this fake state when called
    robot_state_client.get_robot_state.return_value = mock_state

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
            '''
            #battery check, won't run if less than 20%
            if not check_batt_perc(robot_state_client,limit=20.0):
                print(f"\nBattery below 20%. Stopping at N={a}.")
                break
            '''

            print("\nStarting\n")
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


#Functions
def check_batt_perc(robot_state_client,limit=20.0):
    """
    Check battery percentage using protobuf path:
    state.power_state.locomotion_charge_percentage.value
    """
    state=robot_state_client.get_robot_state()

    #check if field exists
    if not state.power_state.HasField('locomotion_charge_percentage'):
        print("\nBattery percentage field not found, assuming sufficient charge\n")
        return True
    
    #Access .value 
    charge= state.power_state.locomotion_charge_percentage.value

    print(f"\nBattery check, charge: {charge:.2f}%\n")

    if charge < limit:
        return False
    return True

def control_height(command_client,height,robot_state_client):

    #0.0 is neutral, 0.1 is high, -0.1 is low, height in meters
    z_offset=height

    #build the pose (position+rotation), w=1 is neutral quaternion
    footprint_R_body=geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0.0,y=0.0,z=z_offset),
        rotation=geometry_pb2.Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
    )

    #create control parameters
    body_control=spot_command_pb2.BodyControlParams(
        base_offset_rt_footprint=footprint_R_body
    )

    #create mobility params and attach body control
    mobility_params=spot_command_pb2.MobilityParams(body_control=body_control)

    #build and send stand command  
    stand_cmd=RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    command_client.robot_command(stand_cmd)

    #wait for stabilization
    time.sleep(2.0)


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
