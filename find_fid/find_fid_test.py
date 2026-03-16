"""
find_fid_test.py

python3 find_fid_test.py <USERNAME> <PASSWORD> <DISTANCE_IN_METERS> 

Use ctrl+c to stop the script at any time, the robot will stop and sit safely, and any completed maps will be saved

This script navigates and faces the robot infront of a fiducial 
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
This script pulls significant portions of code from the Boston Dynamics recording_command_line.py & view_map.py 
Minimal AI was used to aid in syntax and structure
"""


"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert
Date Created: 3/16/2026
Last Updated: 3/16/2026
"""


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

ROBOT_IP ="192.168.80.3"
tag_id=1

def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument("username",help="Username for Spot")
    parser.add_argument("password",help="Password for Spot")
    parser.add_argument("dist",type=int,help="Distance in meters to stay in front of the tag")
    options=parser.parse_args(argv)

    #2. create sdk & authenticate
    #sdk = bosdyn.client.create_standard_sdk('RotatingMapExample')

    #create robot object since
    #robot=sdk.create_robot(ROBOT_IP)
    #robot.authenticate(options.username,options.password)
    robot=MagicMock()

    print("Authenticating...")
    #robot.time_sync.wait_for_sync()

    #3. create clients
    #lease_client=robot.ensure_client('lease')
    lease_client=MagicMock()
    #recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
    recording_client = MagicMock()
    #graph_nav_client=robot.ensure_client(GraphNavClient.default_service_name)
    graph_nav_client=MagicMock()
    #command_client=robot.ensure_client(RobotCommandClient.default_service_name)
    command_client=MagicMock()
    #robot_state_client=robot.ensure_client('robot-state')
    robot_state_client=MagicMock()
    #map_processing_client = robot.ensure_client(MapProcessingServiceClient.default_service_name)
    map_processing_client = MagicMock()

    # 2. Build a fake state object with a hardcoded battery float
    mock_state = MagicMock()
    mock_state.power_state.locomotion_charge_percentage.value = 95.00

    # 3. Tell the client to return this fake state when called
    robot_state_client.get_robot_state.return_value = mock_state

    lease_client.take()
    with LeaseKeepAlive(lease_client, must_acquire=False, return_at_exit=True):
        print("\nbeginning\n")
        #clear estops and power on motors locally
        robot.time_sync.wait_for_sync()
        if not robot.is_powered_on():
            print("\nPowering on leo\n")
            robot.power_on(timeout_sec=20)

        #Command the robot to stand
        print("\nCommanding robot to stand...\n")
        stand=RobotCommandBuilder.synchro_stand_command()
        command_client.robot_command(stand)
        time.sleep(2)

        navigate_to_fiducial(robot, options.tag_id, distance_meters=options.dist)

def navigate_to_fiducial(robot, tag_id, distance_meters=1.5):
    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    
    # 1. Tell the robot to localize specifically to the fiducial
    # This 'snaps' the robot's internal map to the physical tag
    print(f"Attempting to localize to Fiducial ID: {tag_id}...")
    
    # We use an INITIAL_GUESS_FIDUCIAL to force a scan
    localize_req = graph_nav_pb2.SetLocalizationRequest(
        initial_guess=graph_nav_pb2.SetLocalizationRequest.INITIAL_GUESS_FIDUCIAL,
        fiducial_init=graph_nav_pb2.SetLocalizationRequest.FiducialInit(
            frame_name_fiducial=f"fiducial_{tag_id}"
        )
    )
    
    try:
        graph_nav_client.set_localization(localize_req)
        print("Localization successful.")
    except Exception as e:
        print(f"Localization failed: {e}")
        return False

    # 2. Define the goal pose RELATIVE to the fiducial
    # In the AprilTag frame: 
    # +Z is 'Out' from the tag. 
    # To face the tag, we want to be at +Z distance and rotate 180 degrees (pi radians)
    goal_pose_in_fiducial_frame = SE3Pose(x=0, y=0, z=distance_meters, 
                                          rot=math_helpers.Quat.from_yaw(math.pi))

    # 3. Command the navigation
    # We use navigate_to_anchor to move to a specific SE3Pose in the map
    nav_to_cmd = graph_nav_pb2.NavigateToAnchorRequest(
        seed_tform_goal=goal_pose_in_fiducial_frame.to_proto(),
        goal_waypoint_id="", # Leaving empty because we are using an anchor (global coord)
        command_id=1
    )
    
    graph_nav_client.navigate_to_anchor(nav_to_cmd)
    
    # 4. Monitor arrival (Basic polling)
    print("Navigating to front of tag...")
    while True:
        status = graph_nav_client.get_localization_state().navigation_status
        if status == graph_nav_pb2.Iterable_NavStatus.STATUS_REACHED_GOAL:
            print("Arrived perfectly in front of fiducial.")
            break
        time.sleep(1.0)
    
    return True