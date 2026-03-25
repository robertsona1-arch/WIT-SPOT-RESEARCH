"""
find_fid.py

python3 find_fid.py <USERNAME> <PASSWORD> <MASTER_MAP_DIR> <DISTANCE_IN_METERS> 

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
Last Updated: 3/24/2026
"""


from bosdyn.client.math_helpers import SE3Pose #new
from bosdyn.api import world_object_pb2 #new
from bosdyn.client.world_object import WorldObjectClient

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
    parser.add_argument("map_dir",help="Directory where the map is stored on the robot")
    parser.add_argument("dist",type=float,help="Distance in meters to stay in front of the tag")
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
    world_object_client = robot.ensure_client(bosdyn.client.world_object.WorldObjectClient.default_service_name)

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
        time.sleep(4)

        # Upload the map to the robot
        upload_map(graph_nav_client, options.map_dir)
        navigate_to_fiducial(robot,tag_id, distance_meters=options.dist)

#old one is stored in test file
"""
def navigate_to_fiducial(robot, tag_id, distance_meters=1.5):
    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    
    print(f"Attempting to localize to Fiducial ID: {tag_id}...")
    
    # 1. Create an empty Localization message
    # We don't need to guess where we are because the physical tag provides the absolute truth.
    empty_guess = nav_pb2.Localization()
    
    try:
        # 2. Use the Python Wrapper (Bypasses the namespace maze entirely)
        # fiducial_init=4 corresponds to the integer value of FIDUCIAL_INIT_SPECIFIC
        # use_fiducial_id takes the standard integer ID of your tag
        graph_nav_client.set_localization(
            initial_guess_localization=empty_guess,
            fiducial_init=4,
            use_fiducial_id=int(tag_id)
        )
        print("Localization successful.")
    except Exception as e:
        print(f"Localization failed: {e}")
        return False

    # 3. Define the goal pose 
    # +Z is 'Out' from the tag face. We rotate 180 degrees (pi) to face the tag. Have quat as 0 for back facing
    goal_pose = SE3Pose(x=0.0, y=0.0, z=distance_meters, rot=Quat.from_yaw(math.pi))

    # 4. Command the navigation using the Python wrapper arguments
    cmd_dir=100
    print("Navigating to front of tag...")
    graph_nav_client.navigate_to_anchor(
        seed_tform_goal=goal_pose.to_proto(),
        cmd_dir=cmd_dir,
        goal_waypoint_id=""
    )
    
    # 5. Monitor arrival (Basic polling)
    while True:
        status = graph_nav_client.get_localization_state().navigation_status
        # 1 equals STATUS_REACHED_GOAL
        if status == 1: 
            print("Arrived perfectly in front of fiducial.")
            break
        time.sleep(1.0)
    
    return True
"""

def navigate_to_fiducial(robot, tag_id, distance_meters=1.5):
    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    
    # 1. Localize (Snaps the Seed Frame to the Real World)
    print(f"Localizing to Fiducial ID: {tag_id}...")
    empty_guess = nav_pb2.Localization()
    try:
        graph_nav_client.set_localization(
            initial_guess_localization=empty_guess,
            fiducial_init=4,
            use_fiducial_id=int(tag_id)
        )
        print("Localization successful.")
    except Exception as e:
        print(f"Localization failed: {e}")
        return False

    # 2. Get Current State (Seed -> Body)
    localization_state = graph_nav_client.get_localization_state()
    seed_tform_body = SE3Pose.from_proto(localization_state.localization.seed_tform_body)

    # 3. Ask Perception for the Tag's location (Body -> Fiducial)
    world_objects = world_object_client.list_world_objects(
        object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
    ).world_objects

    fiducial_obj = next((obj for obj in world_objects if obj.apriltag_properties.tag_id == int(tag_id)), None)
    if not fiducial_obj:
        print("Error: Tag not currently visible to cameras.")
        return False

    body_tform_fiducial = get_a_tform_b(
        fiducial_obj.transforms_snapshot, 
        BODY_FRAME_NAME, 
        fiducial_obj.apriltag_properties.frame_name_fiducial
    )

    # 4. Execute the SE(3) Transform Chain
    # Map the fiducial into the global map frame
    seed_tform_fiducial = seed_tform_body * body_tform_fiducial
    
    # Define our offset in the Fiducial's local frame
    fiducial_tform_goal = SE3Pose(x=0.0, y=0.0, z=distance_meters, rot=Quat.from_yaw(math.pi))

    # Multiply to get the absolute map coordinate for the target
    seed_tform_goal = seed_tform_fiducial * fiducial_tform_goal

    """# 5. Navigate using the absolute map coordinate
    print("Navigating to calculated map coordinate...")
    try:
        graph_nav_client.navigate_to_anchor(
            seed_tform_goal=seed_tform_goal.to_proto(),
            cmd_duration=30.0
        )
    except Exception as e:
        print(f"Navigation rejected: {e}")
        return False

    # 6. Monitor arrival
    while True:
        status = graph_nav_client.get_localization_state().navigation_status
        if status == 1: 
            print("Arrived perfectly in front of fiducial.")
            break
        time.sleep(1.0)"""
    
    # Capture the navigation ID when you issue the command
    print("Navigating to target coordinate...")
    nav_id = graph_nav_client.navigate_to_anchor(
        seed_tform_goal=seed_tform_goal.to_proto(),
        cmd_duration=30.0
    )

    # Monitor the command using the correct feedback service
    print("Monitoring navigation status...")
    while True:
        # Ask the path planner about this specific command
        feedback = graph_nav_client.navigation_feedback(nav_id)
        
        if feedback.status == 1: # STATUS_REACHED_GOAL
            print("Arrived perfectly in front of fiducial.")
            break
        elif feedback.status == 2: # STATUS_LOST
            print("Error: Robot got lost. Check map alignment.")
            return False
        elif feedback.status == 3: # STATUS_STUCK
            print("Error: Robot got stuck. Check for physical obstacles.")
            return False
            
        time.sleep(1.0)
    
    return True


def upload_map(graph_nav_client, map_dir):
    # 1. Load and Upload the Graph (The Skeleton)
    with open(os.path.join(map_dir, "graph"), "rb") as f:
        graph_data = f.read()
    graph = map_pb2.Graph()
    graph.ParseFromString(graph_data)
    
    print("Uploading graph...")
    graph_nav_client.upload_graph(graph=graph)

    # 2. Upload Waypoint Snapshots (The Muscle)
    snapshot_dir = os.path.join(map_dir, "waypoint_snapshots")
    waypoint_ids = [wp.snapshot_id for wp in graph.waypoints]

    for snapshot_id in waypoint_ids:
        snapshot_path = os.path.join(snapshot_dir, snapshot_id)
        if os.path.exists(snapshot_path):
            with open(snapshot_path, "rb") as f:
                snapshot_data = f.read()
            
            snapshot = map_pb2.WaypointSnapshot()
            snapshot.ParseFromString(snapshot_data)
            
            print(f"Uploading snapshot: {snapshot_id[:8]}...")
            # Using the standard call; async is better for large maps
            graph_nav_client.upload_waypoint_snapshot(snapshot)

    print("Map upload complete.")

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)