"""
varying_ht_mp_fid.py

python3 varying_ht_mp_fid.py <USERNAME> <PASSWORD> --map_dir <DIRECTORY> --mast_dir <MASTER_MP_DIR> --dist <DIST_IN_M> --end_n <END_N>
use 3.5 for dist

This script records maps with the robot standing. The amount of snapshots per map will increase by 2 starting from 1.
The robot will navigate to a fiducial before each recording, and the map will be saved in a separate folder for each N. The master map should be a recording of the robot standing still in front of the fiducial with 1 snapshot, and it will be used to find the fiducial and navigate to it for all subsequent recordings. The master map should be recorded with the robot at the same height as the first recording (N=1) for best results, but the script will attempt to find the fiducial even if there are height differences. 
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
This script pulls significant portions of code from the Boston Dynamics recording_command_line.py & view_map.py 
Minimal AI was used to aid in syntax and structure
"""


"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert
Date Created: 3/24/2026
Last Updated: 4/1/2026
"""

from bosdyn.client.math_helpers import SE2Pose #new
from bosdyn.api import world_object_pb2, basic_command_pb2#new
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
from bosdyn.api import geometry_pb2, trajectory_pb2


ROBOT_IP ="192.168.80.3"
tag_id=1

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import check_batt_perc, convert_map_to_ply,control_height,nav_to_fid,fine_align,upload_map, log_test_metrics

def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser = argparse.ArgumentParser()
    # Positional Arguments (Terminal order matters)
    parser.add_argument('username', help='Spot Username')
    parser.add_argument('password', help='Spot Password')
    
    # Optional/Named Arguments (Requires terminal flags)
    parser.add_argument('--map_dir', help='Directory to save maps to', required=True)
    parser.add_argument('--mast_dir', help='Directory where the map is stored on the robot', required=True)
    parser.add_argument('--dist', type=float, help='Distance in meters', required=True)
    parser.add_argument('--end_n',type=int,help='end n',required=False)
    

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

    #create directory
    if not os.path.exists(options.map_dir):
        os.makedirs(options.map_dir)


    #4. acquire lease & execution

    #forcefully take the lease:
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
        time.sleep(3)

        # Upload the map to the robot
        upload_map(graph_nav_client, options.mast_dir)
        nav_to_fid(robot,tag_id, dist_m=options.dist)
        dist_error_m, final_yaw_error = fine_align(robot, tag_id, options.dist, iter=100)

        for a in range(1, options.end_n + 1):
            start_time = time.time()
            control_height(command_client,-0.1,robot_state_client)
            #battery check, won't run if less than 20%
            if not check_batt_perc(robot_state_client,limit=20.0):
                print(f"\nBattery below 20%. Stopping at N={a}.")
                break
                
            fold_name=f"test_n_{a:02d}"
            full_path=os.path.join(options.map_dir,fold_name)


            if not os.path.exists(full_path):
                os.makedirs(full_path)
                
            #graph_nav_client.clear_graph() got error saying call stop recording first
            recording_client.start_recording()
            print("\nStarting Recording\n")
            time.sleep(0.1)
            low=-0.1
            high=0.1
            step=(high-low)/a
            cur_h=low
            for b in range(a):
                if a!=1:
                    #snapshot
                    recording_client.create_waypoint(waypoint_name=f"N{a}_Snap{b+1}")
                    #time.sleep(3)#need to have this so it goes on when its ready
                    print("\nCreating Waypoint\n")
                    cur_h+=step
                    control_height(command_client,cur_h,robot_state_client)
                    time.sleep(2)
                else:
                    #snapshot
                    recording_client.create_waypoint(waypoint_name=f"N{a}_Snap{b+1}")
                    time.sleep(3)#need to have this so it goes on when its ready
                    print("\nCreating Waypoint\n")
                    time.sleep(3)

            #stop and download
            recording_client.stop_recording()
            time.sleep(0.5)
                
            # Use the module-level helper, passing the directory and the client
            graph_nav_client.write_graph_and_snapshots(full_path)

            #convert
            print(f"\n[N{a}]Converting to ply...\n")
            ply_name=os.path.join(full_path,f"converted_n_{a}.ply")
            convert_map_to_ply(full_path,ply_name,a)
            graph_nav_client.clear_graph()
            end_time = time.time()
            duration_secs = end_time - start_time
            log_test_metrics(map_dir=options.map_dir, test_name=fold_name, durations_secs=duration_secs, dist_error_m=dist_error_m, yaw_error_deg=math.degrees(final_yaw_error))
            
    print("\nScript finished\n")

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)