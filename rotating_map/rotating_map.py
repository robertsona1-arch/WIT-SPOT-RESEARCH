"""
rotating_map.py

python3 rotating_map.py <USERNAME> <PASSWORD> <DIRECTORY> <START_N> <OPTION_END_N>

This script records a map with the robot making N turns
It will begin with <START_N> turns, then increments by factors of 360 until it reaches the battery check
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
This script pulls significant portions of code from the Boston Dynamics recording_command_line.py & view_map.py 
Minimal AI was used to aid in syntax and structure
"""


"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert
Date Created: 1/26/2026
Last Updated: 2/25/2026
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


def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument('username',help='Spot Username')
    parser.add_argument('password',help='Spot Password')
    parser.add_argument('map_dir',help='Directory to save maps to')
    parser.add_argument('start_n',type=int,help='Number of initial rotations to perform')

    #optional end N
    parser.add_argument('--end_n',type=int,help='Number of maximum rotations to perform',default=8)

    options=parser.parse_args(argv)
    if options.start_n<1:
        options.start_n=1

    if options.end_n<options.start_n:
        options.end_n=options.start_n-1
    

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


    #create directory
    if not os.path.exists(options.map_dir):
        os.makedirs(options.map_dir)

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

        for a in range(options.start_n, options.end_n+1):
            #battery check, won't run if less than 20%
            if not check_batt_perc(robot_state_client,limit=20.0):
                print(f"\nBattery below 20%. Stopping at N={a}.")
                break
            
            if 360 % a ==0:
                degPT=360.0/a
                fold_name=f"test_n_{a:02d}"
                full_path=os.path.join(options.map_dir,fold_name)

                print(f"\nStarting mapping with N={a} rotations, {degPT:.2f} degrees per rotation\n")

                if not os.path.exists(full_path):
                    os.makedirs(full_path)
                
                #graph_nav_client.clear_graph() got error saying call stop recording first
                recording_client.start_recording()
                print("\nStarting Recording\n")
                time.sleep(0.1)

                for b in range(a):
                    print(f"\n[N={a} Step{b+1}/{a}] Rotating {degPT:.2f} degrees\n")

                    #snapshot
                    recording_client.create_waypoint(waypoint_name=f"N{a}_Snap{b+1}")
                    time.sleep(3)#need to have this so it goes on when its ready
                    print("\nCreating Waypoint\n")
                    #turn
                    turn_relative(command_client,robot_state_client,degPT)
                    time.sleep(3)

                #stop and download
                recording_client.stop_recording()
                time.sleep(0.5)
                
                # Use the module-level helper, passing the directory and the client
                graph_nav_client.write_graph_and_snapshots(full_path)

                #convert
                print(f"\n[N{a}]Converting to ply...\n")
                ply_name=os.path.join(full_path,f"converted_n_{a}.ply")
                convert_map_to_ply(full_path,ply_name)
                graph_nav_client.clear_graph()
            else:
                print(f"\nN={a} is not a factor of 360, skipping to next N\n")
                continue
            
            
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

    print(f"\nBatter check, charge: {charge:.2f}%\n")

    if charge < limit:
        return False
    return True

def turn_relative(command_client,robot_state_client,yaw_deg):
    yaw_rad=math.radians(yaw_deg)
    transforms=robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    odom_t_body=get_se2_a_tform_b(transforms, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
    new_yaw=odom_t_body.angle+yaw_rad

    #set speed limits using geometry api first
    speed_limit=geometry_pb2.SE2VelocityLimit(
        max_vel=geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=0.5,y=0.5),angular=1.0
        )
    )

    #initialize params and attach speed limits
    params=spot_command_pb2.MobilityParams(vel_limit=speed_limit)

    se2_pose = geometry_pb2.SE2Pose(
        position=geometry_pb2.Vec2(
            x=odom_t_body.x, y=odom_t_body.y)
            ,angle=new_yaw)

    duration=abs(yaw_rad)/0.8
    if duration<2.0: duration=2.0

    cmd = RobotCommandBuilder.synchro_se2_trajectory_command(
        se2_pose,
        frame_name=ODOM_FRAME_NAME,
        params=params  # This is the object built on standalone lines earlier
    )
    command_client.robot_command(cmd, end_time_secs=time.time()+duration)

    time.sleep(duration+0.5)

def convert_map_to_ply(map_dir, output_file):
    """Extracts points directly from the raw Protobuf files and saves a .PLY file"""
    snap_dir = os.path.join(map_dir, 'waypoint_snapshots')
    
    if not os.path.exists(snap_dir):
        print(f"  [ERROR] Could not find 'waypoint_snapshots' inside {map_dir}")
        return

    all_points = []
    
    try:
        files = os.listdir(snap_dir)
        for filename in files:
            # Ignore macOS hidden system files that crash the binary parser
            if filename == '.DS_Store':
                continue
                
            file_path = os.path.join(snap_dir, filename)
            snapshot = map_pb2.WaypointSnapshot()
            
            with open(file_path, 'rb') as f:
                snapshot.ParseFromString(f.read())
                
            cloud = snapshot.point_cloud
            if not cloud.data:
                continue
                
            iter_points = struct.iter_unpack('<3f', cloud.data)
            for p in iter_points:
                all_points.append(p)

        # Write to PLY format
        with open(output_file, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(all_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            for p in all_points:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")
                
    except Exception as e:
        print(f"  [CRITICAL ERROR] Conversion failed: {e}")

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)