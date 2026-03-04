"""
robot_rotation.py

python3 robot_rotation.py <Username> <Password> <Start N>
"""

"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert
Date Created: 3/4/2026
Last Updated: 3/4/2026
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
    parser.add_argument('start_n',type=int,help='Number of initial rotations to perform')

    #optional end N
    parser.add_argument('--end_n',type=int,help='Number of maximum rotations to perform',default=4)

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
    command_client=robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state_client=robot.ensure_client('robot-state')

    if not robot.is_powered_on():
        print("\n robot is powered off, exiting...\n")
        sys.exit(1)

    #4. acquire lease & execution

    #forcefully take the lease:
    lease_client.take()
    with LeaseKeepAlive(lease_client, must_acquire=False, return_at_exit=True):
        print("\nbeginning\n")
        time.sleep(2)

        for a in range(options.start_n, options.end_n+1):
            #battery check, won't run if less than 20%
            if not check_batt_perc(robot_state_client,limit=20.0):
                print(f"\nBattery below 20%. Stopping at N={a}.")
                break
            
            if 360 % a ==0:
                degPT=360.0/a
                fold_name=f"test_n_{a:02d}"

                print(f"\nStarting mapping with N={a} rotations, {degPT:.2f} degrees per rotation\n")

                print(f"\n Taking snapshot at N={a} Step{b+1}/{a}\n")
                time.sleep(0.1)

                for b in range(a):
                    print(f" [N={a} Step{b+1}/{a}] Rotating {degPT:.2f} degrees")

                    #turn
                    turn_relative(command_client,robot_state_client,degPT)
                    time.sleep(0.5)

                #stop 
                time.sleep(0.5)
                
            
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

    params.vel_limit.max_vel.linear.x = 0.5
    params.vel_limit.max_vel.linear.y = 0.5
    params.vel_limit.max_vel.angular = 1.0

    # 1. Package your raw coordinates into an SE2Pose protobuf object
    se2_pose = geometry_pb2.SE2Pose(position=geometry_pb2.Vec2(x=odom_t_body.x, y=odom_t_body.y),angle=new_yaw)

    cmd = RobotCommandBuilder.synchro_se2_trajectory_command(se2_pose,frame_name=ODOM_FRAME_NAME,
    params=params  # This is the object you built on standalone lines earlier
    )
    command_client.robot_command(cmd)

    duration=abs(yaw_rad)/0.8
    if duration<2.0: duration=2.0
    time.sleep(duration)

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)