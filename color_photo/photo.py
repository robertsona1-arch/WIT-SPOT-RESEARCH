"""
photo.py

mac - python3 photo.y <USERNAME> <PASSWORD> --map_dir "DIRECTORY" --mast_dir "DIRECTORY" --dist <NUM> --or_deg <0 or 180>

windows - python photo.y <USERNAME> <PASSWORD> --map_dir "DIRECTORY" --mast_dir "DIRECTORY" --dist <NUM> --or_deg <0 or 180>

This script navigates to the fiducial at the scpeifid distance and orientation, captures a photo, and saves it to the specified directory. The map_dir is where the photos will be saved, and the mast_dir is where the map is stored on the robot (must be pre-processed and uploaded to the robot).

Use ctrl+c to stop the script any time, the robot will stop and sit safetly, and any completed maps will be saved
Please maintian a safe distance from the robot at all time
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET READY TO STOP THE ROBOT IF NEEDED

This script pulls significant portions of code from the Boston Dynamics <blank> files and Google API <blank> files

Minimail AI was used in syntax and structure
"""

"""
Written by <Your name here>, Wentworth Institude of Technology, Douglas D. Schumman School of Engineerning
WIT SPOT Research Group
Advisors: Prof. Tahmid Latif, Prof. Afsaneh Ghanavati
Contributors: Adam Robertson, <other name>, <other name 2>
Date Created: 06/14/2026
Last Updated: 06/14/2026
"""


#general stuff
import argparse
import logging
import os
import struct #ply conversion
import sys
import time
import traceback
import math
from datetime import datetime
from unittest.mock import MagicMock
import numpy as np
import open3d as o3d
import csv
import json
import glob
import pandas 
import re 
import seaborn
import matplotlib.pyplot as plt
import openpyxl
import cv2
from openpyxl.utils import get_column_letter
from pathlib import Path

#bd specific imports
import google.protobuf.timestamp_pb2
#import graph_nav_util
import bosdyn.client.channel 
import bosdyn.client.util
import bosdyn.client.graph_nav 
import bosdyn.client

from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2, robot_command_pb2 as generic_robot_command_pb2, trajectory_pb2, world_object_pb2, basic_command_pb2, image_pb2
from bosdyn.api.gps import gps_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2, map_processing_pb2, recording_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client import map_processing
from bosdyn.client.robot import Robot
from bosdyn.client.lease import LeaseKeepAlive
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_se2_a_tform_b, BODY_FRAME_NAME, get_a_tform_b, VISION_FRAME_NAME
#from bosdyn.client.graph_nav_recording import GraphNavRecordingClient # Standalone in 5.x
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.image import ImageClient, save_images_as_files

#google imports
import grpc

from google.protobuf import wrappers_pb2 as wrappers


ROBOT_IP ="192.168.80.3"
tag_id=1

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import check_batt_perc, convert_map_to_ply_2, nav_to_fid, fine_align, upload_map, log_test_metrics, ensure_recording_stopped, turn_relative

def main(argv):
    parser = argparse.ArgumentParser()
    # Positional Arguments (Terminal order matters)
    parser.add_argument('username', help='Spot Username')
    parser.add_argument('password', help='Spot Password')
    
    # Optional/Named Arguments (Requires terminal flags)
    parser.add_argument('--map_dir', help='Directory to save maps to', required=True)
    parser.add_argument('--mast_dir', help='Directory where the map is stored on the robot', required=True)
    parser.add_argument('--dist', type=float, help='Distance in meters', required=True)
    parser.add_argument('--or_deg',type=float,help='or_deg',required=False, default=0)

    # Execute the parse
    options = parser.parse_args(argv)
    

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
    state_client=robot.ensure_client(RobotStateClient.default_service_name)
    image_client = robot.ensure_client(ImageClient.default_service_name)

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
        ensure_recording_stopped(robot)
        print("\nCommanding robot to stand...\n")
        stand=RobotCommandBuilder.synchro_stand_command()
        command_client.robot_command(stand)
        time.sleep(2)

        # Upload the map to the robot
        upload_map(graph_nav_client, options.mast_dir)
        nav_to_fid(robot,tag_id, dist_m=options.dist)
        dist_error_m, final_yaw_error = fine_align(robot,tag_id, dist=options.dist,iter=100)

        # --- Photo Capture Section (Direct Request Construction) ---
        print(f"\nCapturing image...\n")
        
        # Explicitly declare the target camera channels
        color_sources = [
            'frontleft_fisheye_image', 
            'frontright_fisheye_image',
            'left_fisheye_image', 
            'right_fisheye_image'
        ]
        
        # Manually build an ImageRequest list to correctly inject the pixel format enum
        from bosdyn.client.image import build_image_request
        
        # --- Photo Capture Section (5-Directional Mixed Mode Extraction) ---
        print(f"\nCapturing comprehensive image stack...\n")
        
        # 1. Define distinct lists based on sensor hardware capabilities
        color_sources = [
            'frontleft_fisheye_image', 
            'frontright_fisheye_image',
            'left_fisheye_image', 
            'right_fisheye_image'
        ]
        mono_sources = ['back_fisheye_image']
        
        requests = []
        
        # Build color requests
        for source in color_sources:
            req = build_image_request(source)
            req.pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
            requests.append(req)
            
        # Build monochrome request for the rear sensor array
        for source in mono_sources:
            req = build_image_request(source)
            req.pixel_format = image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
            requests.append(req)
            
        # Fire the unified request array
        image_responses = image_client.get_image(requests)
        
        for response in image_responses:
            source_name = response.source.name
            jpeg_filename = f"Picture_{options.or_deg}_{source_name}.jpg"
            full_save_path = os.path.join(options.map_dir, jpeg_filename)
            
            try:
                img_np = np.frombuffer(response.shot.image.data, dtype=np.uint8)
                
                # Check format to determine OpenCV decoding mode
                if response.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                    img = cv2.imdecode(img_np, cv2.IMREAD_GRAYSCALE)
                else:
                    img = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
                
                if img is not None:
                    # Apply standard spatial coordinate rotations for body mounts
                    if 'left' in source_name:
                        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    elif 'right' in source_name:
                        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                    elif 'back' in source_name:
                        # The rear camera mount orientation is inverted relative to the front line
                        img = cv2.rotate(img, cv2.ROTATE_180)
                        
                    cv2.imwrite(full_save_path, img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                    print(f"  [SAVED] {jpeg_filename}")
                else:
                    print(f"  [ERROR] Could not decode image matrix for {source_name}")
            except Exception as e:
                print(f"  [FAILED] Extraction error on source {source_name}: {e}")
        # -------------------------------------------------------------------

    print("\nScript finished\n")

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)