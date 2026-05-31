"""
calc_abb_vol.py

mac - python3 calc_abb_vol.py --folder "DIRECTORY" 

windows - python calc_abb_vol.py --folder "DIRECTORY" 

This script calculates the AABB volume for 6 predefined areas in each of the 20 .ply files from the folder passed in. 

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
Contributors: Adam Robertson, Geoffrey Siebert, Ryan Staley
Date Created: 05/31/2026
Last Updated: 05/31/2026
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

#bd specific imports
import google.protobuf.timestamp_pb2
#import graph_nav_util
import bosdyn.client.channel 
import bosdyn.client.util
import bosdyn.client.graph_nav 
import bosdyn.client

from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2, robot_command_pb2 as generic_robot_command_pb2, trajectory_pb2, world_object_pb2, basic_command_pb2
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
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_se2_a_tform_b, BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
#from bosdyn.client.graph_nav_recording import GraphNavRecordingClient # Standalone in 5.x
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.world_object import WorldObjectClient

#google imports
import grpc

from google.protobuf import wrappers_pb2 as wrappers

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import calculate_aabb_volume

def main():
    parser = argparse.ArgumentParser(description="Batch process AABB volumes to Excel using local config.")
    parser.add_argument('--folder', required=True, help='Root directory containing test subfolders and JSON config')
    
    options = parser.parse_args()

    # 1. AUTOMATION STEP: Dynamically discover the JSON configuration file in the target folder
    json_files = glob.glob(os.path.join(options.folder, "*.json"))
    
    if not json_files:
        print(f"Error: No JSON configuration file found inside: {options.folder}")
        print("Please ensure your area configuration file is placed directly in that folder.")
        return
        
    config_path = json_files[0]
    print(f"Loaded configuration file: {config_path}")

    # Load Area Configurations
    with open(config_path, 'r') as f:
        config_data = json.load(f)
    areas = config_data.get("areas", {})

    results_data = []

    # Loop through subfolders test_n_01 to test_n_20
    for i in range(1, 21):
        subfolder_name = f"test_n_{i:02d}"
        subfolder_path = os.path.join(options.folder, subfolder_name)

        if not os.path.exists(subfolder_path):
            continue

        # Find the .ply file inside this specific subfolder
        ply_files = glob.glob(os.path.join(subfolder_path, "*.ply"))
        
        if not ply_files:
            print(f"Warning: No .ply file found inside {subfolder_name}. Skipping.")
            continue
            
        filepath = ply_files[0]
        print(f"Processing {filepath}...")

        # Load file into memory
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)

        # Analyze the areas defined in the discovered JSON
        for area_name, bounds in areas.items():
            x_range = bounds["x_range"]
            y_range = bounds["y_range"]
            
            volume, point_count = calculate_aabb_volume(points, x_range, y_range)
            
            results_data.append({
                'Test_Run': i,
                'Area_Name': area_name,
                'Point_Count': point_count,
                'Volume_m3': round(volume, 6)
            })

    if not results_data:
        print("No data processed. Exiting.")
        return

    # Convert to DataFrame and save
    df = pandas.DataFrame(results_data)
    excel_output_path = os.path.join(options.folder, 'volume_analysis.xlsx')
    
    df.to_excel(excel_output_path, index=False, engine='openpyxl')
    print(f"Batch processing complete. Output saved to: {excel_output_path}")

if __name__ == "__main__":
    main()