"""
SOR and Convex Hull filtering for point clouds
Takes from each test set and creates averages for the test
Date Created: 8/26/26
Last Updated: 8/26/26
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
from openpyxl.utils import get_column_letter
from pathlib import Path
from scipy.optimize import curve_fit
import warnings
from scipy.spatial import ConvexHull, QhullError

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

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import aggregate_and_export


def main():
    parser = argparse.ArgumentParser(description="Aggregate Spot active stereo test averages for dual volume metrics.")
    parser.add_argument('--folder', required=True, help="Root directory containing the subfolders")
    parser.add_argument('--volumes', required=True, help="Path to the JSON file containing true primitive volumes")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Directory '{base_dir}' does not exist.")
        sys.exit(1)

    volumes_path = Path(args.volumes)
    if not volumes_path.exists():
        print(f"[ERROR] Volumes JSON file '{volumes_path}' does not exist.")
        sys.exit(1)

    # Load the true volumes into a dictionary, explicitly targeting the nested "volumes" key
    with open(volumes_path, 'r') as f:
        true_volumes = json.load(f).get("volumes", {})

    # Dictionary trees to separate the dual metrics
    test_groups_aabb = {'C': {}, 'V': {}, 'S': {}, 'R': {}}
    test_groups_hull = {'C': {}, 'V': {}, 'S': {}, 'R': {}}

    print(f"Scanning {base_dir.absolute()} for test data...\n")

    sheets_to_ignore = ['Regression_Metrics', 'Master_Data']
    files_found = 0

    for file_path in base_dir.rglob("*.xlsx"):
        if file_path.name.startswith("Total_Averages") or file_path.name.startswith("~"):
            continue
            
        if "SOR_AABB" in file_path.name:
            target_dict = test_groups_aabb
        elif "SOR_Hull" in file_path.name:
            target_dict = test_groups_hull
        else:
            continue 
            
        files_found += 1
        
        match = re.search(r'([CVSR])[1-5]', file_path.as_posix(), re.IGNORECASE)
        
        if match:
            test_type = match.group(1).upper()
            print(f"  [INGESTING] Group {test_type} -> {file_path.name}")
            try:
                xls = pandas.ExcelFile(file_path)
                for sheet in xls.sheet_names:
                    if sheet in sheets_to_ignore:
                        continue
                        
                    df = pandas.read_excel(xls, sheet_name=sheet)
                    
                    if sheet not in target_dict[test_type]:
                        target_dict[test_type][sheet] = []
                        
                    target_dict[test_type][sheet].append(df)
            except Exception as e:
                print(f"  [WARNING] Could not read from {file_path.name}: {e}")

    if files_found == 0:
        print("\n[ERROR] No valid SOR .xlsx files were found. Verify your target execution directory.")
        sys.exit(1)

    print(f"Successfully loaded {files_found} workbooks.")
    
    print("\n--- Executing Aggregations for SOR AABB ---")
    aggregate_and_export(test_groups_aabb, "SOR_AABB", base_dir, true_volumes)
    
    print("\n--- Executing Aggregations for SOR Convex Hull ---")
    aggregate_and_export(test_groups_hull, "SOR_Hull", base_dir, true_volumes)

    print("\nData aggregation complete. Output files are ready for Seaborn visualization.")

if __name__ == "__main__":
    main()