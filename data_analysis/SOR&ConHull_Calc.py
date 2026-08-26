"""
SOR and Convex Hull filtering for point clouds

Date Created: 8/25/26
Last Updated: 8/25/26
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

from leo_funcs import export_to_excel, parse_alignment_log, apply_dataframe_math, calculate_dual_volumes_sor

def main():
    parser = argparse.ArgumentParser(description="Batch process AABB and Convex Hull volumes to Excel.")
    parser.add_argument('--folder', required=True, help='Root directory containing test subfolders and JSON config')
    parser.add_argument('--axis', required=True, choices=['xy', 'xz'], help='Axis combination to calculate volume for')
    
    options = parser.parse_args()

    json_file_n = os.path.join(options.folder, "area_bounds_n.json")
    json_file_1 = os.path.join(options.folder, "area_bounds_1.json")
    log_file = os.path.join(options.folder, "alignment_metrics_log.txt")
    
    if not os.path.exists(json_file_n) or not os.path.exists(json_file_1):
        print(f"Error: Missing JSON configuration files inside: {options.folder}")
        return

    log_metrics = parse_alignment_log(log_file)

    with open(json_file_1, 'r') as f:
        areas_1 = json.load(f).get("areas", {})

    with open(json_file_n, 'r') as f:
        areas_n = json.load(f).get("areas", {})

    # Create two separate data collection lists
    results_aabb = []
    results_hull = []

    for a in range(1, 21):
        subfolder_name = f"test_n_{a:02d}"
        subfolder_path = os.path.join(options.folder, subfolder_name)

        if not os.path.exists(subfolder_path):
            continue

        ply_files = glob.glob(os.path.join(subfolder_path, "*.ply"))
        if not ply_files:
            continue
            
        filepath = ply_files[0]
        print(f"Processing {filepath}...")

        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)

        areas = areas_1 if a == 1 else areas_n
        test_metrics = log_metrics.get(a, {'Time_s': None, 'Dist_Error_m': None, 'Yaw_Error_deg': None})

        for area_name, bounds in areas.items():
            # Calculate both metrics in a single pass
            aabb_vol, hull_vol, point_count = calculate_dual_volumes_sor(points, bounds, options.axis)
            
            # Base dictionary of shared metrics
            base_dict = {
                'Snap_Count': a,
                'Time_s': test_metrics['Time_s'],
                'Area_Name': area_name,
                'Point_Count': point_count,
                'Dist_Error_m': test_metrics['Dist_Error_m'],
                'Yaw_Error_deg': test_metrics['Yaw_Error_deg']
            }
            
            # Append to AABB list
            dict_aabb = base_dict.copy()
            dict_aabb['Volume_m3'] = round(aabb_vol, 6)
            results_aabb.append(dict_aabb)
            
            # Append to Hull list
            dict_hull = base_dict.copy()
            dict_hull['Volume_m3'] = round(hull_vol, 6)
            results_hull.append(dict_hull)

    if not results_aabb:
        print("No data processed. Exiting.")
        return

    # Process DataFrames
    df_master_aabb, df_avg_aabb = apply_dataframe_math(results_aabb)
    df_master_hull, df_avg_hull = apply_dataframe_math(results_hull)

    # Export
    fold_name = os.path.basename(os.path.normpath(options.folder)).replace(" ", "")
    
    excel_path_aabb = os.path.join(options.folder, f'SOR_AABB_{fold_name}.xlsx')
    export_to_excel(df_master_aabb, df_avg_aabb, excel_path_aabb)
    
    excel_path_hull = os.path.join(options.folder, f'SOR_Hull_{fold_name}.xlsx')
    export_to_excel(df_master_hull, df_avg_hull, excel_path_hull)

    print("\nBatch export complete. Dual metrics successfully output.")

if __name__ == "__main__":
    main()