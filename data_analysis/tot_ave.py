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


def main():
    parser = argparse.ArgumentParser(description="Aggregate Spot active stereo test averages per object and summary dashboards.")
    parser.add_argument('--folder', required=True, help="Root directory containing the subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Dictionary tree: test_groups[test_type][sheet_name] = [df1, df2, ...]
    test_groups = {'C': {}, 'V': {}, 'S': {}, 'R': {}}

    print(f"Scanning {base_dir.absolute()} and all subdirectories for test data...\n")

    # Only ignoring the raw logs and derived regressions; we now INCLUDE 'Averages_Dashboard'
    sheets_to_ignore = ['Regression_Metrics', 'Master_Data']
    files_found = 0

    for file_path in base_dir.rglob("*.xlsx"):
        if file_path.name.startswith("Total_Averages") or file_path.name.startswith("~"):
            continue
            
        files_found += 1
        
        # Match test group identifiers (C, V, S, or R followed by run 1-5)
        match = re.search(r'([CVSR])[1-5]', file_path.as_posix(), re.IGNORECASE)
        
        if match:
            test_type = match.group(1).upper()
            print(f"  [MATCH] Found '{file_path.name}' in '{file_path.parent.name}' -> Group {test_type}")
            try:
                xls = pandas.ExcelFile(file_path)
                
                for sheet in xls.sheet_names:
                    if sheet in sheets_to_ignore:
                        continue
                        
                    # --- FILTER STEP: Explicitly omit Front_lf_box ---
                    if sheet == 'Front_lf_box':
                        continue
                    # --------------------------------------------------
                        
                    df = pandas.read_excel(xls, sheet_name=sheet)
                    
                    if sheet not in test_groups[test_type]:
                        test_groups[test_type][sheet] = []
                        
                    test_groups[test_type][sheet].append(df)
            except Exception as e:
                print(f"  [WARNING] Could not read from {file_path.name}: {e}")
        else:
            print(f"  [IGNORED] '{file_path.name}' in '{file_path.parent.name}' (Missing C, V, S, or R + 1-5)")

    if files_found == 0:
        print("\n[ERROR] No .xlsx files were found. Verify your target execution directory.")
        sys.exit(1)

    print("\nExecuting comprehensive aggregations (Objects + Dashboard summaries)...\n")
    for test_type, sheets_dict in test_groups.items():
        if not sheets_dict:
            continue

        print(f"Processing Group {test_type}...")
        output_filename = base_dir / f"Total_Averages_Group_{test_type}.xlsx"
        
        with pandas.ExcelWriter(output_filename, engine='openpyxl') as writer:
            # We sort keys to ensure object sheets write first, and the Dashboard summary writes last
            for sheet_name in sorted(sheets_dict.keys(), key=lambda x: x == 'Averages_Dashboard'):
                combined_df = pandas.concat(sheets_dict[sheet_name], ignore_index=True)
                
                # Compute the mean for numeric data grouped by snapshot count
                averaged_df = combined_df.groupby('Snap_Count').mean(numeric_only=True).reset_index()
                averaged_df.to_excel(writer, index=False, sheet_name=sheet_name)
            
        print(f"  [SUCCESS] Exported all metrics and summary dashboards to: {output_filename.name}")

    print("\nData aggregation complete. Ready for Seaborn visualization.")

if __name__ == "__main__":
    main()