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
    parser = argparse.ArgumentParser(description="Compare specific objects across all 6 test environments.")
    parser.add_argument('--folder', required=True, help="Root directory containing the 6 test condition subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Hardcoded definitions matching your 6 exact experimental configurations
    target_subfolders = [
        'closer',
        'rotating_map',
        'standing_map_180deg',
        'standing_map_0deg',
        'varying_height_0deg',
        #'varying_height_180deg'
    ]

    # Verify all directories exist up front
    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if not valid_paths:
        print("[CRITICAL] None of the specified 6 subfolders were located. Check path structure.")
        sys.exit(1)

    # 1. Ingest and group data across the directory tree
    # Map structure: object_master_data[object_name] = [df_from_test1, df_from_test2, ...]
    object_master_data = {}

    print("Gathering cross-test data matrices...")
    for env_name, env_path in valid_paths.items():
        # Locate the specific Total_*.xlsx file exported by your volume script
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            print(f"  [SKIP] No Total_*.xlsx file found in {env_name}/")
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pandas.ExcelFile(file_to_open)
            sheets_to_ignore = ['Master_Data', 'Averages_Dashboard', 'Regression_Metrics']
            
            for sheet in xls.sheet_names:
                if sheet in sheets_to_ignore:
                    continue
                    
                df = pandas.read_excel(xls, sheet_name=sheet)
                df['Test_Environment'] = env_name  # Label the condition row
                
                if sheet not in object_master_data:
                    object_master_data[sheet] = []
                object_master_data[sheet].append(df)
                
        except Exception as e:
            print(f"  [ERROR] Failed to read {file_to_open.name}: {e}")

    if not object_master_data:
        print("[CRITICAL] No object tabs could be compiled from Excel sheets.")
        sys.exit(1)

    # 2. Establish Metric Analysis Suite
    metric_pairs = [
        ('Snap_Count', 'Point_Count', 'Snap Count vs Point Count'),
        ('Snap_Count', 'Density: Pts Per m3', 'Snap Count vs Density'),
        ('Time_s', 'Point_Count', 'Time vs Point Count'),
        ('Time_s', 'Density: Pts Per m3', 'Time vs Density'),
        ('Time_s', 'Snap_Count', 'Time vs Snap Count'),
        ('Point_Count', 'Density: Pts Per m3', 'Point Count vs Density'),
        ('Snap_Count', 'Points Per Snapshot', 'Snap Count vs Points per Snapshot'),
        ('Time_s', 'Points Per Time', 'Time vs Average Points per Time'),
        ('Time_s', 'Density Per Time', 'Time vs Average Density per Time'),
        ('Snap_Count', 'Density Per Snapshot', 'Snap Count vs Density per Snapshot')
    ]

    output_root_dir = base_dir / "Cross_Test_Object_Comparisons"
    output_root_dir.mkdir(exist_ok=True)
    print(f"\nGenerating plots into: {output_root_dir.name}/")

    seaborn.set_theme(context="paper", style="whitegrid", font_scale=1.2)
    
    # Custom high-contrast color scheme for your 6 exact configurations
    env_colors = {
        'closer': '#1f77b4',                # Blue
        'rotating_map': '#ff7f0e',          # Orange
        'standing_map_0deg': '#2ca02c',     # Green
        'standing_map_180deg': '#d62728',   # Red
        'varying_height_0deg': '#9467bd',   # Purple
        #'varying_height_180deg': '#8c564b' # Brown
    }

    # 3. Plotting Loop: One folder per object, containing 10 metrics each
    for obj_name, df_list in object_master_data.items():
        combined_obj_df = pandas.concat(df_list, ignore_index=True)
        
        # Isolate outputs into object-specific directories to stay organized
        obj_plot_dir = output_root_dir / f"Analysis_{obj_name}"
        obj_plot_dir.mkdir(exist_ok=True)
        
        print(f"  -> Processing plots for object: [{obj_name}]")

        for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
            if x_col not in combined_obj_df.columns or y_col not in combined_obj_df.columns:
                continue

            plt.figure(figsize=(10, 6.5))
            
            # Use standard lineplot sorting rules based on whether the X-axis is continuous
            if x_col == 'Point_Count':
                # Bivariate mapping: Sort individual tracking vectors to prevent horizontal cross-lines
                for env, env_df in combined_obj_df.groupby('Test_Environment'):
                    sorted_df = env_df.sort_values(by='Point_Count')
                    plt.plot(sorted_df[x_col], sorted_df[y_col], color=env_colors.get(env, '#7f7f7f'),
                             linewidth=2.5, marker='o', markersize=6, label=env)
            else:
                # Standard temporal sequencing
                seaborn.lineplot(
                    data=combined_obj_df, x=x_col, y=y_col, 
                    hue='Test_Environment', palette=env_colors,
                    linewidth=2.5, marker='o', markersize=6
                )

            plt.title(f"{obj_name}: {title}", fontsize=13, weight='bold', pad=12)
            plt.xlabel(x_col.replace('_', ' '))
            plt.ylabel(y_col.replace('_', ' '))
            plt.legend(loc='best', frameon=True, shadow=True)
            plt.tight_layout()

            safe_title = title.replace(' ', '_').replace(':', '')
            plt.savefig(obj_plot_dir / f"{idx:02d}_{safe_title}.png", dpi=300)
            plt.close()

    print(f"\n[SUCCESS] Cross-test execution complete for all objects.")

if __name__ == "__main__":
    main()