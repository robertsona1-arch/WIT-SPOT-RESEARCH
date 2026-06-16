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
    parser = argparse.ArgumentParser(description="Plot object comparisons vs global average for a single test set.")
    parser.add_argument('--file', required=True, help="Path to a single Total_Averages_Group_X.xlsx file")
    args = parser.parse_args()

    file_path = Path(args.file)
    if not file_path.exists():
        print(f"[ERROR] File '{file_path}' does not exist.")
        sys.exit(1)

    print(f"Parsing data from {file_path.name}...")

    # Mapping to align Averages_Dashboard nomenclature with Object sheet nomenclature
    rename_map = {
        'Average Point Count': 'Point_Count',
        'Average Density': 'Density: Pts Per m3',
        'Average Points Per Snapshot': 'Points Per Snapshot',
        'Average Points Per Time': 'Points Per Time',
        'Average Density Per Time': 'Density Per Time',
        'Average Density Per Snapshot': 'Density Per Snapshot'
    }

    all_data = []
    object_names = []

    try:
        xls = pandas.ExcelFile(file_path)
        
        # 1. Parse Individual Object Sheets
        sheets_to_ignore = ['Averages_Dashboard', 'Regression_Metrics', 'Master_Data']
        for sheet in xls.sheet_names:
            if sheet in sheets_to_ignore:
                continue
            df_obj = pandas.read_excel(xls, sheet_name=sheet)
            df_obj['Object_Name'] = sheet
            all_data.append(df_obj)
            object_names.append(sheet)
            
        # 2. Parse the Dashboard as the "Global Average"
        if 'Averages_Dashboard' in xls.sheet_names:
            df_dash = pandas.read_excel(xls, sheet_name='Averages_Dashboard')
            # Standardize the column names so it can be plotted on the same axes
            df_dash = df_dash.rename(columns=rename_map)
            df_dash['Object_Name'] = 'Global Average'
            all_data.append(df_dash)
        else:
            print("[WARNING] 'Averages_Dashboard' not found. Global Average will not be plotted.")

    except Exception as e:
        print(f"[CRITICAL] Failed reading Excel data: {e}")
        sys.exit(1)

    # Combine into one Master DataFrame
    master_df = pandas.concat(all_data, ignore_index=True)

    # 3. Define the 10 Target Metric Pairs
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

    plot_dir = file_path.parent / f"Object_Comparisons_{file_path.stem}"
    plot_dir.mkdir(exist_ok=True)
    print(f"Rendering 10 plots to {plot_dir.name}/ ...")

    # 4. Configure Visual Styling
    seaborn.set_theme(context="paper", style="whitegrid", font_scale=1.2)
    
    # Generate unique colors for objects, force black for Global Average
    palette = seaborn.color_palette("husl", len(object_names))
    color_dict = {obj: color for obj, color in zip(object_names, palette)}
    color_dict['Global Average'] = 'black'
    
    # Generate solid lines for objects, dashed line for Global Average
    style_dict = {obj: "" for obj in object_names}
    style_dict['Global Average'] = (3, 2) # Dash format (3 points on, 2 points off)

    # 5. Plotting Loop
    for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
        if x_col not in master_df.columns or y_col not in master_df.columns:
            print(f"  [SKIP] Missing columns for {title}")
            continue

        plt.figure(figsize=(9, 6))
        seaborn.lineplot(
            data=master_df, x=x_col, y=y_col, 
            hue='Object_Name', palette=color_dict, 
            style='Object_Name', dashes=style_dict,
            linewidth=2.5, markers=True, markersize=8
        )
        
        plt.title(f"{title}", fontsize=14, weight='bold')
        plt.xlabel(x_col.replace('_', ' '))
        plt.ylabel(y_col.replace('_', ' '))
        plt.tight_layout()
        
        safe_title = title.replace(' ', '_')
        plt.savefig(plot_dir / f"Plot_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

    print("[SUCCESS] All single-test object comparisons generated.")

if __name__ == "__main__":
    main()