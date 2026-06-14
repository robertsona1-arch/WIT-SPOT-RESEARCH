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

#google imports
import grpc

from google.protobuf import wrappers_pb2 as wrappers

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import calculate_aabb_volume, parse_alignment_log, calculate_aabb_volume_xz, create_plot, create_plot_tl, create_plot_expd,analyze_and_plot

def main():
    parser = argparse.ArgumentParser(description="Generate plots and export regression math to Excel.")
    parser.add_argument('--folder', required=True, help='Directory containing the target Excel file.')
    parser.add_argument('--file', default='volume_analysis_by_area.xlsx', help='Specific Excel file to parse and append to.')
    options = parser.parse_args()

    excel_path = os.path.join(options.folder, options.file)
    if not os.path.exists(excel_path):
        print(f"Error: Cannot find target Excel file at {excel_path}")
        return

    graphs_dir = os.path.join(options.folder, 'analysis_graphs')
    os.makedirs(graphs_dir, exist_ok=True)
    seaborn.set_theme(style="whitegrid")
    
    # We load all tabs into a dictionary of DataFrames to parse them
    all_sheets = pandas.read_excel(excel_path, sheet_name=None)
    global_metrics_list = []
    
    print(f"Generating charts and extracting regression algorithms...\n")

    # =========================================================================
    # GENERATE GRAPHS AND CAPTURE MATH
    # =========================================================================
    if 'Averages_Dashboard' in all_sheets:
        df_avg = all_sheets['Averages_Dashboard']
        print("Parsing [Averages_Dashboard] tab...")
        
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Snap_Count', 'Average Point Count', 
                    'Global: Average Point Count vs Snap Count', 'Snap Count', 'Average Point Count', 
                    os.path.join(graphs_dir, 'Dashboard_Snap_vs_AvgPointCount.png'), 'Averages_Dashboard', color='tab:blue'))

        global_metrics_list.extend(analyze_and_plot(df_avg, 'Snap_Count', 'Average Density', 
                    'Global: Average Density vs Snap Count', 'Snap Count', 'Average Density (Pts/m³)', 
                    os.path.join(graphs_dir, 'Dashboard_Snap_vs_AvgDensity.png'), 'Averages_Dashboard', color='tab:orange'))

        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Average Point Count', 
                    'Global: Average Point Count vs Total Time', 'Time_s', 'Average Point Count', 
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_AvgPointCount.png'), 'Averages_Dashboard', color='tab:green'))

        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Average Density', 
                    'Global: Average Density vs Total Time', 'Time_s', 'Average Density (Pts/m³)', 
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_AvgDensity.png'), 'Averages_Dashboard', color='tab:red'))

    area_sheets = [s for s in all_sheets.keys() if s not in ['Master_Data', 'Averages_Dashboard', 'Regression_Metrics']]
    
    for area in area_sheets:
        df_area = all_sheets[area]
        print(f"Parsing [{area}] tab...")
        
        global_metrics_list.extend(analyze_and_plot(df_area, 'Snap_Count', 'Point_Count', 
                    f'{area}: Point Count vs Snap Count', 'Snap Count', 'Point Count', 
                    os.path.join(graphs_dir, f'{area}_Snap_vs_PointCount.png'), area, color='tab:blue'))

        global_metrics_list.extend(analyze_and_plot(df_area, 'Snap_Count', 'Density: Pts Per m3', 
                    f'{area}: Density vs Snap Count', 'Snap Count', 'Density: Pts Per m3', 
                    os.path.join(graphs_dir, f'{area}_Snap_vs_Density.png'), area, color='tab:orange'))

        global_metrics_list.extend(analyze_and_plot(df_area, 'Time_s', 'Point_Count', 
                    f'{area}: Point Count vs Total Time', 'Time_s', 'Point Count', 
                    os.path.join(graphs_dir, f'{area}_Time_vs_PointCount.png'), area, color='tab:green'))

        global_metrics_list.extend(analyze_and_plot(df_area, 'Time_s', 'Density: Pts Per m3', 
                    f'{area}: Density vs Total Time', 'Time_s', 'Density: Pts Per m3', 
                    os.path.join(graphs_dir, f'{area}_Time_vs_Density.png'), area, color='tab:red'))

        global_metrics_list.extend(analyze_and_plot(df_area, 'Time_s', 'Snap_Count', 
                    f'{area}: Snap Count vs Total Time', 'Time_s', 'Snap Count', 
                    os.path.join(graphs_dir, f'{area}_Time_vs_SnapCount.png'), area, color='tab:purple'))

    # =========================================================================
    # EXPORT METRICS BACK TO THE ORIGINAL EXCEL WORKBOOK
    # =========================================================================
    print(f"\nAppending Regression Math back into: {options.file}")
    
    df_metrics = pandas.DataFrame(global_metrics_list)
    
    # We use mode='a' (append) and if_sheet_exists='replace' to overwrite just this single tab without deleting your others
    with pandas.ExcelWriter(excel_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
        df_metrics.to_excel(writer, sheet_name='Regression_Metrics', index=False)
        
        # Apply Auto-Fit Formatting to the new tab
        worksheet = writer.sheets['Regression_Metrics']
        for col in worksheet.columns:
            max_len = max(len(str(cell.value or '')) for cell in col)
            col_letter = get_column_letter(col[0].column)
            worksheet.column_dimensions[col_letter].width = max(max_len + 3, 15)

    print("\nComplete. Check the [Regression_Metrics] tab in your Excel file.")

if __name__ == "__main__":
    main()