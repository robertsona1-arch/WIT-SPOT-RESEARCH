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

    options = parser.parse_args()
    
    # Extract the folder name and clean the spaces (e.g., "4-8 V1" -> "4-8V1")
    folder_name = os.path.basename(os.path.normpath(options.folder))
    clean_folder_suffix = folder_name.replace(" ", "")
    
    # Dynamically generate the expected filename based on the input directory
    default_excel_name = f"vabb_{clean_folder_suffix}.xlsx"

    excel_path = os.path.join(options.folder, default_excel_name)

    if not os.path.exists(excel_path):
        print(f"Error: Cannot find target Excel file at {excel_path}")
        return

    # Apply the exact same naming scheme to the output directory
    graphs_dir = os.path.join(options.folder, f"analysis_graphs_{clean_folder_suffix}")
    os.makedirs(graphs_dir, exist_ok=True)
    seaborn.set_theme(style="whitegrid")

    # Absolute normalized path to eliminate relative quirks
    norm_path = os.path.normpath(options.folder)

    # Extract "6-10 C1" -> Clean to "6-10C1"
    run_name = os.path.basename(norm_path)
    run_suffix = run_name.replace(" ", "")

    # Extract the parent directory: "/path/to/closer" -> returns "closer"
    parent_dir_path = os.path.dirname(norm_path)
    test_type = os.path.basename(parent_dir_path) 

    # Combine them for clear names (e.g., "closer_6-10C1")
    test_namespace = f"{test_type}_{run_suffix}"
    
    # We load all tabs into a dictionary of DataFrames to parse them
    excel_file = pandas.read_excel(excel_path, sheet_name=None)
    global_metrics_list = []
    
    print(f"Generating charts and extracting regression algorithms...\n")

    # =========================================================================
    # GENERATE GRAPHS AND CAPTURE MATH
    # =========================================================================
    if 'Averages_Dashboard' in excel_file:
        df_avg = excel_file['Averages_Dashboard']
        print("Parsing [Averages_Dashboard] tab...")
        
        global_metrics_list.extend(
            analyze_and_plot(
                df_avg, 
                'Snap_Count', 
                'Average Point Count', 
                f'{test_type.capitalize()} ({run_name}): Average Point Count vs Snap Count',  # Dynamic Title
                'Snap Count', 
                'Average Point Count', 
                os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Snap_vs_AvgPointCount.png'),  # Dynamic Filename
                'Averages_Dashboard', 
                color='tab:blue'
            )
        )

        # Plot 2: Snap_Count vs Average Density
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Snap_Count', 'Average Density', 
                    f'{test_type.capitalize()} ({run_name}): Average Density vs Snap Count', 
                    'Snap Count', 'Average Density (Pts/m³)', 
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Snap_vs_AvgDensity.png'), 'Averages_Dashboard', color='tab:orange'))

        # Plot 3: Time_s vs Average Point Count
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Average Point Count', 
                    f'{test_type.capitalize()} ({run_name}): Average Point Count vs Total Time', 
                    'Time_s', 'Average Point Count', 
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Time_vs_AvgPointCount.png'), 'Averages_Dashboard', color='tab:green'))

        # Plot 4: Time_s vs Average Density
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Average Density', 
                    f'{test_type.capitalize()} ({run_name}): Average Density vs Total Time', 
                    'Time_s', 'Average Density (Pts/m³)', 
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Time_vs_AvgDensity.png'), 'Averages_Dashboard', color='tab:red'))

        # Plot 5: Time_s vs Snap_Count
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Snap_Count',
                    f'{test_type.capitalize()} ({run_name}): Snap Count Evolution vs Total Time',
                    'Time_s', 'Snap Count',
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Time_vs_SnapCount.png'), 'Averages_Dashboard', color='tab:purple'))

        # Plot 6: Average Point Count vs Average Density
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Average Point Count', 'Average Density',
                    f'{test_type.capitalize()} ({run_name}): Average Density vs Average Point Count',
                    'Average Point Count', 'Average Density (Pts/m³)',
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_AvgPointCount_vs_AvgDensity.png'), 'Averages_Dashboard', color='tab:brown'))
        
        # Plot 7: Snap Count vs Average Points Per Snapshot
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Snap_Count', 'Average Points Per Snapshot',
                    f'{test_type.capitalize()} ({run_name}): Average Points Per Snapshot vs Snap Count',
                    'Snap Count', 'Average Points Per Snapshot',
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Snap_vs_AvgPointsPerSnapshot.png'), 'Averages_Dashboard', color='tab:brown'))
        
        # Plot 8: Time vs Average Points Per Time
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Average Points Per Time',
                    f'{test_type.capitalize()} ({run_name}): Average Points Per Time vs Time',
                    'Time_s', 'Average Points Per Time',
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Time_vs_AvgPointsPerTime.png'), 'Averages_Dashboard', color='tab:cyan'))
        
        # Plot 9: Time vs Average Density Per Time
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Time_s', 'Average Density Per Time',
                    f'{test_type.capitalize()} ({run_name}): Average Density Per Time vs Time',
                    'Time_s', 'Average Density Per Time',
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Time_vs_AvgDensityPerTime.png'), 'Averages_Dashboard', color='tab:gray'))
        
        # Plot 10: Snap Count vs Average Density Per Snapshot
        global_metrics_list.extend(analyze_and_plot(df_avg, 'Snap_Count', 'Average Density Per Snapshot',
                    f'{test_type.capitalize()} ({run_name}): Average Density Per Snapshot vs Snap Count',
                    'Snap Count', 'Average Density Per Snapshot',
                    os.path.join(graphs_dir, f'{test_namespace}_Dashboard_Snap_vs_AvgDensityPerSnapshot.png'), 'Averages_Dashboard', color='tab:olive'))

    # =========================================================================
    # 2. GRAPH GENERATION: INDIVIDUAL ISOLATED AREAS
    # =========================================================================
    # Isolate only the area specific sheets by filtering out the master and summary ledgers
    area_sheets = [s for s in excel_file.keys() if s not in ['Master_Data', 'Averages_Dashboard','Regression_Metrics']]
    
    for area in area_sheets:
        print(f"Parsing [{area}] tab...")
        df_area = excel_file[area]
        
        # Plot 1: Snap_Count vs Point_Count
        analyze_and_plot(df_area, 'Snap_Count', 'Point_Count', 
                    f'{test_type.capitalize()} ({run_name}) - {area}: Point Count vs Snap Count', 
                    'Snap Count', 'Point Count', 
                    os.path.join(graphs_dir, f'{test_namespace}_{area}_Snap_vs_PointCount.png'),area, color='tab:blue')

        # Plot 2: Snap_Count vs Density: Pts Per m3
        analyze_and_plot(df_area, 'Snap_Count', 'Density: Pts Per m3', 
                    f'{test_type.capitalize()} ({run_name}) - {area}: Spatial Density vs Snap Count', 
                    'Snap Count', 'Density: Pts Per m3', 
                    os.path.join(graphs_dir, f'{test_namespace}_{area}_Snap_vs_Density.png'), area, color='tab:orange')

        # Plot 3: Time_s vs Point_Count
        analyze_and_plot(df_area, 'Time_s', 'Point_Count', 
                    f'{test_type.capitalize()} ({run_name}) - {area}: Point Count vs Total Time', 
                    'Time_s', 'Point Count', 
                    os.path.join(graphs_dir, f'{test_namespace}_{area}_Time_vs_PointCount.png'), area, color='tab:green')

        # Plot 4: Time_s vs Density: Pts Per m3
        analyze_and_plot(df_area, 'Time_s', 'Density: Pts Per m3', 
                    f'{test_type.capitalize()} ({run_name}) - {area}: Spatial Density vs Total Time', 
                    'Time_s', 'Density: Pts Per m3', 
                    os.path.join(graphs_dir, f'{test_namespace}_{area}_Time_vs_Density.png'), area, color='tab:red')
        
    print(f"\nExecution complete. All 34 plots output directly to: {graphs_dir}")

if __name__ == "__main__":
    main()