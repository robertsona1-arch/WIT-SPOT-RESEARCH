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
Last Updated: 06/9/2026
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
import openpyxl
from openpyxl import utils

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

from leo_funcs import calculate_aabb_volume, parse_alignment_log

def main():
    parser = argparse.ArgumentParser(description="Batch process AABB volumes to Excel using local config.")
    parser.add_argument('--folder', required=True, help='Root directory containing test subfolders and JSON config')
    
    options = parser.parse_args()

    # 1. Load Configurations and Logs
    json_file_n = os.path.join(options.folder, "area_bounds_n.json")
    json_file_1 = os.path.join(options.folder, "area_bounds_1.json")
    log_file = os.path.join(options.folder, "alignment_metrics_log.txt")
    
    if not os.path.exists(json_file_n) or not os.path.exists(json_file_1):
        print(f"Error: Missing JSON configuration files inside: {options.folder}")
        return

    # Parse the text log into memory
    log_metrics = parse_alignment_log(log_file)

    with open(json_file_1, 'r') as f:
        areas_1 = json.load(f).get("areas", {})

    with open(json_file_n, 'r') as f:
        areas_n = json.load(f).get("areas", {})

    results_data = []
    log_format="%Y-%m-%d %H:%M:%S" 


    # 2. Process Data
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
        
        # Grab the alignment metrics for this specific test run (default to None if missing)
        test_metrics = log_metrics.get(a, {'Time_s': None, 'Dist_Error_m': None, 'Yaw_Error_deg': None})

        for area_name, bounds in areas.items():
            x_range = bounds["x_range"]
            y_range = bounds["y_range"]
            
            volume, point_count = calculate_aabb_volume(points, x_range, y_range)
            
            results_data.append({
                'Snap_Count': a,
                'Time_s': test_metrics['Time_s'],
                'Area_Name': area_name,
                'Point_Count': point_count,
                'Volume_m3': round(volume, 6),
                'Dist_Error_m': test_metrics['Dist_Error_m'],
                'Yaw_Error_deg': test_metrics['Yaw_Error_deg']
            })

    if not results_data:
        print("No data processed. Exiting.")
        return

    # 3. DataFrame Math & Export
    df_master = pandas.DataFrame(results_data) #column gets renamed to just 'Time_s' 
    
    # --- PIPELINE MATH EXAMPLE ---
    # Pandas handles array math automatically. This prevents you from having to drag formulas in Excel.
    # We use np.where to avoid divide-by-zero errors if volume is exactly 0.
    df_master['Time Per Snapshot'] = np.where(df_master['Time_s'] > 0, round(df_master['Time_s'] / df_master['Snap_Count'], 2), 0)
    df_master['Density: Pts Per m3'] = np.where(df_master['Volume_m3'] > 0, round(df_master['Point_Count'] / df_master['Volume_m3'], 2), 0)
    df_master['Points Per Snapshot'] = np.where(df_master['Snap_Count'] > 0, round(df_master['Point_Count'] / df_master['Snap_Count'], 2), 0)
    df_master['Density Per Time']= np.where(df_master['Time_s'] > 0, round(df_master['Density: Pts Per m3'] / df_master['Time_s'], 2), 0)
    df_master['Density Per Snapshot'] = np.where(df_master['Snap_Count'] > 0, round(df_master['Density: Pts Per m3'] / df_master['Snap_Count'], 2), 0)
    df_master['Points Per Time'] = np.where(df_master['Time_s'] > 0, round(df_master['Point_Count'] / df_master['Time_s'], 2), 0)
    df_master['Average Point Count'] = round(df_master.groupby('Snap_Count')['Point_Count'].transform('mean'), 2)
    df_master['Average Density'] = round(df_master.groupby('Snap_Count')['Density: Pts Per m3'].transform('mean'), 6)
    df_master['Average Points Per Snapshot'] = round(df_master.groupby('Snap_Count')['Points Per Snapshot'].transform('mean'), 2)
    df_master['Average Points Per Time']=round(df_master.groupby('Snap_Count')['Points Per Time'].transform('mean'), 2)
    df_master['Average Density Per Time']=round(df_master.groupby('Snap_Count')['Density Per Time'].transform('mean'), 2)
    df_master['Average Density Per Snapshot']=round(df_master.groupby('Snap_Count')['Density Per Snapshot'].transform('mean'), 2)

    # CONSTRUCT THE AVERAGES DASHBOARD SHEET
    # We group by Snap_Count and extract the mean of numeric columns across all 6 areas
    # Since Time, Dist_Error, and Yaw_Error are identical for all areas within a run, .first() safely grabs them
    df_averages = df_master.groupby('Snap_Count').agg({
        'Time_s': 'first',
        'Average Point Count': 'first',
        'Average Density': 'first',
        'Average Points Per Snapshot': 'first',
        'Average Points Per Time': 'first',
        'Average Density Per Time': 'first',
        'Average Density Per Snapshot': 'first',
    }).reset_index()

    # 4. EXCEL MULTI-TAB EXPORT PIPELINE
    excel_output_path = os.path.join(options.folder, 'volume_analysis_by_area.xlsx')
    print(f"\nExporting structured sheets to: {excel_output_path}\n")
    
    with pandas.ExcelWriter(excel_output_path, engine='openpyxl') as writer:
        
        # TAB 1: Master Data (The uncollapsed full history ledger)
        df_master.to_excel(writer, sheet_name='Master_Data', index=False)
        print("  --> Created worksheet tab: [Master_Data]")
        
        # TAB 2: Averages Dashboard (The aggregated overview tracking run performance)
        df_averages.to_excel(writer, sheet_name='Averages_Dashboard', index=False)
        print("  --> Created worksheet tab: [Averages_Dashboard] (Run-level summaries)")
        
        # TABS 3-8: Individual Area Sheets (Cleaned slices for specific spatial analysis)
        unique_areas = sorted(df_master['Area_Name'].unique())
        for area in unique_areas:
            df_area_slice = df_master[df_master['Area_Name'] == area].copy()
            
            # Drop the string name since it matches the tab name, keeping columns streamlined
            df_area_slice = df_area_slice.drop(columns=['Area_Name','Yaw_Error_deg','Dist_Error_m', 'Average Point Count','Average Density','Average Points Per Snapshot','Average Density Per Snapshot','Average Density Per Time','Average Points Per Time'])
            
            # Sanitize tab name string limits for the openpyxl backend
            safe_tab_name = re.sub(r'[\*\\\/\? \:\[\]]', '_', area)[:31]
            
            df_area_slice.to_excel(writer, sheet_name=safe_tab_name, index=False)
            print(f"  --> Created worksheet tab: [{safe_tab_name}]")

            # --- THE AUTOMATED AUTOFIT LOOP ---
        print("\nAdjusting column widths to prevent text clipping...")
        for sheet_name in writer.sheets:
            worksheet = writer.sheets[sheet_name]
            
            # Iterate over every single column in the worksheet
            for col in worksheet.columns:
                # Find the maximum character length among all cells in this column
                # We cast to string to handle integers/floats accurately
                max_len = max(len(str(cell.value or '')) for cell in col)
                
                # Extract the alphanumeric letter coordinate for the column (e.g., 'A', 'B', 'AA')
                col_letter = openpyxl.utils.get_column_letter(col[0].column)
                
                # Apply the calculation: max length found + 3 buffer characters for visual padding
                # We enforce a minimum width of 10 so thin data columns don't compress their headers
                worksheet.column_dimensions[col_letter].width = max(max_len + 3, 10)

    print("\nBatch export complete. All worksheets isolated and successfully output.")

if __name__ == "__main__":
    main()