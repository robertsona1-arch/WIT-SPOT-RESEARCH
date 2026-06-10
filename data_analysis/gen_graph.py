"""
gen_graph.py

mac - python3 gen_graph.py --folder "DIRECTORY" 

windows - python gen_graph.py --folder "DIRECTORY" 

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
Date Created: 06/08/2026
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
import seaborn
import matplotlib.pyplot as plt

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

from leo_funcs import calculate_aabb_volume, parse_alignment_log, create_plot, create_plot_tl

def main():
    parser = argparse.ArgumentParser(description="Generate engineering plots directly from computed Excel sheets.")
    parser.add_argument('--folder', required=True, help='Root directory containing volume_analysis_by_area.xlsx')
    options = parser.parse_args()

    excel_path = os.path.join(options.folder, 'volume_analysis_by_area.xlsx')
    
    if not os.path.exists(excel_path):
        print(f"Error: Cannot find generated Excel file at {excel_path}")
        return

    # Create a dedicated subdirectory for the output images to keep workspace clean
    graphs_dir = os.path.join(options.folder, 'analysis_graphs')
    os.makedirs(graphs_dir, exist_ok=True)
    
    seaborn.set_theme(style="whitegrid")
    excel_file = pandas.ExcelFile(excel_path)
    
    print(f"Extracting metrics and writing plots to: {graphs_dir}\n")

    # =========================================================================
    # 1. GRAPH GENERATION: AVERAGES DASHBOARD
    # =========================================================================
    if 'Averages_Dashboard' in excel_file.sheet_names:
        print("Parsing [Averages_Dashboard] tab...")
        df_avg = excel_file.parse('Averages_Dashboard')
        
        # Plot 1: Snap_Count vs Average Point Count
        create_plot_tl(df_avg, 'Snap_Count', 'Average Point Count', 
                    'Global Metric: Average Point Count vs Snap Count', 
                    'Snap Count', 'Average Point Count', 
                    os.path.join(graphs_dir, 'Dashboard_Snap_vs_AvgPointCount.png'), color='tab:blue')

        # Plot 2: Snap_Count vs Average Density
        create_plot_tl(df_avg, 'Snap_Count', 'Average Density', 
                    'Global Metric: Average Density vs Snap Count', 
                    'Snap Count', 'Average Density (Pts/m³)', 
                    os.path.join(graphs_dir, 'Dashboard_Snap_vs_AvgDensity.png'), color='tab:orange')

        # Plot 3: Time_s vs Average Point Count
        create_plot_tl(df_avg, 'Time_s', 'Average Point Count', 
                    'Global Metric: Average Point Count vs Total Time', 
                    'Time_s', 'Average Point Count', 
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_AvgPointCount.png'), color='tab:green')

        # Plot 4: Time_s vs Average Density
        create_plot_tl(df_avg, 'Time_s', 'Average Density', 
                    'Global Metric: Average Density vs Total Time', 
                    'Time_s', 'Average Density (Pts/m³)', 
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_AvgDensity.png'), color='tab:red')

        #Plot 5: Time_s vs Snap_Count
        create_plot_tl(df_avg, 'Time_s', 'Snap_Count',
                    'Global Metric: Snap Count Evolution vs Total Time',
                    'Time_s', 'Snap Count',
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_SnapCount.png'), color='tab:purple')

        #Plot 6: Average Point Count vs Average Density
        create_plot_tl(df_avg, 'Average Point Count', 'Average Density',
                    'Global Metric: Average Density vs Average Point Count',
                    'Average Point Count', 'Average Density (Pts/m³)',
                    os.path.join(graphs_dir, 'Dashboard_AvgPointCount_vs_AvgDensity.png'), color='tab:brown')
        
        #Plot 7: Snap Count vs Average Points Per Snapshot
        create_plot(df_avg, 'Snap_Count', 'Average Points Per Snapshot',
                    'Global Metric: Average Points Per Snapshot vs Snap Count',
                    'Snap Count', 'Average Points Per Snapshot',
                    os.path.join(graphs_dir, 'Dashboard_Snap_vs_AvgPointsPerSnapshot.png'), color='tab:pink')
        
        #Plot 8 Time vs Average Points Per Time
        create_plot(df_avg, 'Time_s', 'Average Points Per Time',
                    'Global Metric: Average Points Per Time vs Time',
                    'Time_s', 'Average Points Per Time',
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_AvgPointsPerTime.png'), color='tab:cyan')
        
        #Plot 9: Time vs Average Density Per Time
        create_plot(df_avg, 'Time_s', 'Average Density Per Time',
                    'Global Metric: Average Density Per Time vs Time',
                    'Time_s', 'Average Density Per Time',
                    os.path.join(graphs_dir, 'Dashboard_Time_vs_AvgDensityPerTime.png'), color='tab:gray')
        
        #Plot 10: Snap Count vs Average Density Per Snapshot
        create_plot(df_avg, 'Snap_Count', 'Average Density Per Snapshot',
                    'Global Metric: Average Density Per Snapshot vs Snap Count',
                    'Snap Count', 'Average Density Per Snapshot',
                    os.path.join(graphs_dir, 'Dashboard_Snap_vs_AvgDensityPerSnapshot.png'), color='tab:olive')

    # =========================================================================
    # 2. GRAPH GENERATION: INDIVIDUAL ISOLATED AREAS
    # =========================================================================
    # Isolate only the area specific sheets by filtering out the master and summary ledgers
    area_sheets = [s for s in excel_file.sheet_names if s not in ['Master_Data', 'Averages_Dashboard']]
    
    for area in area_sheets:
        print(f"Parsing [{area}] tab...")
        df_area = excel_file.parse(area)
        
        # Plot 1: Snap_Count vs Point_Count
        create_plot_tl(df_area, 'Snap_Count', 'Point_Count', 
                    f'{area}: Point Count vs Snap Count', 
                    'Snap Count', 'Point Count', 
                    os.path.join(graphs_dir, f'{area}_Snap_vs_PointCount.png'), color='tab:blue')

        # Plot 2: Snap_Count vs Density: Pts Per m3
        create_plot_tl(df_area, 'Snap_Count', 'Density: Pts Per m3', 
                    f'{area}: Spatial Density vs Snap Count', 
                    'Snap Count', 'Density: Pts Per m3', 
                    os.path.join(graphs_dir, f'{area}_Snap_vs_Density.png'), color='tab:orange')

        # Plot 3: Time_s vs Point_Count
        create_plot_tl(df_area, 'Time_s', 'Point_Count', 
                    f'{area}: Point Count vs Total Time', 
                    'Time_s', 'Point Count', 
                    os.path.join(graphs_dir, f'{area}_Time_vs_PointCount.png'), color='tab:green')

        # Plot 4: Time_s vs Density: Pts Per m3
        create_plot_tl(df_area, 'Time_s', 'Density: Pts Per m3', 
                    f'{area}: Spatial Density vs Total Time', 
                    'Time_s', 'Density: Pts Per m3', 
                    os.path.join(graphs_dir, f'{area}_Time_vs_Density.png'), color='tab:red')

    print(f"\nExecution complete. All 34 plots output directly to: {graphs_dir}")

if __name__ == "__main__":
    main()