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
Last Updated: 06/8/2026
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
import seaborn as sns
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

def main():
    parser = argparse.ArgumentParser(description="Generate data visualization charts from volume analysis Excel data.")
    parser.add_argument('--folder', required=True, help='Root directory containing volume_analysis.xlsx')
    options = parser.parse_args()

    excel_path = os.path.join(options.folder, 'volume_analysis.xlsx')
    
    if not os.path.exists(excel_path):
        print(f"Error: Cannot find data file at {excel_path}")
        return

    # Load the processed DataFrame
    df = pandas.read_excel(excel_path)
    
    # Set a clean, professional grid style for engineering data
    sns.set_theme(style="whitegrid")
    
    # ---------------------------------------------------------
    # CHART 1: Volume Stability per Test Run (Bar Chart)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 5))
    sns.barplot(data=df, x='Test_Run', y='Volume_m3', hue='Area_Name')
    plt.title('AABB Volume Stability Across Test Runs (N=1 to N=20)', fontsize=14, fontweight='bold')
    plt.ylabel('Volume (m³)', fontsize=12)
    plt.xlabel('Test Run (Number of Snapshots / Rotations)', fontsize=12)
    plt.legend(title='Monitored Bounding Boxes', loc='lower left')
    
    chart1_path = os.path.join(options.folder, 'plot_volume_stability.png')
    plt.savefig(chart1_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Generated: {chart1_path}")

    # ---------------------------------------------------------
    # CHART 2: Data Acquisition Efficiency (Time per Snapshot Line Chart)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 5))
    # Drop duplicate rows since Time_per_snapshot_s is identical per Test_Run across areas
    df_unique_runs = df.drop_duplicates(subset=['Test_Run'])
    df_unique_time=df.drop_duplicates(subset=['Time_s'])
    
    sns.lineplot(data=df_unique_runs, x='Test_Run', y='Time_per_snapshot', marker='o', color='purple', linewidth=2.5)
    plt.title('Data Acquisition Efficiency: Time per Snapshot', fontsize=14, fontweight='bold')
    plt.ylabel('Time Per Snapshot (seconds)', fontsize=12)
    plt.xlabel('Test Run (Total Snapshots in Sequence)', fontsize=12)
    
    chart2_path = os.path.join(options.folder, 'plot_snapshot_efficiency.png')
    plt.savefig(chart2_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Generated: {chart2_path}")

    # ---------------------------------------------------------
    # CHART 3: Kinematic Drift: Distance vs. Yaw Error (Dual-Axis Line Chart)
    # ---------------------------------------------------------
    fig, ax1 = plt.subplots(figsize=(10, 5))
    
    # Left Y-Axis: Distance Error
    color = 'tab:red'
    ax1.set_xlabel('Test Run (N)', fontsize=12)
    ax1.set_ylabel('Fiducial Distance Error (meters)', color=color, fontsize=12)
    sns.lineplot(data=df_unique_runs, x='Test_Run', y='Dist_Error_m', color=color, marker='s', ax=ax1, linewidth=2)
    ax1.tick_params(axis='y', labelcolor=color)
    
    # Right Y-Axis: Yaw Error
    ax2 = ax1.twinx()  
    color = 'tab:blue'
    ax2.set_ylabel('Fiducial Yaw Error (degrees)', color=color, fontsize=12)
    sns.lineplot(data=df_unique_runs, x='Test_Run', y='Yaw_Error_deg', color=color, marker='^', ax=ax2, linewidth=2)
    ax2.tick_params(axis='y', labelcolor=color)
    
    plt.title('Kinematic Tracking Alignment Errors Across Test Runs', fontsize=14, fontweight='bold')
    fig.tight_layout()  
    
    chart3_path = os.path.join(options.folder, 'plot_alignment_errors.png')
    plt.savefig(chart3_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Generated: {chart3_path}")
    
    print("\nAll engineering plots successfully output to your folder.")

if __name__ == "__main__":
    main()