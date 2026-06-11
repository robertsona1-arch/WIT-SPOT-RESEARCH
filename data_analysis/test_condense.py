"""
test_condense.py

mac - python3 test_condense.py --folder "DIRECTORY" 

windows - python test_condense.py --folder "DIRECTORY" 

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



def main():
    parser = argparse.ArgumentParser(description="Ensemble averaging across 5 test subset Excel files.")
    parser.add_argument('--folder', required=True, help='Directory containing test_1.xlsx through test_5.xlsx')
    options = parser.parse_args()

    # Define the expected subset files
    subset_files = [os.path.join(options.folder, f'test_{i}.xlsx') for i in range(1, 6)]
    
    # Validate that all 5 files exist before starting the math
    for file in subset_files:
        if not os.path.exists(file):
            print(f"Error: Missing required subset file: {file}")
            return

    print(f"Discovered all 5 subsets. Beginning ensemble aggregation...\n")

    # Read the sheet names from the first file to establish the baseline structure
    base_excel = pandas.ExcelFile(subset_files[0])
    sheet_names = base_excel.sheet_names

    output_path = os.path.join(options.folder, 'final_ensemble_averages.xlsx')
    
    with pandas.ExcelWriter(output_path, engine='openpyxl') as writer:
        
        # Loop through every single tab (Master_Data, Averages_Dashboard, Area_1, etc.)
        for sheet in sheet_names:
            print(f"Aggregating data for tab: [{sheet}]")
            
            # 1. Load the corresponding sheet from all 5 subset files into memory
            sheet_dataframes = []
            for file in subset_files:
                df = pandas.read_excel(file, sheet_name=sheet)
                sheet_dataframes.append(df)
            
            # 2. Stack them vertically into one massive DataFrame
            stacked_df = pandas.concat(sheet_dataframes, ignore_index=True)
            
            # 3. Collapse the data: Group by Snap_Count and calculate the mean
            # numeric_only=True prevents Pandas from crashing if it tries to average string columns
            ensemble_avg_df = stacked_df.groupby('Snap_Count').mean(numeric_only=True).reset_index()
            
            # 4. Round everything cleanly for the final engineering output
            ensemble_avg_df = ensemble_avg_df.round(4)
            
            # Write the aggregated data to the new master workbook
            ensemble_avg_df.to_excel(writer, sheet_name=sheet, index=False)

        # --- THE AUTOMATED AUTOFIT LOOP (Carried over from the previous script) ---
        print("\nApplying auto-fit formatting to final ensemble columns...")
        for sheet_name in writer.sheets:
            worksheet = writer.sheets[sheet_name]
            for col in worksheet.columns:
                max_len = max(len(str(cell.value or '')) for cell in col)
                col_letter = get_column_letter(col[0].column)
                worksheet.column_dimensions[col_letter].width = max(max_len + 3, 10)

    print(f"\nAggregation complete! Final ensemble data saved to: {output_path}")

if __name__ == "__main__":
    main()