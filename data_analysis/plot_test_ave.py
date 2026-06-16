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

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *

# Suppress polyfit RankWarnings for perfect fits on small data
warnings.simplefilter('ignore', np.RankWarning)

def main():
    parser = argparse.ArgumentParser(description="Plot Mean, STD, and the single highest-performing regression line.")
    parser.add_argument('--file', required=True, help="Path to the Total_Averages_Group_X.xlsx file")
    args = parser.parse_args()

    file_path = Path(args.file)
    if not file_path.exists():
        print(f"[ERROR] file {file_path} not found.")
        return

    # Ingest individual object tabs to compute the true variance
    xls = pandas.ExcelFile(file_path)
    sheets_to_ignore = ['Averages_Dashboard', 'Regression_Metrics', 'Master_Data']
    
    all_data = []
    for sheet in xls.sheet_names:
        if sheet in sheets_to_ignore:
            continue
        df = pandas.read_excel(xls, sheet_name=sheet)
        all_data.append(df)

    if not all_data:
        print("[ERROR] No valid object data parsed.")
        return

    combined_df = pandas.concat(all_data, ignore_index=True)

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

    output_dir = file_path.parent / f"Best_Fit_Analysis_{file_path.stem}"
    output_dir.mkdir(exist_ok=True)
    print(f"Plotting high-DPI figures to {output_dir.name}/ ...")

    # Establish clean journal styling
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({'font.size': 12, 'axes.labelweight': 'bold', 'legend.frameon': True})

    for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
        if x_col not in combined_df.columns or y_col not in combined_df.columns:
            continue

        # Group data points by X to isolate discrete mean and variance
        agg_df = combined_df.groupby(x_col)[y_col].agg(['mean', 'std']).reset_index()
        agg_df['std'] = agg_df['std'].fillna(0)

        x_data = agg_df[x_col]
        y_mean = agg_df['mean']
        y_std = agg_df['std']

        fig, ax = plt.subplots(figsize=(9, 6))

        # Plot raw experimental data points with standard deviation error bars
        ax.errorbar(
            x_data, y_mean, yerr=y_std, 
            fmt='o', color='black', ecolor='#888888', 
            elinewidth=1.5, capsize=4, markersize=6, 
            label='Global Mean ± 1 STD'
        )

        # Isolate the highest performing mathematical model
        func, params, name, r2 = calculate_best_fit(x_data, y_mean)
        
        if func is not None and r2 > 0:
            # Generate smooth line array spanning the active range
            x_smooth = np.linspace(x_data.min(), x_data.max(), 250)
            y_smooth = func(x_smooth, *params)
            
            ax.plot(
                x_smooth, y_smooth, 
                linestyle='--', color='#d62728', linewidth=2, 
                label=f"Best Fit: {name} ($R^2 = {r2:.3f}$)"
            )

        ax.set_title(title, fontsize=14, weight='bold', pad=12)
        ax.set_xlabel(x_col.replace('_', ' '))
        ax.set_ylabel(y_col.replace('_', ' '))
        ax.legend(loc='best', shadow=True)
        
        plt.tight_layout()
        safe_title = title.replace(' ', '_')
        plt.savefig(output_dir / f"BestFit_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

    print(f"[SUCCESS] Exported 10 optimized plots to: {output_dir.absolute()}")

if __name__ == "__main__":
    main()