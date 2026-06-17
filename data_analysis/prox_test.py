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
    parser = argparse.ArgumentParser(description="Compare standing baseline vs closer proximity conditions for quadrants and run averages.")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Isolate target test folders
    target_subfolders = ['standing_map_0deg', 'closer']
    
    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if len(valid_paths) < 2:
        print("[CRITICAL] Proximity impact analysis requires BOTH standing_map_0deg and closer folders.")
        sys.exit(1)

    quadrant_data_compiled = []
    dashboard_data_compiled = []

    rename_map = {
        'Average Point Count': 'Point_Count',
        'Average Density': 'Density: Pts Per m3',
        'Average Points Per Snapshot': 'Points Per Snapshot',
        'Average Points Per Time': 'Points Per Time',
        'Average Density Per Time': 'Density Per Time',
        'Average Density Per Snapshot': 'Density Per Snapshot'
    }

    print("Extracting sheets from baseline and closer test environments...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pandas.ExcelFile(file_to_open)
            
            # Ingest Global Dashboard Averages
            if 'Averages_Dashboard' in xls.sheet_names:
                df_dash = pandas.read_excel(xls, sheet_name='Averages_Dashboard')
                df_dash = df_dash.rename(columns=rename_map)
                df_dash['Test_Environment'] = env_name
                dashboard_data_compiled.append(df_dash)

            # Ingest Specific Target Quadrants
            sheets_to_ignore = ['master_data', 'averages_dashboard', 'regression_metrics']
            for sheet in xls.sheet_names:
                sheet_lower = sheet.lower()
                if sheet_lower in sheets_to_ignore or 'mid' in sheet_lower or 'center' in sheet_lower:
                    continue
                
                if any(q in sheet_lower for q in ['front_lf', 'front_rt', 'back_lf', 'back_rt', 'front lf', 'front rt', 'back lf', 'back rt']):
                    df = pandas.read_excel(xls, sheet_name=sheet)
                    df['Test_Environment'] = env_name
                    df['Box_Identifier'] = sheet  
                    
                    if 'front' in sheet_lower and ('lf' in sheet_lower or 'left' in sheet_lower):
                        df['Quadrant_Class'] = 'Front Left Box'
                    elif 'front' in sheet_lower and ('rt' in sheet_lower or 'right' in sheet_lower):
                        df['Quadrant_Class'] = 'Front Right Box'
                    elif 'back' in sheet_lower and ('lf' in sheet_lower or 'left' in sheet_lower):
                        df['Quadrant_Class'] = 'Back Left Box'
                    elif 'back' in sheet_lower and ('rt' in sheet_lower or 'right' in sheet_lower):
                        df['Quadrant_Class'] = 'Back Right Box'
                    else:
                        df['Quadrant_Class'] = sheet
                        
                    quadrant_data_compiled.append(df)
                    
        except Exception as e:
            print(f"[ERROR] Failed to parse data from {file_to_open.name}: {e}")

    if not quadrant_data_compiled or not dashboard_data_compiled:
        print("[CRITICAL] Missing necessary quadrant or dashboard data sheets.")
        sys.exit(1)

    combined_quad_df = pandas.concat(quadrant_data_compiled, ignore_index=True)
    combined_dash_df = pandas.concat(dashboard_data_compiled, ignore_index=True)

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

    # Establish clean output folder structure
    parent_output_dir = base_dir / "Proximity_Impact_Analysis"
    quad_output_dir = parent_output_dir / "Quadrant_Comparisons"
    avg_output_dir = parent_output_dir / "Global_Average_Comparisons"
    
    quad_output_dir.mkdir(parents=True, exist_ok=True)
    avg_output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nExporting high-contrast figures into: {parent_output_dir.name}/")

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 13,
        'axes.labelweight': 'bold',
        'axes.titlesize': 14,
        'axes.titleweight': 'bold',
        'legend.frameon': True,
        'legend.edgecolor': '#000000'
    })

    # High-Contrast IEEE Schemes (Solid colors, no transparency)
    quad_color_map = {
        ('Front Left Box', 'standing_map_0deg'): '#d62728',    # Solid Red
        ('Front Left Box', 'closer'): '#ff9896',              # Light Red
        ('Front Right Box', 'standing_map_0deg'): '#ff7f0e',   # Solid Orange
        ('Front Right Box', 'closer'): '#fdbf6f',             # Light Orange
        ('Back Left Box', 'standing_map_0deg'): '#1f77b4',     # Solid Blue
        ('Back Left Box', 'closer'): '#a6cee3',               # Light Blue
        ('Back Right Box', 'standing_map_0deg'): '#2ca02c',    # Solid Green
        ('Back Right Box', 'closer'): '#b2df8a'               # Light Green
    }

    avg_color_map = {
        'standing_map_0deg': '#000000',  # Black for baseline
        'closer': '#2ca02c'              # Green for close proximity
    }

    marker_map = {
        'Front Left Box': 'o', 'Front Right Box': 'v', 'Back Left Box': 's', 'Back Right Box': 'D'
    }

    for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
        if x_col not in combined_quad_df.columns or y_col not in combined_quad_df.columns:
            continue

        safe_title = title.replace(' ', '_').replace(':', '')

        # --- GRAPH 1: QUADRANT COMPARISONS ---
        fig, ax = plt.subplots(figsize=(11, 7))
        for (quadrant, env), group_df in combined_quad_df.groupby(['Quadrant_Class', 'Test_Environment']):
            sorted_df = group_df.sort_values(by=x_col)
            line_style = '-' if env == 'standing_map_0deg' else '-.'
            env_label = "Standard Standoff" if env == 'standing_map_0deg' else "Closer Standoff"
            
            ax.plot(
                sorted_df[x_col], sorted_df[y_col],
                color=quad_color_map.get((quadrant, env), '#7f7f7f'),
                linestyle=line_style,
                marker=marker_map.get(quadrant, 'x'),
                linewidth=2.5, markersize=7,
                label=f"{quadrant} ({env_label})"
            )
        ax.set_title(f"Proximity Impact: Quadrant Comparison\n{title}", pad=12)
        ax.set_xlabel(x_col.replace('_', ' '))
        ax.set_ylabel(y_col.replace('_', ' '))
        ax.legend(loc='best', framealpha=1.0, edgecolor='black')
        plt.tight_layout()
        plt.savefig(quad_output_dir / f"Quad_Prox_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

        # --- GRAPH 2: GLOBAL AVERAGE COMPARISONS ---
        fig, ax = plt.subplots(figsize=(10, 6.5))
        for env, group_df in combined_dash_df.groupby('Test_Environment'):
            sorted_df = group_df.sort_values(by=x_col)
            line_style = '-' if env == 'standing_map_0deg' else '--'
            env_label = "Baseline Standoff" if env == 'standing_map_0deg' else "Closer Proximity"
            
            ax.plot(
                sorted_df[x_col], sorted_df[y_col],
                color=avg_color_map.get(env, '#7f7f7f'),
                linestyle=line_style,
                marker='s' if env == 'standing_map_0deg' else '^',
                linewidth=3.0, markersize=7,
                label=env_label
            )
        ax.set_title(f"Global Run Averages Comparison: Baseline vs Closer\n{title}", pad=12)
        ax.set_xlabel(x_col.replace('_', ' '))
        ax.set_ylabel(y_col.replace('_', ' '))
        ax.legend(loc='best', framealpha=1.0, edgecolor='black')
        plt.tight_layout()
        plt.savefig(avg_output_dir / f"Avg_Prox_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

    print(f"\n[SUCCESS] Proximity impact analysis compiled successfully.")
    print(f"  -> Corner quadrant plots saved to: {quad_output_dir.name}/")
    print(f"  -> Isolated run average plots saved to: {avg_output_dir.name}/")

if __name__ == "__main__":
    main()