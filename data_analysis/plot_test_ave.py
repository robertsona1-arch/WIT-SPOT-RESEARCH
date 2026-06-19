import argparse
import logging
import os
import struct 
import sys
import time
import traceback
import math
from datetime import datetime
import numpy as np
import open3d as o3d
import csv
import json
import glob
import pandas as pd
import re 
import seaborn as sns
import matplotlib.pyplot as plt
import openpyxl
from openpyxl.utils import get_column_letter
from pathlib import Path
from scipy.optimize import curve_fit
import warnings

#bd specific imports
import google.protobuf.timestamp_pb2
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
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.image import ImageClient, save_images_as_files

#google imports
import grpc
from google.protobuf import wrappers_pb2 as wrappers

# Custom library imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *

# Suppress warnings
warnings.filterwarnings('ignore')

def standardize_columns(df):
    """Aggressively maps variations of Excel headers using a 1-to-1 target lock."""
    df.columns = df.columns.astype(str).str.strip()
    renamed_targets = set()
    new_names = {}
    
    for col in df.columns:
        c_lower = col.lower()
        
        if 'actual' in c_lower or 'absolute' in c_lower:
            continue
            
        if 'volume' in c_lower and 'Volume_m3' not in renamed_targets:
            new_names[col] = 'Volume_m3'
            renamed_targets.add('Volume_m3')
        elif ('global_mean_percent_error' in c_lower or 'global mean percent error' in c_lower or 'percent error' in c_lower or '% error' in c_lower) and 'Percent_Error' not in renamed_targets:
            new_names[col] = 'Percent_Error'
            renamed_targets.add('Percent_Error')
        elif 'snap' in c_lower and 'count' in c_lower and 'Snap_Count' not in renamed_targets:
            new_names[col] = 'Snap_Count'
            renamed_targets.add('Snap_Count')
        elif c_lower in ['time', 'time_s', 'time_sec', 'time (s)'] and 'Time_s' not in renamed_targets:
            new_names[col] = 'Time_s'
            renamed_targets.add('Time_s')
        elif 'points per snap' in c_lower and 'Points_Per_Snap' not in renamed_targets:
            new_names[col] = 'Points_Per_Snap'
            renamed_targets.add('Points_Per_Snap')
        elif 'density per snap' in c_lower and 'Density_Per_Snap' not in renamed_targets:
            new_names[col] = 'Density_Per_Snap'
            renamed_targets.add('Density_Per_Snap')
        elif 'points per time' in c_lower and 'Points_Per_Time' not in renamed_targets:
            new_names[col] = 'Points_Per_Time'
            renamed_targets.add('Points_Per_Time')
        elif 'density per time' in c_lower and 'Density_Per_Time' not in renamed_targets:
            new_names[col] = 'Density_Per_Time'
            renamed_targets.add('Density_Per_Time')
        elif ('density' in c_lower or 'concentration' in c_lower) and 'time' not in c_lower and 'snap' not in c_lower and 'Density_pts_m3' not in renamed_targets:
            new_names[col] = 'Density_pts_m3'
            renamed_targets.add('Density_pts_m3')
        elif 'point' in c_lower and 'count' in c_lower and 'time' not in c_lower and 'snap' not in c_lower and 'Point_Count' not in renamed_targets:
            new_names[col] = 'Point_Count'
            renamed_targets.add('Point_Count')
            
    df.rename(columns=new_names, inplace=True)
    df = df.loc[:, ~df.columns.duplicated()]
    return df

def main():
    parser = argparse.ArgumentParser(description="Plot Mean, STD, and the single highest-performing regression line for Primitives.")
    parser.add_argument('--file', required=True, help="Path to the Total_Averages_Group_X.xlsx file")
    args = parser.parse_args()

    file_path = Path(args.file)
    if not file_path.exists():
        print(f"[ERROR] file {file_path} not found.")
        return

    # Ingest individual object tabs to compute the true variance
    xls = pd.ExcelFile(file_path)
    sheets_to_ignore = ['Averages_Dashboard', 'Regression_Metrics', 'Master_Data']
    
    all_data = []
    for sheet in xls.sheet_names:
        if sheet in sheets_to_ignore:
            continue
        df = pd.read_excel(xls, sheet_name=sheet)
        df = standardize_columns(df)
        all_data.append(df)

    if not all_data:
        print("[ERROR] No valid object data parsed.")
        return

    combined_df = pd.concat(all_data, ignore_index=True)

    # 3-PLOT ISOLATED MATRIX
    plots_to_generate = [
        {'y': 'Volume_m3', 'x': 'Snap_Count', 
         'title': r'Average Volume ($V$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Average Volume ($V$) [m$^3$]', 'x_lbl': r'Amount of Snapshots ($n$)', 
         'fn': 'Primitive_01_Volume_vs_SnapCount.png', 'legend_loc': 'bottom'},
         
        {'y': 'Density_pts_m3', 'x': 'Time_s', 
         'title': r'Point Concentration ($C$) vs Map Capture Time ($t$)', 
         'y_lbl': r'Point Concentration ($C$) [pts/m$^3$]', 'x_lbl': r'Map Capture Time ($t$) [s]', 
         'fn': 'Primitive_02_Concentration_vs_Time.png', 'legend_loc': 'bottom'},
         
        {'y': 'Density_Per_Time', 'x': 'Time_s', 
         'title': r'Concentration Accumulation Rate ($\dot{C}$) vs Map Capture Time ($t$)', 
         'y_lbl': r'Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Map Capture Time ($t$) [s]', 
         'fn': 'Primitive_03_ConcentrationRate_vs_Time.png', 'legend_loc': 'top_right'}
    ]

    output_dir = file_path.parent / f"Best_Fit_Analysis_{file_path.stem}"
    output_dir.mkdir(exist_ok=True)
    print(f"\nPlotting high-DPI figures to {output_dir.name}/ ...")

    # Establish clean journal styling
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 12, 
        'axes.labelsize': 13,
        'axes.labelweight': 'bold', 
        'axes.titlesize': 14,
        'axes.titleweight': 'bold',
        'legend.frameon': True,
        'legend.edgecolor': '#000000',
        'legend.facecolor': '#ffffff'
    })

    for idx, plot_cfg in enumerate(plots_to_generate, 1):
        y_col = plot_cfg['y']
        x_col = plot_cfg['x']
        
        if x_col not in combined_df.columns or y_col not in combined_df.columns:
            missing = [c for c in [x_col, y_col] if c not in combined_df.columns]
            print(f"  [SKIP] Plot '{plot_cfg['fn']}': Missing target columns {missing}")
            continue

        # Group data points by X to isolate discrete mean and variance
        agg_df = combined_df.groupby(x_col)[y_col].agg(['mean', 'std']).reset_index()
        agg_df['std'] = agg_df['std'].fillna(0)

        x_data = agg_df[x_col]
        y_mean = agg_df['mean']
        y_std = agg_df['std']

        fig, ax = plt.subplots(figsize=(10, 6.5))

        # Plot raw experimental data points with standard deviation error bars (High-Contrast Blue)
        ax.errorbar(
            x_data, y_mean, yerr=y_std, 
            fmt='o', color='#1f77b4', ecolor='#a6cee3', 
            elinewidth=2.0, capsize=5, markersize=7, 
            label='Global Mean ± 1 STD'
        )

        # Star-unpacking operator added here (*_) to catch trailing values from backend library
        func, params, name, r2, *_ = calculate_best_fit(x_data, y_mean)
        
        if func is not None and r2 > 0:
            x_smooth = np.linspace(x_data.min(), x_data.max(), 250)
            y_smooth = func(x_smooth, *params)
            
            # (High-Contrast Red for Regression)
            ax.plot(
                x_smooth, y_smooth, 
                linestyle='--', color='#d62728', linewidth=2.5, 
                label=f"Best Fit: {name} ($R^2 = {r2:.3f}$)"
            )

        ax.set_title(plot_cfg['title'], pad=12)
        ax.set_xlabel(plot_cfg['x_lbl'])
        ax.set_ylabel(plot_cfg['y_lbl'])
        
        if plot_cfg['legend_loc'] == 'bottom':
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2, framealpha=1.0, edgecolor='black', fontsize=11)
        else:
            ax.legend(loc='upper right', framealpha=1.0, edgecolor='black', fontsize=11)
        
        plt.tight_layout()
        plt.savefig(output_dir / plot_cfg['fn'], dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"  [SAVED] -> {plot_cfg['fn']}")

    print(f"\n[SUCCESS] Exported primitive trajectory models complete.")

if __name__ == "__main__":
    main()