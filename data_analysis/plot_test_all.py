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

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *
# Suppress curve_fit optimization convergence warnings
warnings.filterwarnings('ignore')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True, help="Path to Total_Averages_Group_X.xlsx")
    args = parser.parse_args()
    file_path = Path(args.file)

    xls = pandas.ExcelFile(file_path)
    sheets_to_ignore = ['Averages_Dashboard', 'Regression_Metrics', 'Master_Data']
    object_data = []
    
    for sheet in xls.sheet_names:
        if sheet in sheets_to_ignore: continue
        df_obj = pandas.read_excel(xls, sheet_name=sheet)
        df_obj['Object_Name'] = sheet
        object_data.append(df_obj)

    combined_objects_df = pandas.concat(object_data, ignore_index=True)

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

    unified_out_dir = file_path.parent / f"Unified_Analysis_{file_path.stem}"
    isolated_out_dir = file_path.parent / f"Isolated_Global_Analysis_{file_path.stem}"
    unified_out_dir.mkdir(exist_ok=True)
    isolated_out_dir.mkdir(exist_ok=True)

    # DYNAMIC PALETTE ALLOCATION: Adapts gracefully to whether 5 or 6 objects exist in the sheet
    active_objects = combined_objects_df['Object_Name'].unique()
    seaborn.set_theme(context="paper", style="whitegrid", font_scale=1.2)
    palette = seaborn.color_palette("husl", len(active_objects))
    color_dict = {obj: color for obj, color in zip(active_objects, palette)}

    for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
        if x_col not in combined_objects_df.columns or y_col not in combined_objects_df.columns: continue

        is_bivariate = (x_col == 'Point_Count' and y_col == 'Density: Pts Per m3')

        if is_bivariate:
            state_agg = combined_objects_df.groupby('Snap_Count').agg({
                'Point_Count': ['mean', 'std'],
                'Density: Pts Per m3': ['mean', 'std']
            }).reset_index()
            
            agg_df = pandas.DataFrame({
                'x_mean': state_agg['Point_Count']['mean'],
                'x_std': state_agg['Point_Count']['std'].fillna(0),
                'y_mean': state_agg['Density: Pts Per m3']['mean'],
                'y_std': state_agg['Density: Pts Per m3']['std'].fillna(0)
            }).sort_values(by='x_mean').reset_index(drop=True)
        else:
            standard_agg = combined_objects_df.groupby(x_col)[y_col].agg(['mean', 'std']).reset_index()
            agg_df = pandas.DataFrame({
                'x_mean': standard_agg[x_col],
                'x_std': 0,
                'y_mean': standard_agg['mean'],
                'y_std': standard_agg['std'].fillna(0)
            })

        x_plot, y_plot = agg_df['x_mean'], agg_df['y_mean']
        x_err = agg_df['x_std'] if is_bivariate else None
        y_err = agg_df['y_std']

        fit_result, params, name, r2, x_min, x_max = calculate_best_fit(x_plot, y_plot, is_bivariate=is_bivariate)

        x_smooth_raw = np.linspace(x_plot.min(), x_plot.max(), 300)
        x_smooth_scaled = (x_smooth_raw - x_min) / (x_max - x_min) if x_min is not None else None

        if fit_result is None and name == "2nd-Order Polynomial":
            y_smooth = np.polyval(params, x_smooth_scaled)
        elif fit_result is not None and name == "Linear":
            y_smooth = fit_result(x_smooth_raw, *params)
        elif fit_result is not None and name == "Exponential":
            y_smooth = fit_result(x_smooth_scaled, *params)
        elif fit_result is not None and name == "Power Law":
            y_smooth = fit_result(x_smooth_scaled + 0.01, *params)
        else:
            y_smooth = None

        # --- 1. UNIFIED PLOT ---
        fig, ax = plt.subplots(figsize=(10, 6.5))
        if x_col == 'Point_Count':
            for obj, obj_df in combined_objects_df.groupby('Object_Name'):
                ax.plot(obj_df.sort_values('Point_Count')[x_col], obj_df.sort_values('Point_Count')[y_col], 
                        color=color_dict[obj], alpha=0.35, linewidth=1.5, marker='o', markersize=4, label=obj)
        else:
            seaborn.lineplot(data=combined_objects_df, x=x_col, y=y_col, hue='Object_Name', 
                         palette=color_dict, alpha=0.35, linewidth=1.5, marker='o', markersize=4, ax=ax)

        ax.errorbar(x_plot, y_plot, yerr=y_err, xerr=x_err, fmt='o', color='black', 
                    ecolor='#555555', elinewidth=2, capsize=4, markersize=7, label='Global Mean ± 1 STD', zorder=5)
        
        if y_smooth is not None and r2 > 0:
            ax.plot(x_smooth_raw, y_smooth, linestyle='--', color='#d62728', 
                    linewidth=2.5, label=f"Trendline: {name} ($R^2 = {r2:.3f}$)", zorder=6)

        ax.set_title(title, fontsize=14, weight='bold', pad=12)
        ax.set_xlabel(x_col.replace('_', ' '))
        ax.set_ylabel(y_col.replace('_', ' '))
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='best', frameon=True, shadow=True)
        plt.tight_layout()
        safe_title = title.replace(' ', '_').replace(':', '')
        plt.savefig(unified_out_dir / f"Unified_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

        # --- 2. ISOLATED PLOT ---
        plt.figure(figsize=(10, 6.5))
        plt.errorbar(x_plot, y_plot, yerr=y_err, xerr=x_err, fmt='o', color='black', 
                    ecolor='#555555', elinewidth=2, capsize=4, markersize=7, label='Global Mean ± 1 STD', zorder=5)
        
        if y_smooth is not None and r2 > 0:
            plt.plot(x_smooth_raw, y_smooth, linestyle='--', color='#d62728', 
                     linewidth=2.5, label=f"Best Fit: {name} ($R^2 = {r2:.3f}$)", zorder=6)
                     
        plt.title(f"Isolated Global Mean: {title}", fontsize=14, weight='bold', pad=12)
        plt.xlabel(x_col.replace('_', ' '))
        plt.ylabel(y_col.replace('_', ' '))
        plt.legend(loc='best', frameon=True, shadow=True)
        plt.tight_layout()
        plt.savefig(isolated_out_dir / f"Isolated_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

    print(f"\n[SUCCESS] Execution completed. System dynamically adapted to {len(active_objects)} objects.")

if __name__ == "__main__":
    main()