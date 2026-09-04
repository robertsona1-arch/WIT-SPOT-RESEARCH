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
from scipy.spatial import ConvexHull, QhullError

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

from matplotlib.lines import Line2D

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *



def main():
    parser = argparse.ArgumentParser(description="Compile 3-Way Composite Dashboard Analysis.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    
    # IMPORTANT: Verify 'proximity_map' matches your exact 1.5m test folder name
    target_subfolders = ['standing_map_0deg', 'rotating_map', 'closer'] 
    #target_subfolders = ['standing_map_0deg', 'rotating_map']
    
    dashboard_rows = []
    raw_quadrant_rows = []
    
    print("Ingesting dashboard baselines and computing quadrant variance structures...")
    for env_name in target_subfolders:
        env_path = base_dir / env_name
        #excel_files = list(env_path.glob("Total_*.xlsx"))
        excel_files = list(env_path.glob("Total_Averages_SOR_Hull*.xlsx"))
        if not excel_files: 
            print(f"[WARNING] Missing directory or target file: {env_name}")
            continue
        
        xls = pandas.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            
            if sheet_lower == 'averages_dashboard':
                df = pandas.read_excel(xls, sheet_name=sheet)
                df.columns = df.columns.astype(str).str.strip()
                # Dynamically map the columns based on your requested inputs
                rename_map = {}
                for col in df.columns:
                    c_low = col.lower()
                    if 'average density per time' in c_low or 'density per time' in c_low:
                        rename_map[col] = 'Concentration_Rate'
                    elif 'global_mean_percent_error' in c_low or 'global mean percent error' in c_low:
                        rename_map[col] = 'Percent_Error'
                df.rename(columns=rename_map, inplace=True)
                df['Test_Environment'] = env_name
                dashboard_rows.append(df)
            
            elif sheet_lower in ['front_lf_box', 'front_rt_box', 'back_rt_box', 'back_lf_box']:
                df = pandas.read_excel(xls, sheet_name=sheet)
                df = standardize_columns(df)
                df['Test_Environment'] = env_name
                raw_quadrant_rows.append(df)

    if not dashboard_rows or not raw_quadrant_rows:
        print("[CRITICAL] Ingestion failure. Verify sheets exist.")
        sys.exit(1)

    dash_df = pandas.concat(dashboard_rows, ignore_index=True)
    quad_df = pandas.concat(raw_quadrant_rows, ignore_index=True)
    
    output_dir = base_dir / "Composite_System_Analysis_SOR_Con"
    output_dir.mkdir(exist_ok=True)

    # Aligned configuration matrix containing explicit raw mapping definitions for both panels
    panel_cfgs = [
        {
            'y': 'Percent_Error', 
            'x': 'Snap_Count', 
            'raw_y': 'Percent_Error', 
            'raw_x': 'Snap_Count', 
            'y_lbl': r'GlobalAverage Volume Percent Error ($\bar{E}$) [%]', 
            'x_lbl': r'Number of Snapshots ($n$)', 
            'title': '(a)  Global Average Volume Percent Error vs Amount of Snapshots'
        },
        {
            'y': 'Concentration_Rate', 
            'x': 'Time_s', 
            'raw_y': 'Concentration_Rate', 
            'raw_x': 'Time_s', 
            'y_lbl': r'Global Point Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 
            'x_lbl': r'Time ($t$) [s]', 
            'title': '(b) Global Point Concentration Accumulation Rate vs Time'
        }
    ]

    # Integrated precise shapes and hex colors for distinct visual contrast
    env_style_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': 'Static Baseline (3.5m)', 'color': '#ae2012'},
        'rotating_map': {'linestyle': '--', 'marker': '^', 'label': 'Rotational Drive', 'color': '#005f73'},
        'closer': {'linestyle': '-.', 'marker': '*', 'label': 'Proximity Range (1.5m)', 'color': '#ca6702'}
    }

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(1, 2, figsize=(15, 8.5)) 
    fig.suptitle("Composite Operational Limits Analysis", fontsize=22, weight='bold', y=0.98)
    
    legend_handles_err = []
    legend_labels_err = []
    legend_handles_rate = []
    legend_labels_rate = []

    for idx, cfg in enumerate(panel_cfgs):
        ax = axes[idx]
        is_rate = (idx == 1)
        
        for env, group_dash in dash_df.groupby('Test_Environment'):
            if env not in env_style_map: continue
            style = env_style_map[env]
            
            sorted_dash = group_dash.sort_values(by=cfg['x'])
            x_dash = sorted_dash[cfg['x']].to_numpy()
            y_dash = sorted_dash[cfg['y']].to_numpy()
            
            env_quads = quad_df[quad_df['Test_Environment'] == env]
            agg_quads = env_quads.groupby(cfg['raw_x'])[cfg['raw_y']].agg(['std']).reset_index()
            
            merged_stats = pandas.merge(sorted_dash, agg_quads, left_on=cfg['x'], right_on=cfg['raw_x'], how='left')
            y_std = merged_stats['std'].fillna(0).to_numpy()
            
            y_pred, metrics_str = compute_regression_stats(x_dash, y_dash, is_rate)
            
            metric_tag = "Volume Error" if idx == 0 else "Accumulation Rate"
            full_label = f"{style['label']} ({metric_tag})\n[{metrics_str}]"
            
            clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_dash)) * 0.75)
            ax.fill_between(x_dash, y_dash - clamped_std, y_dash + clamped_std, color=style['color'], alpha=0.12)
            
            ax.plot(x_dash, y_dash, color=style['color'], linestyle=style['linestyle'], marker=style['marker'], markersize=8, linewidth=3.5)
            ax.plot(x_dash, y_pred, color=style['color'], linestyle=':', linewidth=2.5)

            # Generate massive handles for the 2x3 independent legend export
            custom_handle = Line2D([0], [0], color=style['color'], linestyle=':', linewidth=3.0, marker=style['marker'], markersize=14, markeredgecolor=style['color'])
            
            if idx == 0:
                legend_handles_err.append(custom_handle)
                legend_labels_err.append(full_label)
            else:
                legend_handles_rate.append(custom_handle)
                legend_labels_rate.append(full_label)

        ax.set_title(cfg['title'], weight='bold', fontsize=18, pad=12)
        ax.set_xlabel(cfg['x_lbl'], fontsize=16, weight='bold')
        ax.set_ylabel(cfg['y_lbl'], fontsize=16, weight='bold')
        ax.tick_params(axis='both', which='major', labelsize=14)

    plt.tight_layout()
    main_save_path = output_dir / "01_Composite_Dashboard.png"
    plt.savefig(main_save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    # --- FIXED 3x2 VERTICAL MATRIX LEGEND EXPORT ---
    # Column 1 (Left): Volume Error      | Column 2 (Right): Accumulation Rate
    # Row 1 (Top):    Rotational Drive  | Row 1 (Top):     Rotational Drive
    # Row 2 (Middle): Proximity Range   | Row 2 (Middle):  Proximity Range
    # Row 3 (Bottom): Static Baseline   | Row 3 (Bottom):  Static Baseline
    # Defensive check to catch directory mismatch bugs before they trigger an IndexError
    print("\n" + str(legend_handles_err))
    print(str(legend_handles_rate))
    if len(legend_handles_err) < 3 or len(legend_handles_rate) < 3:
        print(f"\n[CRITICAL ERROR] Legend matrix construction failed.")
        print(f"Expected 3 target profiles but only ingested {len(legend_handles_err)}.")
        print(f"Check your file directory. Is your proximity folder named exactly as specified in the script?")
        sys.exit(1)

    # --- FIXED 3x2 VERTICAL MATRIX LEGEND EXPORT ---
    legend_fig, legend_ax = plt.subplots(figsize=(14, 4.0)) 
    legend_ax.axis('off')
    
    # Map loop indexes to fit row-by-row population across exactly 2 columns
    # legend_handles_err:  [0]=Baseline, [1]=Rotating, [2]=Proximity
    # legend_handles_rate: [0]=Baseline, [1]=Rotating, [2]=Proximity

    
    matrix_3x2_handles = [
        legend_handles_err[0], legend_handles_err[1],  # Row 1: Rotating
        legend_handles_err[2], legend_handles_rate[0],  # Row 2: Proximity
        legend_handles_rate[1], legend_handles_rate[2]   # Row 3: Baseline
    ]
    
    matrix_3x2_labels = [
        legend_labels_err[0], legend_labels_err[1],
        legend_labels_err[2], legend_labels_rate[0],
        legend_labels_rate[1], legend_labels_rate[2]
    ]
    """
    matrix_2x2_handles = [
        legend_handles_err[0], legend_handles_err[1],  # Row 1: Rotating
        #legend_handles_err[2], legend_handles_rate[0],  # Row 2: Proximity
        legend_handles_rate[1], legend_handles_rate[2]   # Row 3: Baseline
    ]
    
    matrix_2x2_labels = [
        legend_labels_err[0], legend_labels_err[1],
        #legend_labels_err[2], legend_labels_rate[0],
        legend_labels_rate[1], legend_labels_rate[2]
    ]
    """
    legend_ax.legend(
        handles=matrix_3x2_handles, 
        labels=matrix_3x2_labels,
        loc='center', 
        ncol=2,             # Constrains the canvas to exactly 2 columns wide
        fontsize=13,      
        frameon=True,     
        shadow=True,
        edgecolor='black',
        facecolor='#ffffff',
        framealpha=1.0,
        handlelength=4.0,
        columnspacing=3.0,
        labelspacing=1.2    # Adds explicit vertical breathing room between your stacked rows
    )
    
    legend_save_path = output_dir / "01_Composite_Dashboard_3x2_Legend.png"
    legend_fig.savefig(legend_save_path, bbox_inches='tight', dpi=300, transparent=True)
    plt.close(legend_fig)
    print(f"\n[SUCCESS] Custom 3x2 Matrix Legend compiled -> {legend_save_path.name}")

if __name__ == "__main__":
    main()