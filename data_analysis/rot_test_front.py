import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
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
import grpc
from google.protobuf import wrappers_pb2 as wrappers

# Custom library imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *

# Suppress warnings
warnings.filterwarnings('ignore')



def main():
    parser = argparse.ArgumentParser(description="Generate 11 standardized IEEE publication plots for Test 4 (Yaw Sweep).")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    parser.add_argument('--name', default="Test 4 Yaw Sweep Analysis", help="Official Test Name to replace {test name} in titles")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = args.name

    static_folder = base_dir / "standing_map_0deg"
    rotating_folder = base_dir / "rotating_map"

    if not static_folder.exists() or not rotating_folder.exists():
        print("[CRITICAL] Mandatory baseline folders missing. Check execution target directory path.")
        return

    compiled_runs = []
    target_conditions = {
        'Static Baseline (0° Heading)': static_folder,
        'Dynamic Yaw Rotation Sweep': rotating_folder
    }

    print("Executing dynamic string-match parsing pass for primitives...")
    for env_name, path in target_conditions.items():
        excel_files = list(path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            
            # 1. Ingest Global Dashboard Averages (if applicable)
            if 'Averages_Dashboard' in xls.sheet_names:
                df_dash = pd.read_excel(xls, sheet_name='Averages_Dashboard')
                df_dash = standardize_columns(df_dash)
                df_dash['Operational_Condition'] = env_name
                df_dash['Primitive_Target'] = 'Global Average'
                compiled_runs.append(df_dash)

            # 2. Ingest Specific Target Quadrants
            sheets_to_ignore = ['master_data', 'averages_dashboard', 'regression_metrics']
            for sheet in xls.sheet_names:
                sheet_lower = sheet.lower().strip()
                if sheet_lower in sheets_to_ignore or 'mid' in sheet_lower or 'center' in sheet_lower:
                    continue
                
                # Broadened inclusion to match Test 4 scope (front boxes)
                if 'front' in sheet_lower:
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df = standardize_columns(df)
                    
                    df['Operational_Condition'] = env_name
                    df['Primitive_Target'] = sheet
                    
                    if 'lf' in sheet_lower or 'left' in sheet_lower:
                        df['Primitive_Target'] = 'Front Left Box'
                    elif 'rt' in sheet_lower or 'right' in sheet_lower:
                        df['Primitive_Target'] = 'Front Right Box'
                        
                    compiled_runs.append(df)
                    
        except Exception as e:
            print(f"  [ERROR] Processing crash on workbook {file_to_open.name}: {e}")

    if not compiled_runs:
        print("[CRITICAL] Dataframe compilation vector is completely empty. Verify tab spellings.")
        return

    master_df = pd.concat(compiled_runs, ignore_index=True)
    output_dir = base_dir / "Test4_Comprehensive_Plots"
    output_dir.mkdir(exist_ok=True)

    # 11-PLOT STANDARDIZED MATRIX
    plots_to_generate = [
        {'y': 'Volume_m3', 'x': 'Time_s', 
         'title': r'{test name} Average Volume ($V$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Average Volume ($V$) [m$^3$]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'Volume_vs_Time.png', 'is_vol': True},
         
        {'y': 'Percent_Error', 'x': 'Time_s', 
         'title': r'{test name} Average Volume Percent Error ($E$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Volume Percent Error ($E$) [%]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'PercentError_vs_Time.png', 'is_vol': False},
         
        {'y': 'Point_Count', 'x': 'Snap_Count', 
         'title': r'{test name} Average Total Point Count ($P$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Total Point Count ($P$) [pts]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'PointCount_vs_SnapCount.png', 'is_vol': False},
         
        {'y': 'Density_pts_m3', 'x': 'Snap_Count', 
         'title': r'{test name} Average Point Density ($\rho$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Point Density ($\rho$) [pts/m$^3$]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'Density_vs_SnapCount.png', 'is_vol': False},
         
        {'y': 'Point_Count', 'x': 'Time_s', 
         'title': r'{test name} Average Total Point Count ($P$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Total Point Count ($P$) [pts]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'PointCount_vs_Time.png', 'is_vol': False},
         
        {'y': 'Density_pts_m3', 'x': 'Time_s', 
         'title': r'{test name} Average Point Density ($\rho$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Point Density ($\rho$) [pts/m$^3$]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'Density_vs_Time.png', 'is_vol': False},
         
        {'y': 'Snap_Count', 'x': 'Time_s', 
         'title': r'{test name} Amount of Snapshots ($n$) vs Total Latency Time ($t$)', 
         'y_lbl': r'Amount of Snapshots ($n$)', 'x_lbl': r'Total Latency Time ($t$) [s]', 'fn': 'SnapCount_vs_Time.png', 'is_vol': False},
         
        {'y': 'Points_Per_Snap', 'x': 'Snap_Count', 
         'title': r'{test name} Point Capture per Snapshot ($\Delta P$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Point Capture Rate ($\Delta P$) [pts/n]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'PointsPerSnap_vs_SnapCount.png', 'is_vol': False},
         
        {'y': 'Points_Per_Time', 'x': 'Time_s', 
         'title': r'{test name} Point Capture per Second ($\dot{P}$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Point Capture Rate ($\dot{P}$) [pts/s]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'PointsPerTime_vs_Time.png', 'is_vol': False},
         
        {'y': 'Density_Per_Snap', 'x': 'Snap_Count', 
         'title': r'{test name} Density Increase per Snapshot ($\Delta \rho$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Density Increase Rate ($\Delta \rho$) [pts/m$^3$/n]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'DensityPerSnap_vs_SnapCount.png', 'is_vol': False},
         
        {'y': 'Density_Per_Time', 'x': 'Time_s', 
         'title': r'{test name} Density Increase per Second ($\dot{\rho}$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Density Increase Rate ($\dot{\rho}$) [pts/m$^3$/s]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'DensityPerTime_vs_Time.png', 'is_vol': False}
    ]

    print(f"\nExporting high-contrast standardized figures into: {output_dir.name}/")

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.labelweight': 'bold',
        'axes.titlesize': 13,
        'axes.titleweight': 'bold',
        'legend.frameon': True,
        'legend.edgecolor': '#000000',
        'legend.facecolor': '#ffffff'
    })

    # --- GRAYSCALE IEEE COMPLIANCE ALLOCATION ---
    markers = ['o', 's', '^', 'v', 'D', 'X', 'P']
    linestyles = ['--', '-.', ':', '--', '-.', ':']
    colors = ['#111111', '#333333', '#555555', '#777777', '#222222', '#444444'] 
    
    active_combos = sorted(master_df.groupby(['Operational_Condition', 'Primitive_Target']).groups.keys())
    style_map = {}
    for i, combo in enumerate(active_combos):
        style_map[combo] = {'color': colors[i % len(colors)], 'marker': markers[i % len(markers)], 'linestyle': linestyles[i % len(linestyles)]}

    for idx, plot_cfg in enumerate(plots_to_generate, 1):
        y_attr = plot_cfg['y']
        x_attr = plot_cfg['x']
        safe_title = plot_cfg['fn'].split('.')[0]
        final_title_base = plot_cfg['title'].replace('{test name}', test_name)
        
        if y_attr not in master_df.columns or x_attr not in master_df.columns:
            missing = [c for c in [x_attr, y_attr] if c not in master_df.columns]
            print(f"  [SKIP] Plot '{safe_title}': Missing target columns {missing}")
            continue
            
        fig, ax = plt.subplots(figsize=(10, 6.2))
        
        for (cond, prim), group_data in master_df.groupby(['Operational_Condition', 'Primitive_Target']):
            sorted_data = group_data.sort_values(by=x_attr)
            
            label_str = f"{prim} ({cond.split(' (')[0]})"
            if plot_cfg.get('is_vol') and 'Global' not in prim: 
                label_str += " [Actual: 0.02286 m³]"
            
            if r'\rho' in plot_cfg['y_lbl'] or 'Density' in plot_cfg['y_lbl']:
                j_rms = calculate_temporal_jitter(sorted_data[y_attr])
                label_str += f" [RMS Jitter: {j_rms:.2f}]"
            
            style = style_map.get((cond, prim))
            
            ax.plot(
                sorted_data[x_attr], sorted_data[y_attr],
                label=label_str,
                color=style['color'],
                marker=style['marker'],
                linestyle=style['linestyle'],
                linewidth=2.2,
                markersize=6,
                alpha=0.9
            )
            
        ax.set_title(final_title_base, pad=12)
        ax.set_ylabel(plot_cfg['y_lbl'])
        ax.set_xlabel(plot_cfg['x_lbl'])
        
        # Enforced wide-legend handling
        ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), framealpha=1.0, edgecolor='black', handlelength=4.0)
        
        plt.tight_layout()
        save_path = output_dir / plot_cfg['fn']
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()

    print("\nGlobal data plotting sequence finalized.")

if __name__ == "__main__":
    main()