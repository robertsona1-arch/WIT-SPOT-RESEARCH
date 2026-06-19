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

def standardize_columns(df):
    """Aggressively maps variations of Excel headers using a 1-to-1 target lock to prevent line duplication."""
    df.columns = df.columns.astype(str).str.strip()
    renamed_targets = set()
    new_names = {}
    
    for col in df.columns:
        c_lower = col.lower()
        
        # Hard ignore baseline actuals and absolute metrics to prevent multi-line bleeding
        if 'actual' in c_lower or 'absolute' in c_lower:
            continue
            
        if 'volume' in c_lower and 'Volume_m3' not in renamed_targets:
            new_names[col] = 'Volume_m3'
            renamed_targets.add('Volume_m3')
        # Explicit target lock for error mappings
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
        elif 'density' in c_lower and 'time' not in c_lower and 'snap' not in c_lower and 'Density_pts_m3' not in renamed_targets:
            new_names[col] = 'Density_pts_m3'
            renamed_targets.add('Density_pts_m3')
        elif 'point' in c_lower and 'count' in c_lower and 'time' not in c_lower and 'snap' not in c_lower and 'Point_Count' not in renamed_targets:
            new_names[col] = 'Point_Count'
            renamed_targets.add('Point_Count')
            
    df.rename(columns=new_names, inplace=True)
    
    # Ultimate failsafe: strip duplicate columns if any legacy mapping leaked through
    df = df.loc[:, ~df.columns.duplicated()]
    return df

def main():
    parser = argparse.ArgumentParser(description="Compare isolated global run averages across key operating environments.")
    parser.add_argument('--folder', required=True, help="Root directory containing the test condition subfolders")
    parser.add_argument('--name', default="Macro Condition Cross-Test", help="Official Test Name to replace {test name} in titles")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = args.name

    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Core experimental folder names (standing_map_180deg explicitly removed per specifications)
    target_subfolders = [
        'closer',
        'rotating_map',
        'standing_map_0deg',
        'varying_height_0deg'
    ]

    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if not valid_paths:
        print("[CRITICAL] None of the specified subfolders were located.")
        sys.exit(1)

    dashboard_master_data = []

    print("Extracting global dashboard metrics across conditions...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            
            if 'Averages_Dashboard' in xls.sheet_names:
                df_dash = pd.read_excel(xls, sheet_name='Averages_Dashboard')
                df_dash = standardize_columns(df_dash)
                df_dash['Test_Environment'] = env_name
                dashboard_master_data.append(df_dash)
            else:
                print(f"  [WARNING] 'Averages_Dashboard' sheet missing in {file_to_open.name}")
                
        except Exception as e:
            print(f"  [ERROR] Failed to parse {file_to_open.name}: {e}")

    if not dashboard_master_data:
        print("[CRITICAL] No 'Averages_Dashboard' data could be compiled.")
        sys.exit(1)

    combined_dash_df = pd.concat(dashboard_master_data, ignore_index=True)

    # 11-PLOT STANDARDIZED MATRIX
    plots_to_generate = [
        {'y': 'Volume_m3', 'x': 'Snap_Count', 
         'title': r'{test name} Average Volume ($V$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Average Volume ($V$) [m$^3$]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'Volume_vs_SnapCount.png', 'is_vol': True},
         
        {'y': 'Percent_Error', 'x': 'Snap_Count', 
         'title': r'{test name} Average Volume Percent Error ($E$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Volume Percent Error ($E$) [%]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'PercentError_vs_SnapCount.png', 'is_vol': False},
         
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

    output_dir = base_dir / "Cross_Test_Global_Averages"
    output_dir.mkdir(exist_ok=True)
    print(f"\nGenerating standardized global graphs into: {output_dir.name}/")

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
    
    dash_combos = sorted(combined_dash_df['Test_Environment'].unique())
    dash_style_map = {}
    for i, env in enumerate(dash_combos):
        dash_style_map[env] = {'color': colors[i % len(colors)], 'marker': markers[i % len(markers)], 'linestyle': linestyles[i % len(linestyles)]}

    for idx, plot_cfg in enumerate(plots_to_generate, 1):
        y_col = plot_cfg['y']
        x_col = plot_cfg['x']
        safe_title = plot_cfg['fn'].split('.')[0]
        final_title_base = plot_cfg['title'].replace('{test name}', test_name)

        if x_col in combined_dash_df.columns and y_col in combined_dash_df.columns:
            fig, ax = plt.subplots(figsize=(10, 6.5))
            
            for env, group_df in combined_dash_df.groupby('Test_Environment'):
                sorted_df = group_df.sort_values(by=x_col)
                label_str = env.replace('_', ' ').title()
                
                if plot_cfg.get('is_vol'): 
                    label_str += " [Actual: 0.02286 m³]"
                if r'\rho' in plot_cfg['y_lbl'] or 'Density' in plot_cfg['y_lbl']:
                    j_rms = calculate_temporal_jitter(sorted_df[y_col])
                    label_str += f" [RMS Jitter: {j_rms:.2f}]"
                
                style = dash_style_map.get(env)
                ax.plot(sorted_df[x_col], sorted_df[y_col], color=style['color'], linestyle=style['linestyle'],
                        marker=style['marker'], linewidth=3.0, markersize=8, label=label_str)
                        
            ax.set_title(f"[Global Macro View] {final_title_base}", pad=12)
            ax.set_xlabel(plot_cfg['x_lbl'])
            ax.set_ylabel(plot_cfg['y_lbl'])
            ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), framealpha=1.0, edgecolor='black', fontsize=9, handlelength=4.0)
            plt.tight_layout()
            plt.savefig(output_dir / f"Macro_{idx:02d}_{safe_title}.png", dpi=300, bbox_inches='tight')
            plt.close()
        else:
            missing = [c for c in [x_col, y_col] if c not in combined_dash_df.columns]
            print(f"  [SKIP] Macro View Plot '{safe_title}': Missing target columns {missing}")

    print(f"\n[SUCCESS] Macro 11-plot sequence finalized. Axis standardizations are complete.")

if __name__ == "__main__":
    main()