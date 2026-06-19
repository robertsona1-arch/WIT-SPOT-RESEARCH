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
    """Aggressively maps variations of Excel headers using a unique 1-to-1 target lock."""
    df.columns = df.columns.astype(str).str.strip()
    renamed_targets = set()
    new_names = {}
    
    for col in df.columns:
        c_lower = col.lower()
        
        # Hard ignore baseline actuals and absolute metrics
        if 'actual' in c_lower or 'absolute' in c_lower:
            continue
            
        if 'volume' in c_lower and 'Volume_m3' not in renamed_targets:
            new_names[col] = 'Volume_m3'
            renamed_targets.add('Volume_m3')
        elif ('global_mean_percent_error' in c_lower or 'global mean percent error' in c_lower) and 'Percent_Error' not in renamed_targets:
            new_names[col] = 'Percent_Error'
            renamed_targets.add('Percent_Error')
        elif 'snap' in c_lower and 'count' in c_lower and 'Snap_Count' not in renamed_targets:
            new_names[col] = 'Snap_Count'
            renamed_targets.add('Snap_Count')
        elif c_lower in ['time', 'time_s', 'time_sec', 'time (s)'] and 'Time_s' not in renamed_targets:
            new_names[col] = 'Time_s'
            renamed_targets.add('Time_s')
        elif ('average density per time' in c_lower or 'density per time' in c_lower) and 'Concentration_Rate' not in renamed_targets:
            new_names[col] = 'Concentration_Rate'
            renamed_targets.add('Concentration_Rate')
        elif ('density: pts per m3' in c_lower or 'average density' in c_lower) and 'Concentration_Value' not in renamed_targets:
            new_names[col] = 'Concentration_Value'
            renamed_targets.add('Concentration_Value')
        elif 'point' in c_lower and 'count' in c_lower and 'Point_Count' not in renamed_targets:
            new_names[col] = 'Point_Count'
            renamed_targets.add('Point_Count')
            
    df.rename(columns=new_names, inplace=True)
    df = df.loc[:, ~df.columns.duplicated()]
    return df

def main():
    parser = argparse.ArgumentParser(description="Analyze directional sensor bias using standardized color-coded primitive tracking for RP1-RP4.")
    parser.add_argument('--name', default="Test 2 Directional Bias", help="Official Test Name to replace {test name} in titles")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = args.name

    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    target_subfolders = ['standing_map_0deg', 'standing_map_180deg']
    
    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if len(valid_paths) < 2:
        print("[CRITICAL] Directional bias analysis requires BOTH standing_map_0deg and standing_map_180deg folders.")
        sys.exit(1)

    box_data_compiled = []

    print("Extracting primitive datasets and isolating targets RP1 through RP4...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            
            for sheet in xls.sheet_names:
                sheet_lower = sheet.lower().strip()
                
                # Check for explicit match against your exact Excel workbook tab names for the four corners
                is_matched = True
                if sheet_lower == 'front_lf_box':
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df = standardize_columns(df)
                    df['Quadrant_Class'] = r"Prismatic Target $RP_1$"
                elif sheet_lower == 'front_rt_box':
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df = standardize_columns(df)
                    df['Quadrant_Class'] = r"Prismatic Target $RP_2$"
                elif sheet_lower == 'back_rt_box':
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df = standardize_columns(df)
                    df['Quadrant_Class'] = r"Prismatic Target $RP_3$"
                elif sheet_lower == 'back_lf_box':
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df = standardize_columns(df)
                    df['Quadrant_Class'] = r"Prismatic Target $RP_4$"
                else:
                    is_matched = False
                
                if is_matched:
                    df['Test_Environment'] = env_name
                    box_data_compiled.append(df)
                    
        except Exception as e:
            print(f"[ERROR] Failed to parse box data from {file_to_open.name}: {e}")

    if not box_data_compiled:
        print("[CRITICAL] No matching quadrant box tabs found in the target files.")
        sys.exit(1)

    combined_box_df = pd.concat(box_data_compiled, ignore_index=True)

    # 3-PLOT ISOLATED MATRIX
    plots_to_generate = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 
         'title': r'{test name} Average Volume Percent Error ($E$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Volume Percent Error ($E$) [%]', 'x_lbl': r'Amount of Snapshots ($n$)', 
         'fn': 'Primitive_01_PercentError_vs_SnapCount.png', 'legend_loc': 'top_right'},
         
        {'y': 'Concentration_Value', 'x': 'Time_s', 
         'title': r'{test name} Point Concentration ($C$) vs Map Capture Time ($t$)', 
         'y_lbl': r'Point Concentration ($C$) [pts/m$^3$]', 'x_lbl': r'Map Capture Time ($t$) [s]', 
         'fn': 'Primitive_02_Concentration_vs_Time.png', 'legend_loc': 'top_left'},
         
        {'y': 'Concentration_Rate', 'x': 'Time_s', 
         'title': r'{test name} Concentration Accumulation Rate ($\dot{C}$) vs Map Capture Time ($t$)', 
         'y_lbl': r'Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Map Capture Time ($t$) [s]', 
         'fn': 'Primitive_03_ConcentrationRate_vs_Time.png', 'legend_loc': 'top_right'}
    ]

    output_dir = base_dir / "Directional_Bias_Analysis"
    output_dir.mkdir(exist_ok=True)
    print(f"\nGenerating standardized color-coded figures into: {output_dir.name}/")

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

    # --- BIFURCATED STYLING MATRIX ---
    # Colors lock to the Primitive geometry. Line styles lock to the Test Environment.
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] 
    quadrants = sorted(combined_box_df['Quadrant_Class'].unique())
    color_map = {q: colors[i % len(colors)] for i, q in enumerate(quadrants)}
    
    env_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': '0°'},
        'standing_map_180deg': {'linestyle': '--', 'marker': 's', 'label': '180°'}
    }

    for idx, plot_cfg in enumerate(plots_to_generate, 1):
        y_col = plot_cfg['y']
        x_col = plot_cfg['x']
        
        if x_col not in combined_box_df.columns or y_col not in combined_box_df.columns:
            missing = [c for c in [x_col, y_col] if c not in combined_box_df.columns]
            print(f"  [SKIP] Plot '{plot_cfg['fn']}': Missing target columns {missing}")
            continue

        fig, ax = plt.subplots(figsize=(10, 6.5))

        for (quadrant, env), group_df in combined_box_df.groupby(['Quadrant_Class', 'Test_Environment']):
            # Filter out single-point standard deviation jumps or NaN math lines
            sorted_df = group_df.sort_values(by=x_col)
            
            c_color = color_map.get(quadrant)
            e_style = env_map.get(env)
            
            label_string = f"{quadrant} ({e_style['label']})"
            
            if r'C' in plot_cfg['y_lbl'] or 'Concentration' in plot_cfg['y_lbl']:
                j_rms = calculate_temporal_jitter(sorted_df[y_col])
                label_string += f" [RMS Jitter: {j_rms:.2f}]"
            
            ax.plot(
                sorted_df[x_col], sorted_df[y_col],
                color=c_color,
                linestyle=e_style['linestyle'],
                marker=e_style['marker'],
                linewidth=2.2,
                markersize=6,
                alpha=0.85,
                label=label_string
            )

        final_title = plot_cfg['title'].replace('{test name}', test_name)
        ax.set_title(final_title, pad=12)
        ax.set_xlabel(plot_cfg['x_lbl'])
        ax.set_ylabel(plot_cfg['y_lbl'])
        
        if idx == 1:
            # Graph 1 (Error vs Snap Count): Force directly UNDER the plot canvas axis
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2, framealpha=1.0, edgecolor='black', fontsize=9, handlelength=4.0)
        elif idx == 2:
            # Graph 2 (Concentration vs Time): Force inside the TOP LEFT corner of the inner grid
            ax.legend(loc='upper left', framealpha=1.0, edgecolor='black', fontsize=9, handlelength=4.0)
        elif idx == 3:
            # Graph 3 (Concentration Rate vs Time): Force inside the TOP RIGHT corner of the inner grid
            ax.legend(loc='upper right', framealpha=1.0, edgecolor='black', fontsize=9, handlelength=4.0)

        plt.tight_layout()

        plt.savefig(output_dir / f"Bias_{idx:02d}_{plot_cfg['fn']}", dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  [SAVED] -> Bias_{idx:02d}_{plot_cfg['fn']}")

    print(f"\n[SUCCESS] Directional mapping finalized. RP1 through RP4 isolated successfully.")

if __name__ == "__main__":
    main()