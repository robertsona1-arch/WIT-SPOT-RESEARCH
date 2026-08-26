"""
Creates graph of RP1-RP4 directional bias using standardized color-coded primitive tracking.
Creates legend as seperate png

"""

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
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] 

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
    parser = argparse.ArgumentParser(description="Analyze directional sensor bias using standardized color-coded primitive tracking.")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = "Directional Bias Analysis"

    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Updated to pull the rotating dataset instead of 180deg
    target_subfolders = ['standing_map_0deg', 'rotating_map']
    
    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if len(valid_paths) < 2:
        print(f"[CRITICAL] Analysis requires BOTH {target_subfolders[0]} and {target_subfolders[1]} folders.")
        sys.exit(1)

    box_data_compiled = []

    # Refactored dictionary mapping for clean parsing. 
    # YOU MUST UPDATE the placeholder keys with your exact Excel sheet names.
    sheet_mapping = {
        'front_lf_box': r"Prismatic Target $RP_1$",
        'front_rt_box': r"Prismatic Target $RP_2$",
        'back_rt_box':  r"Prismatic Target $RP_3$",
        'back_lf_box':  r"Prismatic Target $RP_4$",
        'mid_lf_box': r"Prismatic Target $RP_5$",
        'mid_rt_box': r"Prismatic Target $RP_6$",
        'circ_chair': r"Cylindrical Target $CY_1$",
        'roomba': r"Cylindrical Target $CY_2$"
    }

    print("Extracting primitive datasets and applying target classifications...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            
            for sheet in xls.sheet_names:
                sheet_lower = sheet.lower().strip()
                
                # O(1) lookup replaces the long if/elif chain
                if sheet_lower in sheet_mapping:
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df = standardize_columns(df)
                    df['Quadrant_Class'] = sheet_mapping[sheet_lower]
                    df['Test_Environment'] = env_name
                    box_data_compiled.append(df)
                    
        except Exception as e:
            print(f"[ERROR] Failed to parse box data from {file_to_open.name}: {e}")

    if not box_data_compiled:
        print("[CRITICAL] No matching quadrant box tabs found in the target files.")
        sys.exit(1)

    combined_box_df = pd.concat(box_data_compiled, ignore_index=True)

    # Define the two graph groupings
    graphs_to_generate = [
        {
            'id': 'Group1_RP1_RP2_RP3_RP4',
            'targets': [
                r"Prismatic Target $RP_1$", r"Prismatic Target $RP_2$", 
                r"Prismatic Target $RP_3$", r"Prismatic Target $RP_4$"
            ]
        },
        {
            'id': 'Group2_CY1_RP5_CY2_RP6',
            'targets': [
                r"Cylindrical Target $CY_1$", r"Prismatic Target $RP_5$", 
                r"Cylindrical Target $CY_2$", r"Prismatic Target $RP_6$"
            ]
        }
    ]

    output_dir = base_dir / "Rotational_Impact_Analysis"
    output_dir.mkdir(exist_ok=True)
    print(f"\nGenerating standardized color-coded figures into: {output_dir.name}/")

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.labelweight': 'bold',
        'axes.titlesize': 13,
        'axes.titleweight': 'bold'
    })

    # Keep colors consistent across all possible targets
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    all_quadrants = sorted(list(sheet_mapping.values()))
    color_map = {q: colors[i % len(colors)] for i, q in enumerate(all_quadrants)}
    
    # Updated env_map to reflect the rotating dataset
    env_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': '0° (Static)'},
        'rotating_map': {'linestyle': '--', 'marker': '^', 'label': 'Rotating'}
    }

    # Core plotting configuration
    y_col, x_col = 'Percent_Error', 'Snap_Count'
    x_lbl = r'Amount of Snapshots ($n$)'
    y_lbl = r'Average Volume Percent Error ($\bar{E}$) [%]'

    for graph_cfg in graphs_to_generate:
        # Filter dataframe for the specific graph's targets
        plot_df = combined_box_df[combined_box_df['Quadrant_Class'].isin(graph_cfg['targets'])]
        
        if plot_df.empty:
            print(f"  [SKIP] Graph '{graph_cfg['id']}': No data found for specified targets.")
            continue

        fig, ax = plt.subplots(figsize=(10, 6.5))

        for (quadrant, env), group_df in plot_df.groupby(['Quadrant_Class', 'Test_Environment']):
            sorted_df = group_df.sort_values(by=x_col)
            
            c_color = color_map.get(quadrant)
            e_style = env_map.get(env)
            
            label_string = f"{quadrant} ({e_style['label']})"
            
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

        fig.suptitle('Directional Bias Analysis', fontsize=16, fontweight='bold')
        ax.set_title(f"Target Subset: {graph_cfg['id'].replace('Group1_', '').replace('Group2_', '').replace('_', ', ')}", fontsize=12)
        ax.set_xlabel(x_lbl)
        ax.set_ylabel(y_lbl)
        
        # Save the main plot WITHOUT a legend
        plt.tight_layout()
        main_plot_path = output_dir / f"Bias_{graph_cfg['id']}_PercentError_vs_SnapCount.png"
        plt.savefig(main_plot_path, dpi=300, bbox_inches='tight')
        print(f"  [SAVED] -> {main_plot_path.name}")
        
        # --- ISOLATED LEGEND EXPORT ---
        print(f"  [EXPORTING] Generating isolated legend for {graph_cfg['id']}...")
        handles, labels = ax.get_legend_handles_labels()
        
        legend_fig, legend_ax = plt.subplots(figsize=(8, 1.5))
        legend_ax.axis('off') 
        
        legend = legend_ax.legend(
            handles=handles, 
            labels=labels,
            loc='center', 
            ncol=2,            
            fontsize=14,      
            frameon=True,     
            shadow=True,
            edgecolor='black',
            facecolor='#ffffff',
            framealpha=1.0
        )
        
        legend_path = output_dir / f"Bias_{graph_cfg['id']}_Isolated_Legend.png"
        legend_fig.savefig(legend_path, bbox_inches='tight', dpi=300, transparent=True)
        plt.close(legend_fig)
        print(f"  [SAVED] -> {legend_path.name}")
        
        plt.close(fig) # Close main figure

    print(f"\n[SUCCESS] Directional mapping finalized. Multiple target graphs condensed.")

if __name__ == "__main__":
    main()