import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import warnings
from scipy.optimize import curve_fit

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

def offset_power_law(x, a, b, c):
    """Mathematical constraint model for steady-state accumulation bounds."""
    return a * (x ** b) + c

def main():
    parser = argparse.ArgumentParser(description="Analyze rotational sensor bias with dual-pass primitive and mathematical legend overlays.")
    parser.add_argument('--name', default="Test 4 Rotational Impact", help="Official Test Name to replace {test name} in titles")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = args.name

    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    target_subfolders = ['standing_map_0deg', 'rotating_map']
    
    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if len(valid_paths) < 2:
        print("[CRITICAL] Rotation impact analysis requires BOTH standing_map_0deg and rotating_map folders.")
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

    # 3-PLOT MATRIX DEFINITIONS
    plots_to_generate = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 
         'title': r'{test name} Volume Percent Error ($E$) vs Snapshots ($n$)', 
         'y_lbl': r'Volume Percent Error ($E$) [%]', 'x_lbl': r'Amount of Snapshots ($n$)', 
         'fn': '01_PercentError_vs_SnapCount.png', 'idx': 1, 'legend_loc': 'bottom'},
         
        {'y': 'Concentration_Value', 'x': 'Time_s', 
         'title': r'{test name} Point Concentration ($C$) vs Capture Time ($t$)', 
         'y_lbl': r'Point Concentration ($C$) [pts/m$^3$]', 'x_lbl': r'Map Capture Time ($t$) [s]', 
         'fn': '02_Concentration_vs_Time.png', 'idx': 2, 'legend_loc': 'bottom'},
         
        {'y': 'Concentration_Rate', 'x': 'Time_s', 
         'title': r'{test name} Accumulation Rate ($\dot{C}$) vs Capture Time ($t$)', 
         'y_lbl': r'Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Map Capture Time ($t$) [s]', 
         'fn': '03_ConcentrationRate_vs_Time.png', 'idx': 3, 'legend_loc': 'top_right'}
    ]

    output_dir = base_dir / "Rotational_Impact_Analysis"
    prim_dir = output_dir / "Primitive_Comparisons"
    avg_dir = output_dir / "Global_Average_Comparisons"
    prim_dir.mkdir(parents=True, exist_ok=True)
    avg_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nInitializing dual-pass rendering engine with regression matrix legends...")

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
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] 
    quadrants = sorted(combined_box_df['Quadrant_Class'].unique())
    color_map = {q: colors[i % len(colors)] for i, q in enumerate(quadrants)}
    
    env_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': 'Static 0°'},
        'rotating_map': {'linestyle': '--', 'marker': 's', 'label': 'Rotating'}
    }

    # ==========================================
    # PASS 1: CLEAN PRIMITIVE GRAPHS
    # ==========================================
    for plot_cfg in plots_to_generate:
        y_col = plot_cfg['y']
        x_col = plot_cfg['x']
        idx = plot_cfg['idx']
        
        if x_col not in combined_box_df.columns or y_col not in combined_box_df.columns:
            continue

        fig, ax = plt.subplots(figsize=(10, 6.5))

        for (quadrant, env), group_df in combined_box_df.groupby(['Quadrant_Class', 'Test_Environment']):
            sorted_df = group_df.sort_values(by=x_col)
            c_color = color_map.get(quadrant)
            e_style = env_map.get(env)
            
            label_string = f"{quadrant} ({e_style['label']})"
            
            ax.plot(
                sorted_df[x_col], sorted_df[y_col],
                color=c_color, linestyle=e_style['linestyle'], marker=e_style['marker'],
                linewidth=2.2, markersize=6, alpha=0.85, label=label_string
            )

        final_title = f"[Primitives] {plot_cfg['title'].replace('{test name}', test_name)}"
        ax.set_title(final_title, pad=12)
        ax.set_xlabel(plot_cfg['x_lbl'])
        ax.set_ylabel(plot_cfg['y_lbl'])
        
        if plot_cfg['legend_loc'] == 'bottom':
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2, fontsize=9)
        else:
            ax.legend(loc='upper right', fontsize=9)

        plt.tight_layout()
        plt.savefig(prim_dir / f"Primitive_{plot_cfg['fn']}", dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  [SAVED] -> Primitive_{plot_cfg['fn']}")

    # ==========================================
    # PASS 2: GLOBAL AVERAGES & EXPORTED CSV STATS
    # ==========================================
    avg_color_map = {'standing_map_0deg': '#1f77b4', 'rotating_map': '#d62728'}
    master_regression_stats = []

    for plot_cfg in plots_to_generate:
        y_col = plot_cfg['y']
        x_col = plot_cfg['x']
        idx = plot_cfg['idx']
        
        if x_col not in combined_box_df.columns or y_col not in combined_box_df.columns:
            continue

        fig, ax = plt.subplots(figsize=(10, 6.5))

        for env, group_df in combined_box_df.groupby('Test_Environment'):
            agg_df = group_df.groupby(x_col)[y_col].agg(['mean', 'std']).reset_index()
            agg_df['std'] = agg_df['std'].fillna(0)
            
            x_data = agg_df[x_col].to_numpy()
            y_mean = agg_df['mean'].to_numpy()
            y_std = agg_df['std'].to_numpy()
            
            e_style = env_map.get(env)
            c_color = avg_color_map.get(env)
            
            # Analytics: Extract Jitter/Ripple
            try:
                j_rms = calculate_temporal_jitter(y_mean)
            except NameError:
                j_rms = np.std(np.diff(y_mean))
            var_val = np.var(y_mean)
            
            # Structural Regression Pipeline
            if idx in [1, 2]:
                # OLS Linear Fit
                slope, intercept = np.polyfit(x_data, y_mean, 1)
                y_pred = slope * x_data + intercept
                
                ss_res = np.sum((y_mean - y_pred)**2)
                ss_tot = np.sum((y_mean - np.mean(y_mean))**2)
                r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
                
                eq_str = f"y = {slope:.2f}x + {intercept:.2f} ($R^2 = {r2_val:.3f}$)"
                
            elif idx == 3:
                # Upgraded Non-Linear Least Squares with strict boundary shields
                try:
                    # Initial Seeds: a=500 (scaling), b=-0.8 (steep decay), c=150 (horizontal floor baseline)
                    initial_seeds = [500.0, -0.8, 150.0]
                    
                    # Prevent parameters from drifting to infinity:
                    # Lower bounds: [a_min=0.1,  b_min=-3.0, c_min=0.0]
                    # Upper bounds: [a_max=5000, b_max=-0.01, c_max=500]
                    parameter_bounds = ([0.1, -3.0, 0.0], [5000.0, -0.01, 500.0])
                    
                    popt, _ = curve_fit(offset_power_law, x_data, y_mean, 
                                        p0=initial_seeds, bounds=parameter_bounds, maxfev=10000)
                    a, b, c = popt
                    y_pred = offset_power_law(x_data, *popt)
                    
                    ss_res = np.sum((y_mean - y_pred)**2)
                    ss_tot = np.sum((y_mean - np.mean(y_mean))**2)
                    r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
                    
                    eq_str = f"y = {a:.1f}x^{{{b:.2f}}} + {c:.1f} ($R^2 = {r2_val:.3f}$)"
                except Exception as e:
                    print(f"[WARNING] Fit divergence on {env}: {e}. Applying fallback linear model.")
                    # Safe Fallback Model: Use OLS linear if it still fails to prevent script failure
                    slope, intercept = np.polyfit(x_data, y_mean, 1)
                    y_pred = slope * x_data + intercept
                    ss_res = np.sum((y_mean - y_pred)**2)
                    ss_tot = np.sum((y_mean - np.mean(y_mean))**2)
                    r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
                    eq_str = f"y = {slope:.2f}x + {intercept:.2f} ($R^2 = {r2_val:.3f}$)"
            
            # Variance Band Shading (Clamped 75%)
            clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_mean)) * 0.75)
            
            ax.fill_between(x_data, y_mean - clamped_std, y_mean + clamped_std, color=c_color, alpha=0.15)
            ax.plot(x_data, y_mean, color=c_color, linestyle=e_style['linestyle'],
                    marker=e_style['marker'], linewidth=3.0, markersize=8, alpha=0.8, 
                    label=f"Avg ({e_style['label']}) [Ripple: {j_rms:.2f}]")
            ax.plot(x_data, y_pred, color=c_color, linestyle=':', linewidth=2.5, 
                    label=f"Fit: {eq_str}")
            
            # Append metrics for fallback CSV export
            master_regression_stats.append({
                'Metric': plot_cfg['fn'].split('_vs_')[0],
                'Condition': e_style['label'],
                'Equation': eq_str.replace('$', '').replace('{', '').replace('}', ''),
                'Ripple': round(j_rms, 2),
                'Variance': round(var_val, 4)
            })

        final_title = f"[Global Averages] {plot_cfg['title'].replace('{test name}', test_name)}"
        ax.set_title(final_title, pad=12)
        ax.set_xlabel(plot_cfg['x_lbl'])
        ax.set_ylabel(plot_cfg['y_lbl'])
        
        y_min, y_max = ax.get_ylim()
        if y_max > np.nanmax(combined_box_df[y_col]) * 1.2:
            ax.set_ylim(top=np.nanmax(combined_box_df[y_col]) * 1.2)
        
        if plot_cfg['legend_loc'] == 'bottom':
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2, fontsize=9)
        else:
            ax.legend(loc='upper right', fontsize=9)

        plt.tight_layout()
        plt.savefig(avg_dir / f"GlobalAvg_Analyzed_{plot_cfg['fn']}", dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  [SAVED] -> GlobalAvg_Analyzed_{plot_cfg['fn']}")

    # Compile and export regression metrics natively
    stats_df = pd.DataFrame(master_regression_stats)
    csv_path = avg_dir / "Global_Averages_Regression_Metrics.csv"
    stats_df.to_csv(csv_path, index=False)
    
    print("\n[SUCCESS] Unified pipeline execution complete. Primitive and Global Average graphs generated.")
    print(f"[EXPORT] Regression statistics logged to: {csv_path.name}\n")
    print(stats_df.to_string(index=False))

if __name__ == "__main__":
    main()