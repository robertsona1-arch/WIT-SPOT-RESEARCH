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
    parser = argparse.ArgumentParser(description="Generate Grayscale-Compliant IEEE metrics with Jitter Analysis.")
    parser.add_argument('--name', default="Test Condition", help="Official Test Name to replace {test name} in titles")
    parser.add_argument('--file', required=True, help="Path to Total_Averages_Group_X.xlsx")
    
    args = parser.parse_args()
    
    file_path = Path(args.file)
    test_name = args.name

    xls = pandas.ExcelFile(file_path)
    sheets_to_ignore = ['Averages_Dashboard', 'Regression_Metrics', 'Master_Data']
    object_data = []
    
    for sheet in xls.sheet_names:
        if sheet in sheets_to_ignore: continue
        df_obj = pandas.read_excel(xls, sheet_name=sheet)
        
        # Standardize nomenclature from legacy scripts dynamically
        if 'Density: Pts Per m3' in df_obj.columns: df_obj.rename(columns={'Density: Pts Per m3': 'Density_pts_m3'}, inplace=True)
        if 'Time' in df_obj.columns: df_obj.rename(columns={'Time': 'Time_s'}, inplace=True)
        if 'Points Per Snapshot' in df_obj.columns: df_obj.rename(columns={'Points Per Snapshot': 'Points_Per_Snap'}, inplace=True)
        if 'Points Per Time' in df_obj.columns: df_obj.rename(columns={'Points Per Time': 'Points_Per_Time'}, inplace=True)
        if 'Density Per Time' in df_obj.columns: df_obj.rename(columns={'Density Per Time': 'Density_Per_Time'}, inplace=True)
        if 'Density Per Snapshot' in df_obj.columns: df_obj.rename(columns={'Density Per Snapshot': 'Density_Per_Snap'}, inplace=True)
        
        df_obj['Object_Name'] = sheet
        object_data.append(df_obj)

    if not object_data:
        print("[CRITICAL] No object sheets found. Check input file.")
        return

    combined_objects_df = pandas.concat(object_data, ignore_index=True)

    # EXACT Metric Mapping strictly utilizing raw LaTeX Strings for publication formatting
    plots_to_generate = [
        {'y': 'Volume_m3', 'x': 'Snap_Count', 
         'title': r'{test name} Average Primitive Volume ($V$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Average Volume ($V$) [m$^3$]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'Volume_vs_SnapCount.png'},
         
        {'y': 'Percent_Error', 'x': 'Snap_Count', 
         'title': r'{test name} Average Volume Percent Error ($E$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Average Volume Percent Error ($E$) [%]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'PercentError_vs_SnapCount.png'},
         
        {'y': 'Point_Count', 'x': 'Snap_Count', 
         'title': r'{test name} Average Total Point Count ($P$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Total Point Count ($P$) [pts]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'PointCount_vs_SnapCount.png'},
         
        {'y': 'Density_pts_m3', 'x': 'Snap_Count', 
         'title': r'{test name} Average Point Density ($\rho$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Point Density ($\rho$) [pts/m$^3$]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'Density_vs_SnapCount.png'},
         
        {'y': 'Point_Count', 'x': 'Time_s', 
         'title': r'{test name} Average Total Point Count ($P$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Total Point Count ($P$) [pts]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'PointCount_vs_Time.png'},
         
        {'y': 'Density_pts_m3', 'x': 'Time_s', 
         'title': r'{test name} Average Point Density ($\rho$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Point Density ($\rho$) [pts/m$^3$]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'Density_vs_Time.png'},
         
        {'y': 'Snap_Count', 'x': 'Time_s', 
         'title': r'{test name} Amount of Snapshots ($n$) vs Total Latency Time ($t$)', 
         'y_lbl': r'Amount of Snapshots ($n$)', 'x_lbl': r'Total Latency Time ($t$) [s]', 'fn': 'SnapCount_vs_Time.png'},
         
        {'y': 'Points_Per_Snap', 'x': 'Snap_Count', 
         'title': r'{test name} Rate of Point Capture per Snapshot ($\Delta P$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Point Capture Rate ($\Delta P$) [pts/n]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'PointsPerSnap_vs_SnapCount.png'},
         
        {'y': 'Points_Per_Time', 'x': 'Time_s', 
         'title': r'{test name} Rate of Point Capture per Second ($\dot{P}$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Point Capture Rate ($\dot{P}$) [pts/s]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'PointsPerTime_vs_Time.png'},
         
        {'y': 'Density_Per_Snap', 'x': 'Snap_Count', 
         'title': r'{test name} Rate of Density Increase per Snapshot ($\Delta \rho$) vs Amount of Snapshots ($n$)', 
         'y_lbl': r'Density Increase Rate ($\Delta \rho$) [pts/m$^3$/n]', 'x_lbl': r'Amount of Snapshots ($n$)', 'fn': 'DensityPerSnap_vs_SnapCount.png'},
         
        {'y': 'Density_Per_Time', 'x': 'Time_s', 
         'title': r'{test name} Rate of Density Increase per Second ($\dot{\rho}$) vs Map Latency Time ($t$)', 
         'y_lbl': r'Density Increase Rate ($\dot{\rho}$) [pts/m$^3$/s]', 'x_lbl': r'Map Latency Time ($t$) [s]', 'fn': 'DensityPerTime_vs_Time.png'}
    ]

    unified_out_dir = file_path.parent / f"Unified_Analysis_{file_path.stem}"
    isolated_out_dir = file_path.parent / f"Isolated_Global_Analysis_{file_path.stem}"
    unified_out_dir.mkdir(exist_ok=True)
    isolated_out_dir.mkdir(exist_ok=True)

    active_objects = sorted(combined_objects_df['Object_Name'].unique())
    plt.style.use('seaborn-v0_8-whitegrid')
    
    # --- GRAYSCALE IEEE COMPLIANCE ALLOCATION ---
    markers = ['o', 's', '^', 'v', 'D', 'X', 'P']
    linestyles = ['--', '-.', ':', '--', '-.', ':']
    colors = ['#111111', '#333333', '#555555', '#777777', '#222222', '#444444'] 
    
    style_map = {}
    for i, obj in enumerate(active_objects):
        style_map[obj] = {
            'color': colors[i % len(colors)],
            'marker': markers[i % len(markers)],
            'linestyle': linestyles[i % len(linestyles)]
        }

    for idx, plot_cfg in enumerate(plots_to_generate, 1):
        y_col = plot_cfg['y']
        x_col = plot_cfg['x']
        
        if x_col not in combined_objects_df.columns or y_col not in combined_objects_df.columns: 
            print(f"  [SKIP] Missing data vectors for pairing: {y_col} vs {x_col}")
            continue

        is_bivariate = (x_col == 'Point_Count' and y_col == 'Density_pts_m3')

        if is_bivariate:
            state_agg = combined_objects_df.groupby('Snap_Count').agg({
                x_col: ['mean', 'std'],
                y_col: ['mean', 'std']
            }).reset_index()
            
            agg_df = pandas.DataFrame({
                'x_mean': state_agg[x_col]['mean'],
                'x_std': state_agg[x_col]['std'].fillna(0),
                'y_mean': state_agg[y_col]['mean'],
                'y_std': state_agg[y_col]['std'].fillna(0)
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

        try:
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
        except Exception as e:
            y_smooth = None
            r2 = 0
            name = "None"

        # Dynamically compile the official title and axis labels
        final_title = plot_cfg['title'].replace('{test name}', test_name)
        y_axis_label = plot_cfg['y_lbl']
        x_axis_label = plot_cfg['x_lbl']

        # --- 1. UNIFIED PLOT ---
        fig, ax = plt.subplots(figsize=(10, 6.5))
        
        for obj, obj_df in combined_objects_df.groupby('Object_Name'):
            sorted_df = obj_df.sort_values(x_col)
            label_str = obj
            
            # TRIGGER JITTER CALCULATION IF PLOTTING DENSITY
            if 'Density' in final_title or 'rho' in final_title:
                j_rms = calculate_temporal_jitter(sorted_df[y_col])
                label_str = f"{obj} [RMS Jitter: {j_rms:.2f}]"
            
            style = style_map[obj]
            ax.plot(
                sorted_df[x_col], sorted_df[y_col],
                color=style['color'],
                marker=style['marker'],
                linestyle=style['linestyle'],
                linewidth=1.8,
                markersize=5,
                alpha=0.6, 
                label=label_str
            )

        # Plot Global Mean Data
        global_label = 'Global Mean ± 1 STD'
        if 'Density' in final_title or 'rho' in final_title:
            g_j_rms = calculate_temporal_jitter(agg_df['y_mean'])
            global_label = f"Global Mean ± 1 STD [RMS Jitter: {g_j_rms:.2f}]"

        ax.errorbar(x_plot, y_plot, yerr=y_err, xerr=x_err, fmt='o', color='black', 
                    ecolor='#555555', elinewidth=2, capsize=4, markersize=8, 
                    linestyle='-', linewidth=3.0, zorder=5, label=global_label)
        
        if y_smooth is not None and r2 > 0:
            ax.plot(x_smooth_raw, y_smooth, linestyle='-', color='#d62728', 
                    linewidth=2.5, label=f"Trendline: {name} ($R^2 = {r2:.3f}$)", zorder=6)

        ax.set_title(final_title, fontsize=11, weight='bold', pad=12)
        ax.set_xlabel(x_axis_label, weight='bold')
        ax.set_ylabel(y_axis_label, weight='bold')
        
        ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), frameon=True, edgecolor='black', fontsize=9)
        plt.tight_layout()
        
        plt.savefig(unified_out_dir / f"Unified_{idx:02d}_{plot_cfg['fn']}", dpi=300, bbox_inches='tight')
        plt.close()

        # --- 2. ISOLATED PLOT ---
        fig, ax = plt.subplots(figsize=(10, 6.5))
        ax.errorbar(x_plot, y_plot, yerr=y_err, xerr=x_err, fmt='o', color='black', 
                    ecolor='#555555', elinewidth=2, capsize=4, markersize=8, 
                    linestyle='-', linewidth=3.0, zorder=5, label=global_label)
        
        if y_smooth is not None and r2 > 0:
            ax.plot(x_smooth_raw, y_smooth, linestyle='--', color='#d62728', 
                     linewidth=2.5, label=f"Best Fit: {name} ($R^2 = {r2:.3f}$)", zorder=6)
                     
        ax.set_title(f"[Isolated Profile] {final_title}", fontsize=11, weight='bold', pad=12)
        ax.set_xlabel(x_axis_label, weight='bold')
        ax.set_ylabel(y_axis_label, weight='bold')
        ax.legend(loc='best', frameon=True, edgecolor='black')
        plt.tight_layout()
        plt.savefig(isolated_out_dir / f"Isolated_{idx:02d}_{plot_cfg['fn']}", dpi=300)
        plt.close()

    print(f"\n[SUCCESS] Execution completed. Config matrix adapted successfully.")

if __name__ == "__main__":
    main()

    """
    metric_pairs = [
        ('Volume_m3', 'Snap_Count', '{test name} Average Primitive Calculated Volume (V)(m³) vs Amount of Snapshots (n)'),
        ('Percent_Error', 'Snap_Count', '{test name} Primitive/Global Average Volume Percent Error E (%) vs Amount of Snapshots (n)'),
        ('Point_Count', 'Snap_Count', '{test name} Primitive/Global Average Total Point Count (P) vs Amount of Snapshots (n)'),
        ('Density_pts_m3', 'Snap_Count', '{test name} Primitive/Global Average Point Density (ρ) vs Amount of Snapshots (n)'),
        ('Point_Count', 'Time_s', '{test name} Primitive/Global Average Total Point Count (P) vs Map Latency Time (s)'),
        ('Density_pts_m3', 'Time_s', '{test name} Primitive/Global Average Point Density (ρ) vs Map Latency Time (s)'),
        ('Snap_Count', 'Time_s', '{test name} Amount of Snapshots (n) vs Total Latency Time (s)'),
        ('Density_pts_m3', 'Point_Count', "{test name} Primitive/Global Average Point Density (r'\rho') vs Primitive/Global Average Total Point Count (P)"),
        ('Points_Per_Snap', 'Snap_Count', "{test name} Primitive/Global Average Rate of Point Capture per Snapshot ax.set_ylabel(r'($\Delta \rho$)P') (P/n) vs Amount of Snapshots (n)"),
        ('Points_Per_Time', 'Time_s', '{test name} Primitive/Global Average Rate of Point Capture per second (Ṗ) (P/s) vs Map Latency Time (s)'),
        #('Points_Per_Time', 'Time_s', '{test name} Primitive/Global Average Rate of Point Capture per second (ax.set_ylabel(r'Rate of Point Capture ($\dot{P}$)')) (P/s) vs Map Latency Time (s)'),
        ('Density_Per_Snap', 'Snap_Count', "{test name} Primitive/Global Average Rate of Point Density Increase per Snapshot ax.set_ylabel(r'Density change per snapshot ($\Delta \rho$)') (ρ/n) vs Amount of Snapshots (n)"),
        ('Density_Per_Time', 'Time_s', "{test name} Primitive/Global Average Rate of Point Density Increase per second (ax.set_ylabel(r'Rate of Density Increase ($\dot{\rho}$)'))(ρ/s) vs Map Latency Time (s)")
    ]"""