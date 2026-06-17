import os
import glob
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import argparse
import sys
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="Generate 13 individual publication plots for Test 4 matching flipped Y vs X guidelines.")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
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

    print("Executing flexible string-match parsing pass for front primitives...")
    for label, path in target_conditions.items():
        excel_files = list(path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            for sheet_name in xls.sheet_names:
                sheet_lower = sheet_name.lower().strip()
                
                if 'front' in sheet_lower and ('mid' not in sheet_lower and 'center' not in sheet_lower):
                    if any(l in sheet_lower for l in ['lf', 'left']):
                        box_class = 'Front Left Box'
                    elif any(r in sheet_lower for r in ['rt', 'right']):
                        box_class = 'Front Right Box'
                    else:
                        continue
                        
                    df = pd.read_excel(xls, sheet_name=sheet_name)
                    
                    if 'Density: Pts Per m3' not in df.columns and 'Density' in df.columns:
                        df['Density: Pts Per m3'] = df['Density']
                    if 'Time' not in df.columns and 'Time_sec' in df.columns:
                        df['Time'] = df['Time_sec']
                    elif 'Time' not in df.columns:
                        df['Time'] = df['Snap_Count'] * 1.5 
                        
                    # Calculate Derived Time/Snap Ratio Metrics Requested
                    df['Points_Per_Snap'] = df['Point_Count'] / df['Snap_Count'].replace(0, 1)
                    df['Avg_Points_Per_Time'] = df['Point_Count'] / df['Time'].replace(0, 1)
                    df['Density_Per_Time'] = df['Density: Pts Per m3'] / df['Time'].replace(0, 1)
                    df['Density_Per_Snap'] = df['Density: Pts Per m3'] / df['Snap_Count'].replace(0, 1)
                    
                    df['Operational_Condition'] = label
                    df['Primitive_Target'] = box_class
                    compiled_runs.append(df)
                    
            print(f"  [SUCCESS] Data ingestion mapping finalized for: {path.name}")
        except Exception as e:
            print(f"  [ERROR] Processing crash on workbook {file_to_open.name}: {e}")

    if not compiled_runs:
        print("[CRITICAL] Dataframe compilation vector is completely empty. Verify tab spellings.")
        return

    master_df = pd.concat(compiled_runs, ignore_index=True)
    output_dir = base_dir / "Test4_Comprehensive_Plots"
    output_dir.mkdir(exist_ok=True)

    # Apply Strict IEEE Publication Aesthetics
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

    # Updated Matrix Map: Core trackers + 10 explicitly FLIPPED axis pairings
    plots_to_generate = [
        # Standard Baseline Trackers (Locked in Original Positions)
        {'y': 'Volume_m3', 'x': 'Time', 'y_lbl': 'Volume (m³)', 'x_lbl': 'Time (s)', 'fn': 'Volume_vs_Time.png'},
        {'y': 'Percent_Error', 'x': 'Time', 'y_lbl': 'Volumetric Percent Error (%)', 'x_lbl': 'Time (s)', 'fn': 'PercentError_vs_Time.png'},
        
        # 10 Explicitly Inverted Axis Pairs (Y and X flipped)
        {'y': 'Point_Count', 'x': 'Snap_Count', 'y_lbl': 'Point Count (pts)', 'x_lbl': 'Snapshot Count', 'fn': 'PointCount_vs_SnapCount.png'},
        {'y': 'Density: Pts Per m3', 'x': 'Snap_Count', 'y_lbl': 'Spatial Density (pts/m³)', 'x_lbl': 'Snapshot Count', 'fn': 'Density_vs_SnapCount.png'},
        {'y': 'Point_Count', 'x': 'Time', 'y_lbl': 'Point Count (pts)', 'x_lbl': 'Time (s)', 'fn': 'PointCount_vs_Time.png'},
        {'y': 'Density: Pts Per m3', 'x': 'Time', 'y_lbl': 'Spatial Density (pts/m³)', 'x_lbl': 'Time (s)', 'fn': 'Density_vs_Time.png'},
        {'y': 'Snap_Count', 'x': 'Time', 'y_lbl': 'Snapshot Count', 'x_lbl': 'Time (s)', 'fn': 'SnapCount_vs_Time.png'},
        {'y': 'Density: Pts Per m3', 'x': 'Point_Count', 'y_lbl': 'Spatial Density (pts/m³)', 'x_lbl': 'Point Count (pts)', 'fn': 'Density_vs_PointCount.png'},
        {'y': 'Points_Per_Snap', 'x': 'Snap_Count', 'y_lbl': 'Points per Snapshot (pts/snap)', 'x_lbl': 'Snapshot Count', 'fn': 'PointsPerSnap_vs_SnapCount.png'},
        {'y': 'Avg_Points_Per_Time', 'x': 'Time', 'y_lbl': 'Average Points per Time (pts/s)', 'x_lbl': 'Time (s)', 'fn': 'AvgPointsPerTime_vs_Time.png'},
        {'y': 'Density_Per_Time', 'x': 'Time', 'y_lbl': 'Density Accumulation Rate (pts/m³/s)', 'x_lbl': 'Time (s)', 'fn': 'DensityPerTime_vs_Time.png'},
        {'y': 'Density_Per_Snap', 'x': 'Snap_Count', 'y_lbl': 'Density per Snapshot (pts/m³/snap)', 'x_lbl': 'Snapshot Count', 'fn': 'DensityPerSnap_vs_SnapCount.png'}
    ]

    style_hues = {
        ('Static Baseline (0° Heading)', 'Front Left Box'): ('#000000', 'o', '-'),
        ('Static Baseline (0° Heading)', 'Front Right Box'): ('#555555', 's', '-'),
        ('Dynamic Yaw Rotation Sweep', 'Front Left Box'): ('#d62728', 'v', '--'),
        ('Dynamic Yaw Rotation Sweep', 'Front Right Box'): ('#ff7f0e', '^', '--')
    }

    print(f"\nProcessing inverted image matrix loop inside: {output_dir.name}/")

    for plot_cfg in plots_to_generate:
        y_attr = plot_cfg['y']
        x_attr = plot_cfg['x']
        
        if y_attr not in master_df.columns or x_attr not in master_df.columns:
            print(f"  [SKIP] Column mapping pairing [{y_attr} vs {x_attr}] is missing data vectors.")
            continue
            
        fig, ax = plt.subplots(figsize=(10, 6.2))
        
        for (cond, prim), group_data in master_df.groupby(['Operational_Condition', 'Primitive_Target']):
            sorted_data = group_data.sort_values(by=x_attr)
            color, marker, lstyle = style_hues.get((cond, prim), ('#7f7f7f', 'x', ':'))
            
            ax.plot(
                sorted_data[x_attr], sorted_data[y_attr],
                label=f"{prim} ({cond.split(' (')[0]})",
                color=color,
                marker=marker,
                linestyle=lstyle,
                linewidth=2.2,
                markersize=6,
                alpha=0.9
            )
            
        ax.set_title(f"{plot_cfg['y_lbl'].split(' (')[0]} vs. {plot_cfg['x_lbl'].split(' (')[0]} Tracking Spectrum", pad=12)
        ax.set_ylabel(plot_cfg['y_lbl'])
        ax.set_xlabel(plot_cfg['x_lbl'])
        ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), framealpha=1.0, edgecolor='black')
        
        plt.tight_layout()
        save_path = output_dir / plot_cfg['fn']
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  [SAVED INVERTED PNG] -> {plot_cfg['fn']}")

    print("\nGlobal data plotting sequence finalized. Axis inversions are complete.")

if __name__ == "__main__":
    main()