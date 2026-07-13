import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import warnings
from scipy.optimize import curve_fit

print("\n" + "="*50)
print("CRITICAL LOG: IF YOU SEE THIS, YOU ARE EDITING THE RIGHT FILE!")
print("="*50 + "\n")


# Suppress warnings
warnings.filterwarnings('ignore')

# Custom library imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *

# Rigidly locked color mapping array to maintain graph symmetry
quad_colors = {r"$RP_1$":'#1f77b4', r"$RP_2$": '#ff7f0e', r"$RP_3$": '#2ca02c', r"$RP_4$": '#d62728'}

def build_1x2_dashboard_panel(df, panel_cfgs, save_path, global_title, env_style_map):
    """FORCED OVERRIDE: Overwrites build_horizontal_panel locally with giant fonts."""
    import matplotlib.pyplot as plt
    
    # Force style sheet load first
    plt.style.use('seaborn-v0_8-whitegrid')
    
    fig, axes = plt.subplots(1, 2, figsize=(15, 8.5)) 
    fig.suptitle(global_title, fontsize=22, weight='bold', y=0.98)
    
    for idx, cfg in enumerate(panel_cfgs):
        ax = axes[idx]
        is_rate = (idx == 1)
        
        for env, group_dash in df.groupby('Test_Environment'):
            style = env_style_map[env]
            
            sorted_dash = group_dash.sort_values(by=cfg['x'])
            x_dash = sorted_dash[cfg['x']].to_numpy()
            y_dash = sorted_dash[cfg['y']].to_numpy()
            
            y_pred, metrics_str = compute_regression_stats(x_dash, y_dash, is_rate)
            full_label = f"Global Avg ({style['label']})\n[{metrics_str}]"
            
            ax.plot(x_dash, y_dash, color=style['color'], linestyle=style['linestyle'], 
                    marker=style['marker'], markersize=9, linewidth=3.5, label=full_label)
            ax.plot(x_dash, y_pred, color=style['color'], linestyle=':', linewidth=2.5)

        # Scale up axes structures
        ax.set_title(cfg['title'], weight='bold', fontsize=18, pad=12)
        ax.set_xlabel(cfg['x_lbl'], fontsize=16, weight='bold')
        ax.set_ylabel(cfg['y_lbl'], fontsize=16, weight='bold')
        ax.tick_params(axis='both', which='major', labelsize=14)
        
        # Mount the legend container
        leg = ax.legend(
            loc='upper center', 
            bbox_to_anchor=(0.5, -0.25), # Pushed down to clear the larger axis labels
            ncol=1, 
            frameon=True, 
            edgecolor='black', 
            facecolor='white',
            framealpha=1.0,
            handlelength=5.0
        )
        
        # Hard override every text asset manually inside the active container
        if leg:
            for text_obj in leg.get_texts():
                text_obj.set_fontsize(16)
                text_obj.set_weight('bold')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\n[SUCCESS] Local intercept successful. Figures saved with massive fonts: {save_path.name}")

def main():
    parser = argparse.ArgumentParser(description="Compile 1x2 Dashboard Rotational Impact Analysis with Spatial Variance.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    target_subfolders = ['standing_map_0deg', 'rotating_map']
    
    dashboard_rows = []
    raw_quadrant_rows = []
    
    print("Ingesting dashboard baselines and computing quadrant variance structures...")
    for env_name in target_subfolders:
        env_path = base_dir / env_name
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: continue
        
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            
            # Extract clean dashboard lines
            if sheet_lower == 'averages_dashboard':
                df = pd.read_excel(xls, sheet_name=sheet)
                df.columns = df.columns.str.strip()
                df.rename(columns={'Average Density Per Time': 'Concentration_Rate', 'Global_Mean_Percent_Error': 'Percent_Error'}, inplace=True)
                df['Test_Environment'] = env_name
                dashboard_rows.append(df)
            
            # Extract underlying primitives to map spatial variance fields
            elif sheet_lower in ['front_lf_box', 'front_rt_box', 'back_rt_box', 'back_lf_box']:
                df = pd.read_excel(xls, sheet_name=sheet)
                df = standardize_columns(df)
                df['Test_Environment'] = env_name
                raw_quadrant_rows.append(df)

    if not dashboard_rows or not raw_quadrant_rows:
        print("[CRITICAL] Ingestion failure. Verify sheets exist.")
        sys.exit(1)

    dash_df = pd.concat(dashboard_rows, ignore_index=True)
    quad_df = pd.concat(raw_quadrant_rows, ignore_index=True)
    
    output_dir = base_dir / "Rotational_Impact_Analysis"
    output_dir.mkdir(exist_ok=True)

    build_1x2_dashboard_panel = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 'raw_y': 'Percent_Error', 'raw_x': 'Snap_Count', 'y_lbl': r'Volume Error ($E$) [%]', 'x_lbl': r'Number of Snapshots ($n$)', 'title': '(a) Volume Percent Error vs Number of Snapshots'},
        {'y': 'Concentration_Rate', 'x': 'Time_s', 'raw_y': 'Concentration_Rate', 'raw_x': 'Time_s', 'y_lbl': r'Point Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Time ($t$) [s]', 'title': '(b) Point Concentration Accumulation Rate vs Time'}
    ]

    env_style_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': 'Static Baseline 0°', 'color': '#005f73'},
        'rotating_map': {'linestyle': '--', 'marker': 's', 'label': 'Rotational Drive', 'color': '#ae2012'}
    }

    fig, axes = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.suptitle("Rotational Sensor Impact Analysis", fontsize=15, weight='bold', y=0.98)
    plt.style.use('seaborn-v0_8-whitegrid')

    for idx, cfg in enumerate(build_1x2_dashboard_panel):
        ax = axes[idx]
        is_rate = (idx == 1)
        
        for env, group_dash in dash_df.groupby('Test_Environment'):
            style = env_style_map[env]
            
            # 1. Pull Mean Line directly from Dashboard DataFrame
            sorted_dash = group_dash.sort_values(by=cfg['x'])
            x_dash = sorted_dash[cfg['x']].to_numpy()
            y_dash = sorted_dash[cfg['y']].to_numpy()
            
            # 2. Extract matching Standard Deviation from Raw Quadrants DataFrame
            env_quads = quad_df[quad_df['Test_Environment'] == env]
            agg_quads = env_quads.groupby(cfg['raw_x'])[cfg['raw_y']].agg(['std']).reset_index()
            
            # Align lengths cleanly to ensure no plotting mismatches
            merged_stats = pd.merge(sorted_dash, agg_quads, left_on=cfg['x'], right_on=cfg['raw_x'], how='left')
            y_std = merged_stats['std'].fillna(0).to_numpy()
            
            y_pred, metrics_str = compute_regression_stats(x_dash, y_dash, is_rate)
            full_label = f"Global Avg ({style['label']})\n[{metrics_str}]"
            
            # Plot standard deviation envelope using the raw cross-sheet math
            clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_dash)) * 0.75)
            ax.fill_between(x_dash, y_dash - clamped_std, y_dash + clamped_std, color=style['color'], alpha=0.12)
            
            # Plot the accurate Dashboard Line
            ax.plot(x_dash, y_dash, color=style['color'], linestyle=style['linestyle'], marker=style['marker'], markersize=5, linewidth=2.5, label=full_label)
            ax.plot(x_dash, y_pred, color=style['color'], linestyle=':', linewidth=2.0)

        ax.set_title(cfg['title'], weight='bold', fontsize=11, pad=10)
        ax.set_xlabel(cfg['x_lbl'])
        ax.set_ylabel(cfg['y_lbl'])
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=1, fontsize=8, frameon=True, edgecolor='black', handlelength=3.0)

    plt.tight_layout()
    save_path = output_dir / "01_Rotational_Dashboard_With_Variance.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"\n[SUCCESS] Figures rendered with accurate dashboard means and spatial halos: {save_path.name}")

if __name__ == "__main__":
    main()