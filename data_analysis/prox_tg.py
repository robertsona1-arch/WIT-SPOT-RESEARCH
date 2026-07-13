import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import warnings
from scipy.optimize import curve_fit

# Suppress warnings
warnings.filterwarnings('ignore')

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from leo_funcs import *

# Shared target color matrix for multi-panel row symmetry
quad_colors = {r"$RP_1$": '#1f77b4', r"$RP_2$": '#ff7f0e', r"$RP_3$": '#2ca02c', r"$RP_4$": '#d62728'}
    

def main():
    parser = argparse.ArgumentParser(description="Compile 1x2 Dashboard Proximity Analysis with Variance Bounds.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    target_subfolders = ['standing_map_0deg', 'closer']
    
    dashboard_rows = []
    raw_quadrant_rows = []
    
    print("Ingesting proximity dashboard baselines and computing spatial halos...")
    for env_name in target_subfolders:
        env_path = base_dir / env_name
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: continue
        
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            
            if sheet_lower == 'averages_dashboard':
                df = pd.read_excel(xls, sheet_name=sheet)
                df.columns = df.columns.str.strip()
                df.rename(columns={'Average Density Per Time': 'Concentration_Rate', 'Global_Mean_Percent_Error': 'Percent_Error'}, inplace=True)
                df['Test_Environment'] = env_name
                dashboard_rows.append(df)
            
            elif sheet_lower in ['front_lf_box', 'front_rt_box', 'back_rt_box', 'back_lf_box']:
                df = pd.read_excel(xls, sheet_name=sheet)
                df = standardize_columns(df)
                df['Test_Environment'] = env_name
                raw_quadrant_rows.append(df)

    if not dashboard_rows or not raw_quadrant_rows:
        print("[CRITICAL] Ingestion failure. Verify 'Averages_Dashboard' and box sheets exist.")
        sys.exit(1)

    dash_df = pd.concat(dashboard_rows, ignore_index=True)
    quad_df = pd.concat(raw_quadrant_rows, ignore_index=True)
    
    output_dir = base_dir / "Proximity_Impact_Analysis"
    output_dir.mkdir(exist_ok=True)

    # 1x2 panel configurations: Point Concentration has been stripped out
    build_1x2_dashboard_panel = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 'raw_y': 'Percent_Error', 'raw_x': 'Snap_Count', 'y_lbl': r'Volume Error ($E$) [%]', 'x_lbl': r'Snapshots ($n$)', 'title': '(a) Volume Percent Error'},
        {'y': 'Concentration_Rate', 'x': 'Time_s', 'raw_y': 'Concentration_Rate', 'raw_x': 'Time_s', 'y_lbl': r'Point Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Time ($t$) [s]', 'title': '(b) Point Concentration Accumulation Rate'}
    ]

    env_style_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': 'Standard Standoff (3.5m)', 'color': '#1d3557'},
        'closer': {'linestyle': '--', 'marker': 's', 'label': 'Proximity Range (1.5m)', 'color': '#e63946'}
    }

    fig, axes = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.suptitle("Sensor Proximity Impact Analysis", fontsize=15, weight='bold', y=0.98)
    plt.style.use('seaborn-v0_8-whitegrid')

    for idx, cfg in enumerate(build_1x2_dashboard_panel):
        ax = axes[idx]
        is_rate = (idx == 1)
        
        for env, group_dash in dash_df.groupby('Test_Environment'):
            style = env_style_map[env]
            
            # 1. Pull exact mean line trajectory from Dashboard DataFrame
            sorted_dash = group_dash.sort_values(by=cfg['x'])
            x_dash = sorted_dash[cfg['x']].to_numpy()
            y_dash = sorted_dash[cfg['y']].to_numpy()
            
            # 2. Extract matching standard deviation constraints from Raw Quadrant DataFrames
            env_quads = quad_df[quad_df['Test_Environment'] == env]
            agg_quads = env_quads.groupby(cfg['raw_x'])[cfg['raw_y']].agg(['std']).reset_index()
            
            merged_stats = pd.merge(sorted_dash, agg_quads, left_on=cfg['x'], right_on=cfg['raw_x'], how='left')
            y_std = merged_stats['std'].fillna(0).to_numpy()
            
            y_pred, metrics_str = compute_regression_stats(x_dash, y_dash, is_rate)
            full_label = f"Global Avg ({style['label']})\n[{metrics_str}]"
            
            # Render standard deviation boundary envelope
            clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_dash)) * 0.75)
            ax.fill_between(x_dash, y_dash - clamped_std, y_dash + clamped_std, color=style['color'], alpha=0.12)
            
            # Render clean dashboard plot line
            ax.plot(x_dash, y_dash, color=style['color'], linestyle=style['linestyle'], marker=style['marker'], markersize=5, linewidth=2.5, label=full_label)
            ax.plot(x_dash, y_pred, color=style['color'], linestyle=':', linewidth=2.0)

        ax.set_title(cfg['title'], weight='bold', fontsize=11, pad=10)
        ax.set_xlabel(cfg['x_lbl'])
        ax.set_ylabel(cfg['y_lbl'])
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=1, fontsize=8, frameon=True, edgecolor='black', handlelength=3.0)

    plt.tight_layout()
    save_path = output_dir / "01_Proximity_Dashboard_Averages.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"\n[SUCCESS] Proximity panels compiled cleanly from dashboard targets: {save_path.name}")

if __name__ == "__main__":
    main()