import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import warnings
from scipy.optimize import curve_fit
from matplotlib.lines import Line2D

# Suppress warnings
warnings.filterwarnings('ignore')

# Fallback style guide mapping to match your plotted figures
style_guide = {
    'CY2': {
        'standing_map_0deg': {'color': '#d62728', 'linestyle': ':', 'marker': 'o', 'label': 'CY2 (Fixed Height Baseline)'},
        'varying_height_0deg': {'color': '#2ca02c', 'linestyle': ':', 'marker': 's', 'label': 'CY2 (Varying Height Profile)'}
    },
    'Global': {
        'standing_map_0deg': {'color': '#1f77b4', 'linestyle': ':', 'marker': 'o', 'label': 'Global Avg (Fixed Height Baseline)'},
        'varying_height_0deg': {'color': '#2ca02c', 'linestyle': ':', 'marker': 's', 'label': 'Global Avg (Varying Height Profile)'}
    }
}

def standardize_columns(df):
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
        elif ('global_mean_percent_error' in c_lower or 'error' in c_lower) and 'Percent_Error' not in renamed_targets:
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
        elif ('density: pts per m3' in c_lower or 'average density' in c_lower or 'density' in c_lower) and 'time' not in c_lower and 'snap' not in c_lower and 'Concentration_Value' not in renamed_targets:
            new_names[col] = 'Concentration_Value'
            renamed_targets.add('Concentration_Value')
    df.rename(columns=new_names, inplace=True)
    return df.loc[:, ~df.columns.duplicated()]

def offset_power_law(x, a, b, c):
    return a * (x ** b) + c

def compute_regression_stats(x_data, y_mean, is_rate_curve=False):
    # 1. Create a strict mask to filter out NaN or Inf values from both arrays
    valid_mask = np.isfinite(x_data) & np.isfinite(y_mean)
    fit_x = x_data[valid_mask]
    fit_y = y_mean[valid_mask]
    
    # 2. Defensive fallback if the dataset is entirely corrupted/empty
    if len(fit_x) < 2:
        return np.zeros_like(x_data), "Std: 0.0 | Insufficient Data"

    std_val = np.std(fit_y)
    
    if is_rate_curve:
        try:
            # Fit using the clean data
            popt, _ = curve_fit(offset_power_law, fit_x, fit_y, p0=[500.0, -0.8, 150.0], bounds=([0.1, -3.0, 0.0], [5000.0, -0.01, 500.0]), maxfev=10000)
            
            # Project the trendline back onto the original x-axis length for plotting
            y_pred = offset_power_law(x_data, *popt) 
            
            # Calculate R^2 strictly on the valid data points
            y_pred_fit = offset_power_law(fit_x, *popt)
            ss_res = np.sum((fit_y - y_pred_fit)**2)
            ss_tot = np.sum((fit_y - np.mean(fit_y))**2)
            r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
            eq_str = f"Pow Law ($R^2={r2_val:.2f}$)"
        except:
            slope, intercept = np.polyfit(fit_x, fit_y, 1)
            y_pred = slope * x_data + intercept 
            y_pred_fit = slope * fit_x + intercept
            ss_res, ss_tot = np.sum((fit_y - y_pred_fit)**2), np.sum((fit_y - np.mean(fit_y))**2)
            r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
            eq_str = f"y={slope:.1f}x+{intercept:.1f} ($R^2={r2_val:.2f}$)"
    else:
        # Linear regression using the clean data
        slope, intercept = np.polyfit(fit_x, fit_y, 1)
        y_pred = slope * x_data + intercept 
        
        y_pred_fit = slope * fit_x + intercept
        ss_res, ss_tot = np.sum((fit_y - y_pred_fit)**2), np.sum((fit_y - np.mean(fit_y))**2)
        r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
        eq_str = f"y={slope:.2f}x+{intercept:.1f} ($R^2={r2_val:.2f}$)"
        
    metrics_str = f"Std: {std_val:.1f} | {eq_str}"
    return y_pred, metrics_str

def build_horizontal_panel(df_map, style_guide, panel_cfgs, save_path, global_title):
    # Enforce a 1x2 panel layout
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.suptitle(global_title, fontsize=15, weight='bold', y=0.98)
    
    legend_handles = []
    legend_labels = []
    
    for idx, cfg in enumerate(panel_cfgs):
        ax = axes[idx]
        y_col, x_col = cfg['y'], cfg['x']
        is_rate = (idx == 1) 
        
        for data_key, sub_df in df_map.items():
            if sub_df.empty: continue
                
            for env, group_df in sub_df.groupby('Test_Environment'):
                sorted_df = group_df.sort_values(by=x_col)
                x_data = sorted_df[x_col].to_numpy()
                y_mean = sorted_df[y_col].to_numpy()
                
                y_series = pd.Series(y_mean)
                y_std = y_series.rolling(window=3, min_periods=1).std().fillna(0).to_numpy()
                
                meta = style_guide[data_key][env]
                y_pred, metrics_str = compute_regression_stats(x_data, y_mean, is_rate)
                
                # Dynamically tag labels by metric type to provide clarity in a single legend
                metric_tag = "Volume Error" if idx == 0 else "Accumulation Rate"
                full_label = f"{meta['label']} ({metric_tag})\n[{metrics_str}]"
                
                clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_mean)) * 0.75)
                
                # Render to subplot
                ax.fill_between(x_data, y_mean - clamped_std, y_mean + clamped_std, color=meta['color'], alpha=0.10)
                ax.plot(x_data, y_mean, color=meta['color'], linestyle=meta['linestyle'], marker=meta['marker'], markersize=4, alpha=0.4)
                ax.plot(x_data, y_pred, color=meta['color'], linestyle=':', linewidth=2.0)
                
                # FIXED: Continuously capture custom legend handles for BOTH idx == 0 and idx == 1
                custom_handle = Line2D(
                    [0], [0], 
                    color=meta['color'], 
                    linestyle=':', 
                    linewidth=2.5, 
                    marker=meta['marker'], 
                    markersize=10,
                    markeredgecolor=meta['color']
                )
                legend_handles.append(custom_handle)
                legend_labels.append(full_label)
                
        ax.set_title(cfg['title'], weight='bold', fontsize=11, pad=10)
        ax.set_xlabel(cfg['x_lbl'])
        ax.set_ylabel(cfg['y_lbl'])

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    # --- MATRIX SORTED 2-ROW MASTER LEGEND ---
    # Top Left: Varying Error      | Top Right: Varying Accumulation
    # Bottom Left: Baseline Error  | Bottom Right: Baseline Accumulation
    legend_fig, legend_ax = plt.subplots(figsize=(13, 2.2)) 
    legend_ax.axis('off')
    
    # Map raw loop indices to your precise 2x2 grid layout requirement
    # legend_handles structure from loop: 
    # [0]=Baseline Error, [1]=Varying Error, [2]=Baseline Accumulation, [3]=Varying Accumulation
    matrix_handles = [
        legend_handles[0],  # Top Left: Varying Error
        legend_handles[1],  # Top Right: Varying Accumulation
        legend_handles[2],  # Bottom Left: Baseline Error
        legend_handles[3]   # Bottom Right: Baseline Accumulation
    ]
    
    matrix_labels = [
        legend_labels[0],
        legend_labels[1],
        legend_labels[2],
        legend_labels[3]
    ]
    
    legend_ax.legend(
        handles=matrix_handles, 
        labels=matrix_labels,
        loc='center', 
        ncol=2,             # Locks the layout to exactly 2 columns side-by-side
        fontsize=11,      
        frameon=True,     
        shadow=True,
        edgecolor='black',
        facecolor='#ffffff',
        framealpha=1.0,
        handlelength=4.0,
        columnspacing=3.0   # Ensures ample separation between the left and right columns
    )
    
    legend_path = save_path.parent / f"{save_path.stem}_Matrix_Sorted_Legend.png"
    legend_fig.savefig(legend_path, bbox_inches='tight', dpi=300, transparent=True)
    plt.close(legend_fig)
    print(f"\n[SUCCESS] Matrix-sorted 2x2 legend generated -> {legend_path.name}")

def main():
    parser = argparse.ArgumentParser(description="Compile Test 3 Visual Multi-Panel Suites.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    target_subfolders = ['standing_map_0deg', 'varying_height_0deg']
    
    box_data = []
    for env_name in target_subfolders:
        env_path = base_dir / env_name
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: continue
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            if sheet_lower in ['front_lf_box', 'front_rt_box', 'back_rt_box', 'back_lf_box', 'cy2_box', 'roomba_box', 'cy2', 'roomba']:
                df = pd.read_excel(xls, sheet_name=sheet)
                df = standardize_columns(df)
                df['Test_Environment'] = env_name
                df['Is_CY2'] = True if ('cy2' in sheet_lower or 'roomba' in sheet_lower) else False
                box_data.append(df)

    if not box_data:
        print("[CRITICAL] Ingestion failure. No matching sheets found.")
        sys.exit(1)

    combined_df = pd.concat(box_data, ignore_index=True)
    cy2_isolated = combined_df[combined_df['Is_CY2'] == True].copy()
    global_averages = combined_df[combined_df['Is_CY2'] == False].copy()

    output_dir = base_dir / "Vertical_Scaling_Analysis"
    output_dir.mkdir(exist_ok=True)

    # Simplified to two explicit targets
    panel_cfgs = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 'y_lbl': r'Average Volume Percent Error ($\bar{E}$) [%]', 'x_lbl': r'Amount of Snapshots ($n$)', 'title': '(a)  CY2 Average Volume Percent Error vs Amount of Snapshots'},
        {'y': 'Concentration_Rate', 'x': 'Time_s', 'y_lbl': r'Point Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Time ($t$) [s]', 'title': '(b) CY2 Point Concentration Accumulation Rate vs Time'}
    ]

    g_title = "Vertical Kinematic Scaling Analysis"

    build_horizontal_panel({'CY2': cy2_isolated}, style_guide, panel_cfgs, output_dir / "01_CY2_Isolated_Panel.png", g_title)

    print(f"\n[SUCCESS] Pipeline execution complete. PNG arrays and separated legends exported to /{output_dir.name}/")

if __name__ == "__main__":
    main()