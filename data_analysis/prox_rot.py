import argparse
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from pathlib import Path
from scipy.optimize import curve_fit

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
    valid_mask = np.isfinite(x_data) & np.isfinite(y_mean)
    fit_x = x_data[valid_mask]
    fit_y = y_mean[valid_mask]
    
    if len(fit_x) < 2:
        return np.zeros_like(x_data), "Std: 0.0 | Insufficient Data"

    std_val = np.std(fit_y)
    
    if is_rate_curve:
        try:
            popt, _ = curve_fit(offset_power_law, fit_x, fit_y, p0=[500.0, -0.8, 150.0], bounds=([0.1, -3.0, 0.0], [5000.0, -0.01, 500.0]), maxfev=10000)
            y_pred = offset_power_law(x_data, *popt) 
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
        slope, intercept = np.polyfit(fit_x, fit_y, 1)
        y_pred = slope * x_data + intercept 
        y_pred_fit = slope * fit_x + intercept
        ss_res, ss_tot = np.sum((fit_y - y_pred_fit)**2), np.sum((fit_y - np.mean(fit_y))**2)
        r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
        eq_str = f"y={slope:.2f}x+{intercept:.1f} ($R^2={r2_val:.2f}$)"
        
    metrics_str = f"Std: {std_val:.1f} | {eq_str}"
    return y_pred, metrics_str

def main():
    parser = argparse.ArgumentParser(description="Compile 3-Way Composite Dashboard Analysis.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    
    # IMPORTANT: Verify 'proximity_map' matches your exact 1.5m test folder name
    target_subfolders = ['standing_map_0deg', 'rotating_map', 'closer'] 
    
    dashboard_rows = []
    raw_quadrant_rows = []
    
    print("Ingesting dashboard baselines and computing quadrant variance structures...")
    for env_name in target_subfolders:
        env_path = base_dir / env_name
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: 
            print(f"[WARNING] Missing directory or target file: {env_name}")
            continue
        
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            
            if sheet_lower == 'averages_dashboard':
                df = pd.read_excel(xls, sheet_name=sheet)
                df.columns = df.columns.astype(str).str.strip()
                # Dynamically map the columns based on your requested inputs
                rename_map = {}
                for col in df.columns:
                    c_low = col.lower()
                    if 'average density per time' in c_low or 'density per time' in c_low:
                        rename_map[col] = 'Concentration_Rate'
                    elif 'global_mean_percent_error' in c_low or 'global mean percent error' in c_low:
                        rename_map[col] = 'Percent_Error'
                df.rename(columns=rename_map, inplace=True)
                df['Test_Environment'] = env_name
                dashboard_rows.append(df)
            
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
    
    output_dir = base_dir / "Composite_System_Analysis"
    output_dir.mkdir(exist_ok=True)

    # Aligned configuration matrix containing explicit raw mapping definitions for both panels
    panel_cfgs = [
        {
            'y': 'Percent_Error', 
            'x': 'Snap_Count', 
            'raw_y': 'Percent_Error', 
            'raw_x': 'Snap_Count', 
            'y_lbl': r'GlobalAverage Volume Percent Error ($\bar{E}$) [%]', 
            'x_lbl': r'Number of Snapshots ($n$)', 
            'title': '(a)  Global Average Volume Percent Error vs Amount of Snapshots'
        },
        {
            'y': 'Concentration_Rate', 
            'x': 'Time_s', 
            'raw_y': 'Concentration_Rate', 
            'raw_x': 'Time_s', 
            'y_lbl': r'Global Point Concentration Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 
            'x_lbl': r'Time ($t$) [s]', 
            'title': '(b) Global Point Concentration Accumulation Rate vs Time'
        }
    ]

    # Integrated precise shapes and hex colors for distinct visual contrast
    env_style_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': 'Static Baseline (3.5m)', 'color': '#ae2012'},
        'rotating_map': {'linestyle': '--', 'marker': '^', 'label': 'Rotational Drive', 'color': '#005f73'},
        'closer': {'linestyle': '-.', 'marker': '*', 'label': 'Proximity Range (1.5m)', 'color': '#ca6702'}
    }

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(1, 2, figsize=(15, 8.5)) 
    fig.suptitle("Composite Operational Limits Analysis", fontsize=22, weight='bold', y=0.98)
    
    legend_handles_err = []
    legend_labels_err = []
    legend_handles_rate = []
    legend_labels_rate = []

    for idx, cfg in enumerate(panel_cfgs):
        ax = axes[idx]
        is_rate = (idx == 1)
        
        for env, group_dash in dash_df.groupby('Test_Environment'):
            if env not in env_style_map: continue
            style = env_style_map[env]
            
            sorted_dash = group_dash.sort_values(by=cfg['x'])
            x_dash = sorted_dash[cfg['x']].to_numpy()
            y_dash = sorted_dash[cfg['y']].to_numpy()
            
            env_quads = quad_df[quad_df['Test_Environment'] == env]
            agg_quads = env_quads.groupby(cfg['raw_x'])[cfg['raw_y']].agg(['std']).reset_index()
            
            merged_stats = pd.merge(sorted_dash, agg_quads, left_on=cfg['x'], right_on=cfg['raw_x'], how='left')
            y_std = merged_stats['std'].fillna(0).to_numpy()
            
            y_pred, metrics_str = compute_regression_stats(x_dash, y_dash, is_rate)
            
            metric_tag = "Volume Error" if idx == 0 else "Accumulation Rate"
            full_label = f"{style['label']} ({metric_tag})\n[{metrics_str}]"
            
            clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_dash)) * 0.75)
            ax.fill_between(x_dash, y_dash - clamped_std, y_dash + clamped_std, color=style['color'], alpha=0.12)
            
            ax.plot(x_dash, y_dash, color=style['color'], linestyle=style['linestyle'], marker=style['marker'], markersize=8, linewidth=3.5)
            ax.plot(x_dash, y_pred, color=style['color'], linestyle=':', linewidth=2.5)

            # Generate massive handles for the 2x3 independent legend export
            custom_handle = Line2D([0], [0], color=style['color'], linestyle=':', linewidth=3.0, marker=style['marker'], markersize=14, markeredgecolor=style['color'])
            
            if idx == 0:
                legend_handles_err.append(custom_handle)
                legend_labels_err.append(full_label)
            else:
                legend_handles_rate.append(custom_handle)
                legend_labels_rate.append(full_label)

        ax.set_title(cfg['title'], weight='bold', fontsize=18, pad=12)
        ax.set_xlabel(cfg['x_lbl'], fontsize=16, weight='bold')
        ax.set_ylabel(cfg['y_lbl'], fontsize=16, weight='bold')
        ax.tick_params(axis='both', which='major', labelsize=14)

    plt.tight_layout()
    main_save_path = output_dir / "01_Composite_Dashboard.png"
    plt.savefig(main_save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    # --- FIXED 3x2 VERTICAL MATRIX LEGEND EXPORT ---
    # Column 1 (Left): Volume Error      | Column 2 (Right): Accumulation Rate
    # Row 1 (Top):    Rotational Drive  | Row 1 (Top):     Rotational Drive
    # Row 2 (Middle): Proximity Range   | Row 2 (Middle):  Proximity Range
    # Row 3 (Bottom): Static Baseline   | Row 3 (Bottom):  Static Baseline
    # Defensive check to catch directory mismatch bugs before they trigger an IndexError
    if len(legend_handles_err) < 3 or len(legend_handles_rate) < 3:
        print(f"\n[CRITICAL ERROR] Legend matrix construction failed.")
        print(f"Expected 3 target profiles but only ingested {len(legend_handles_err)}.")
        print(f"Check your file directory. Is your proximity folder named exactly as specified in the script?")
        sys.exit(1)

    # --- FIXED 3x2 VERTICAL MATRIX LEGEND EXPORT ---
    legend_fig, legend_ax = plt.subplots(figsize=(14, 4.0)) 
    legend_ax.axis('off')
    
    # Map loop indexes to fit row-by-row population across exactly 2 columns
    # legend_handles_err:  [0]=Baseline, [1]=Rotating, [2]=Proximity
    # legend_handles_rate: [0]=Baseline, [1]=Rotating, [2]=Proximity

    """
    matrix_3x2_handles = [
        legend_handles_err[1], legend_handles_rate[1],  # Row 1: Rotating
        legend_handles_err[0], legend_handles_rate[0],  # Row 2: Proximity
        legend_handles_err[2], legend_handles_rate[2]   # Row 3: Baseline
    ]
    
    matrix_3x2_labels = [
        legend_labels_err[1], legend_labels_rate[1],
        legend_labels_err[0], legend_labels_rate[0],
        legend_labels_err[2], legend_labels_rate[2]
    ]
    """
    matrix_3x2_handles = [
        legend_handles_err[0], legend_handles_err[1],  # Row 1: Rotating
        legend_handles_err[2], legend_handles_rate[0],  # Row 2: Proximity
        legend_handles_rate[1], legend_handles_rate[2]   # Row 3: Baseline
    ]
    
    matrix_3x2_labels = [
        legend_labels_err[0], legend_labels_err[1],
        legend_labels_err[2], legend_labels_rate[0],
        legend_labels_rate[1], legend_labels_rate[2]
    ]
    
    legend_ax.legend(
        handles=matrix_3x2_handles, 
        labels=matrix_3x2_labels,
        loc='center', 
        ncol=2,             # Constrains the canvas to exactly 2 columns wide
        fontsize=13,      
        frameon=True,     
        shadow=True,
        edgecolor='black',
        facecolor='#ffffff',
        framealpha=1.0,
        handlelength=4.0,
        columnspacing=3.0,
        labelspacing=1.2    # Adds explicit vertical breathing room between your stacked rows
    )
    
    legend_save_path = output_dir / "01_Composite_Dashboard_3x2_Legend.png"
    legend_fig.savefig(legend_save_path, bbox_inches='tight', dpi=300, transparent=True)
    plt.close(legend_fig)
    print(f"\n[SUCCESS] Custom 3x2 Matrix Legend compiled -> {legend_save_path.name}")

if __name__ == "__main__":
    main()