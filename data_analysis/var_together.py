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

def calculate_temporal_jitter(y_array):
    """Calculates high-frequency signal ripple using array-safe NumPy tracking."""
    y_data = np.asarray(y_array)
    y_clean = y_data[~np.isnan(y_data)]
    if len(y_clean) < 2: return 0.0
    return np.sqrt(np.mean(np.diff(y_clean)**2))

def offset_power_law(x, a, b, c):
    """Mathematical constraint model for steady-state accumulation bounds."""
    return a * (x ** b) + c

def compute_regression_stats(x_data, y_mean, is_rate_curve=False):
    """Computes regression arrays and formatted legend strings safely."""
    var_val = np.var(y_mean)
    j_rms = calculate_temporal_jitter(y_mean)
    
    if is_rate_curve:
        try:
            popt, _ = curve_fit(offset_power_law, x_data, y_mean, p0=[500.0, -0.8, 150.0], bounds=([0.1, -3.0, 0.0], [5000.0, -0.01, 500.0]), maxfev=10000)
            y_pred = offset_power_law(x_data, *popt)
            ss_res = np.sum((y_mean - y_pred)**2)
            ss_tot = np.sum((y_mean - np.mean(y_mean))**2)
            r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
            eq_str = f"Pow Law ($R^2={r2_val:.2f}$)"
        except:
            slope, intercept = np.polyfit(x_data, y_mean, 1)
            y_pred = slope * x_data + intercept
            ss_res, ss_tot = np.sum((y_mean - y_pred)**2), np.sum((y_mean - np.mean(y_mean))**2)
            r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
            eq_str = f"y={slope:.1f}x+{intercept:.1f} ($R^2={r2_val:.2f}$)"
    else:
        slope, intercept = np.polyfit(x_data, y_mean, 1)
        y_pred = slope * x_data + intercept
        ss_res, ss_tot = np.sum((y_mean - y_pred)**2), np.sum((y_mean - np.mean(y_mean))**2)
        r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
        eq_str = f"y={slope:.2f}x+{intercept:.1f} ($R^2={r2_val:.2f}$)"
        
    metrics_str = f"Var: {var_val:.1f} | Rpl: {j_rms:.1f} | {eq_str}"
    return y_pred, metrics_str

def build_horizontal_panel(df_map, style_guide, panel_cfgs, save_path, global_title):
    """Generates a structured 1x3 panel row with all legend arrays mounted underneath."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 6.5))
    fig.suptitle(global_title, fontsize=15, weight='bold', y=0.98)
    
    for idx, cfg in enumerate(panel_cfgs):
        ax = axes[idx]
        y_col, x_col = cfg['y'], cfg['x']
        is_rate = (idx == 2)
        
        for data_key, sub_df in df_map.items():
            if sub_df.empty:
                continue
                
            for env, group_df in sub_df.groupby('Test_Environment'):
                agg_df = group_df.groupby(x_col)[y_col].agg(['mean', 'std']).reset_index()
                x_data = agg_df[x_col].to_numpy()
                y_mean = agg_df['mean'].to_numpy()
                y_std = agg_df['std'].fillna(0).to_numpy()
                
                # Verified link to style guide dictionary
                meta = style_guide[data_key][env]
                y_pred, metrics_str = compute_regression_stats(x_data, y_mean, is_rate)
                
                full_label = f"{meta['label']}\n[{metrics_str}]"
                clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_mean)) * 0.75)
                
                ax.fill_between(x_data, y_mean - clamped_std, y_mean + clamped_std, color=meta['color'], alpha=0.10)
                ax.plot(x_data, y_mean, color=meta['color'], linestyle=meta['linestyle'], marker=meta['marker'], markersize=4, alpha=0.4)
                ax.plot(x_data, y_pred, color=meta['color'], linestyle=':', linewidth=2.0, label=full_label)
                
        ax.set_title(cfg['title'], weight='bold', fontsize=11, pad=10)
        ax.set_xlabel(cfg['x_lbl'])
        ax.set_ylabel(cfg['y_lbl'])
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=1, fontsize=8, frameon=True, edgecolor='black', handlelength=3.0)

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Compile Test 3 Visual Multi-Panel Suites.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    target_subfolders = ['standing_map_0deg', 'varying_height_0deg']
    
    box_data = []
    print("Ingesting data structures for vertical kinematic tracking profiles...")
    for env_name in target_subfolders:
        env_path = base_dir / env_name
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: continue
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            # FIXED: Explicitly capture CY2 and Roomba named sheets cleanly
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

    print(f"  [DATA LOG] Ingested rows -> CY2: {len(cy2_isolated)}, Global: {len(global_averages)}")

    output_dir = base_dir / "Vertical_Scaling_Analysis"
    output_dir.mkdir(exist_ok=True)

    panel_cfgs = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 'y_lbl': r'Volume Error ($E$) [%]', 'x_lbl': r'Snapshots ($n$)', 'title': '(a) Volume Percent Error'},
        {'y': 'Concentration_Value', 'x': 'Time_s', 'y_lbl': r'Concentration ($C$) [pts/m$^3$]', 'x_lbl': r'Time ($t$) [s]', 'title': '(b) Point Concentration'},
        {'y': 'Concentration_Rate', 'x': 'Time_s', 'y_lbl': r'Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Time ($t$) [s]', 'title': '(c) Accumulation Rate'}
    ]

    style_guide = {
        'CY2': {
            'standing_map_0deg': {'color': '#ff7f0e', 'linestyle': '-', 'marker': 'o', 'label': 'CY2 (Fixed Height Baseline)'},
            'varying_height_0deg': {'color': '#d62728', 'linestyle': '--', 'marker': 's', 'label': 'CY2 (Varying Height Profile)'}
        },
        'Global': {
            'standing_map_0deg': {'color': '#1f77b4', 'linestyle': '-', 'marker': 'o', 'label': 'Global (Fixed Height Baseline)'},
            'varying_height_0deg': {'color': '#2ca02c', 'linestyle': '--', 'marker': 's', 'label': 'Global (Varying Height Profile)'}
        }
    }

    g_title = "Vertical Kinematic Scaling Analysis"

    # PNG 1: Just CY2 (The Low-Profile Roomba Target)
    build_horizontal_panel({'CY2': cy2_isolated}, style_guide, panel_cfgs, output_dir / "01_CY2_Isolated_Panel.png", g_title)
    print("  [PANEL EXPORT] Saved 01_CY2_Isolated_Panel.png")

    # PNG 2: Just the Globals
    build_horizontal_panel({'Global': global_averages}, style_guide, panel_cfgs, output_dir / "02_Global_Average_Panel.png", g_title)
    print("  [PANEL EXPORT] Saved 02_Global_Average_Panel.png")

    # PNG 3: The Full 6-Plot Composite Overlay
    build_horizontal_panel({'Global': global_averages, 'CY2': cy2_isolated}, style_guide, panel_cfgs, output_dir / "03_Combined_6Plot_Overlay_Panel.png", g_title)
    print("  [PANEL EXPORT] Saved 03_Combined_6Plot_Overlay_Panel.png")

    print(f"\n[SUCCESS] Pipeline execution complete. Figures exported to /{output_dir.name}/")

if __name__ == "__main__":
    main()