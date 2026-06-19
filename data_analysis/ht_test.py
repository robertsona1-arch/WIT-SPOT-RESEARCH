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
            eq_str = f"Pow Law ($R^2={r2_val:.3f}$)"
        except:
            slope, intercept = np.polyfit(x_data, y_mean, 1)
            y_pred = slope * x_data + intercept
            ss_res, ss_tot = np.sum((y_mean - y_pred)**2), np.sum((y_mean - np.mean(y_mean))**2)
            r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
            eq_str = f"y = {slope:.1f}x + {intercept:.1f} ($R^2={r2_val:.3f}$)"
    else:
        slope, intercept = np.polyfit(x_data, y_mean, 1)
        y_pred = slope * x_data + intercept
        ss_res, ss_tot = np.sum((y_mean - y_pred)**2), np.sum((y_mean - np.mean(y_mean))**2)
        r2_val = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0
        eq_str = f"y = {slope:.3f}x + {intercept:.1f} ($R^2={r2_val:.3f}$)"
        
    metrics_str = f"Var: {var_val:.2f} | Rpl: {j_rms:.2f}\nFit: {eq_str}"
    return y_pred, metrics_str

def main():
    parser = argparse.ArgumentParser(description="Isolate CY2 low-profile target graphs into individual PNGs with comprehensive statistical legends.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    parser.add_argument('--name', default="Test 3 Varying Height", help="Official Test Name to replace in titles")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = args.name

    target_subfolders = ['standing_map_0deg', 'varying_height_0deg']
    valid_paths = {f: base_dir / f for f in target_subfolders if (base_dir / f).exists()}

    if len(valid_paths) < 2:
        print("[CRITICAL] Varying height analysis requires BOTH standing_map_0deg and varying_height folders.")
        sys.exit(1)

    cy2_data_compiled = []
    print("Extracting datasets and isolating Low-Profile Target CY2...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: continue
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            # Explicitly isolate the Roomba/CY2 box
            if 'cy2' in sheet_lower or 'roomba' in sheet_lower:
                df = pd.read_excel(xls, sheet_name=sheet)
                df = standardize_columns(df)
                df['Test_Environment'] = env_name
                df['Quadrant_Class'] = r"Low-Profile Target $CY_2$"
                cy2_data_compiled.append(df)

    if not cy2_data_compiled:
        print("[CRITICAL] No CY2/Roomba tabs found in the target files.")
        sys.exit(1)

    combined_cy2_df = pd.concat(cy2_data_compiled, ignore_index=True)
    
    output_dir = base_dir / "CY2_Isolated_Analysis"
    output_dir.mkdir(exist_ok=True)

    plots_to_generate = [
        {'y': 'Percent_Error', 'x': 'Snap_Count', 'y_lbl': r'Volume Error ($E$) [%]', 'x_lbl': r'Snapshots ($n$)', 'title': f'{test_name} Volume Error (CY2)', 'fn': '01_CY2_PercentError.png', 'legend_loc': 'upper right'},
        {'y': 'Concentration_Value', 'x': 'Time_s', 'y_lbl': r'Concentration ($C$) [pts/m$^3$]', 'x_lbl': r'Time ($t$) [s]', 'title': f'{test_name} Point Concentration (CY2)', 'fn': '02_CY2_Concentration.png', 'legend_loc': 'lower right'},
        {'y': 'Concentration_Rate', 'x': 'Time_s', 'y_lbl': r'Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Time ($t$) [s]', 'title': f'{test_name} Accumulation Rate (CY2)', 'fn': '03_CY2_AccumulationRate.png', 'legend_loc': 'upper right'}
    ]

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({'font.size': 11, 'axes.labelsize': 12, 'axes.labelweight': 'bold', 'axes.titlesize': 13, 'axes.titleweight': 'bold'})

    # Update this dictionary block at the top of main()
    env_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': 'Fixed Height Baseline', 'color': '#1f77b4'},
        'varying_height_0deg': {'linestyle': '--', 'marker': 's', 'label': 'Varying Height Profile', 'color': '#d62728'}
    }

    for idx, plot_cfg in enumerate(plots_to_generate):
        y_col, x_col = plot_cfg['y'], plot_cfg['x']
        if x_col not in combined_cy2_df.columns or y_col not in combined_cy2_df.columns:
            continue

        fig, ax = plt.subplots(figsize=(10, 7.5))
        is_rate = (idx == 2)

        for env, group_df in combined_cy2_df.groupby('Test_Environment'):
            agg_df = group_df.groupby(x_col)[y_col].agg(['mean', 'std']).reset_index()
            x_data = agg_df[x_col].to_numpy()
            y_mean = agg_df['mean'].to_numpy()
            y_std = agg_df['std'].fillna(0).to_numpy()
            
            # FIXED: Directly pull the flat style dictionary using the folder string key
            style = env_map.get(env)
            if style is None:
                continue # Failsafe to bypass unexpected folders
                
            y_pred, metrics_str = compute_regression_stats(x_data, y_mean, is_rate)
            
            # Formats the label cleanly using the 'label' element inside the style dictionary
            full_label = f"{style['label']}\n{metrics_str}"
            
            # Update your ax.plot line to use the specific color tracked in the map
            clamped_std = np.clip(y_std, 0, np.nanmax(np.abs(y_mean)) * 0.75)
            ax.fill_between(x_data, y_mean - clamped_std, y_mean + clamped_std, color=style['color'], alpha=0.15)
            ax.plot(x_data, y_mean, color=style['color'], linestyle=style['linestyle'], marker=style['marker'], linewidth=2.5, label=full_label)
            ax.plot(x_data, y_pred, color=style['color'], linestyle=':', linewidth=2.0)

        ax.set_title(f"[CY2 Target] {plot_cfg['title']}", pad=15)
        ax.set_xlabel(plot_cfg['x_lbl'])
        ax.set_ylabel(plot_cfg['y_lbl'])
        
        # Legend formatting specifically designed to handle the multi-line stat blocks
        ax.legend(loc=plot_cfg['legend_loc'], framealpha=1.0, edgecolor='black', fontsize=10, handlelength=3.0, borderpad=1.0, labelspacing=1.2)

        plt.tight_layout()
        save_path = output_dir / plot_cfg['fn']
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  [SAVED] -> {plot_cfg['fn']}")

    print("\n[SUCCESS] Individual CY2 figures generated with embedded regression legends.")

if __name__ == "__main__":
    main()