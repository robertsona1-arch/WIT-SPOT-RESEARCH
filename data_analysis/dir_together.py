import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import warnings

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
        elif ('global_mean_percent_error' in c_lower or 'global mean percent error' in c_lower or 'error' in c_lower) and 'Percent_Error' not in renamed_targets:
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
    if len(y_clean) < 2: 
        return 0.0
    return np.sqrt(np.mean(np.diff(y_clean)**2))

def main():
    parser = argparse.ArgumentParser(description="Compile Test 2 Directional Bias 1x3 panel row for column-spanning layouts.")
    parser.add_argument('--folder', required=True, help="Root directory containing test subfolders")
    parser.add_argument('--name', default="Test 2 Directional Bias", help="Official Test Name to replace in titles")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    test_name = args.name

    target_subfolders = ['standing_map_0deg', 'standing_map_180deg']
    valid_paths = {f: base_dir / f for f in target_subfolders if (base_dir / f).exists()}

    if len(valid_paths) < 2:
        print("[CRITICAL] Directional bias analysis requires BOTH standing_map_0deg and standing_map_180deg folders.")
        sys.exit(1)

    box_data_compiled = []
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files: continue
        xls = pd.ExcelFile(excel_files[0])
        for sheet in xls.sheet_names:
            sheet_lower = sheet.lower().strip()
            if sheet_lower in ['front_lf_box', 'front_rt_box', 'back_rt_box', 'back_lf_box']:
                df = pd.read_excel(xls, sheet_name=sheet)
                df = standardize_columns(df)
                df['Test_Environment'] = env_name
                
                if 'front_lf' in sheet_lower: df['Quadrant_Class'] = r"$RP_1$"
                elif 'front_rt' in sheet_lower: df['Quadrant_Class'] = r"$RP_2$"
                elif 'back_rt' in sheet_lower: df['Quadrant_Class'] = r"$RP_3$"
                elif 'back_lf' in sheet_lower: df['Quadrant_Class'] = r"$RP_4$"
                
                box_data_compiled.append(df)

    combined_box_df = pd.concat(box_data_compiled, ignore_index=True)
    output_dir = base_dir / "Directional_Bias_Analysis"
    output_dir.mkdir(exist_ok=True)

    # Instantiate the 1x3 canvas matrix
    fig, axes = plt.subplots(1, 3, figsize=(15, 6.0))
    
    # ADD THIS LINE: Sets the global title and offsets it to prevent subheader collision
    fig.suptitle(f"{test_name}", fontsize=16, weight='bold', y=1.05)
    
    panel_cfgs = [
        {'ax': axes[0], 'y': 'Percent_Error', 'x': 'Snap_Count', 'y_lbl': r'Volume Error ($E$) [%]', 'x_lbl': r'Snapshots ($n$)', 'title': '(a) Volume Percent Error', 'legend_loc': 'bottom'},
        {'ax': axes[1], 'y': 'Concentration_Value', 'x': 'Time_s', 'y_lbl': r'Concentration ($C$) [pts/m$^3$]', 'x_lbl': r'Time ($t$) [s]', 'title': '(b) Point Concentration', 'legend_loc': 'bottom'},
        {'ax': axes[2], 'y': 'Concentration_Rate', 'x': 'Time_s', 'y_lbl': r'Accumulation Rate ($\dot{C}$) [pts/m$^3$/s]', 'x_lbl': r'Time ($t$) [s]', 'title': '(c) Concentration Accumulation Rate', 'legend_loc': 'bottom'}
    ]

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({'font.size': 9, 'axes.labelsize': 11, 'axes.labelweight': 'bold'})

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] 
    quadrants = [r"$RP_1$", r"$RP_2$", r"$RP_3$", r"$RP_4$"]
    color_map = {q: colors[i] for i, q in enumerate(quadrants)}
    
    env_map = {
        'standing_map_0deg': {'linestyle': '-', 'marker': 'o', 'label': '0°'},
        'standing_map_180deg': {'linestyle': '--', 'marker': 's', 'label': '180°'}
    }

    
    
    for p_idx, cfg in enumerate(panel_cfgs):
        ax = cfg['ax']
        y_col, x_col = cfg['y'], cfg['x']
        
        
        for (quadrant, env), group_df in combined_box_df.groupby(['Quadrant_Class', 'Test_Environment']):
            sorted_df = group_df.sort_values(by=x_col)
            x_data = sorted_df[x_col].to_numpy()
            y_data = sorted_df[y_col].to_numpy()
            
            c_color = color_map.get(quadrant)
            style = env_map.get(env)
            j_ripple = calculate_temporal_jitter(y_data)
            
            label_str = f"{quadrant} ({style['label']}) [Rpl: {j_ripple:.1f}]"
            
            ax.plot(x_data, y_data, color=c_color, linestyle=style['linestyle'], 
                    marker=style['marker'], linewidth=1.8, markersize=4, alpha=0.85, label=label_str)

        ax.set_title(cfg['title'], weight='bold', fontsize=11, pad=10)
        ax.set_xlabel(cfg['x_lbl'])
        ax.set_ylabel(cfg['y_lbl'])
        
        if cfg['legend_loc'] == 'bottom':
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=2, fontsize=8, frameon=True, edgecolor='black')
        else:
            ax.legend(loc='upper right', fontsize=8, frameon=True, edgecolor='black')

    plt.tight_layout()
    panel_path = output_dir / "Test2_Directional_Bias_Combined_Panel.png"
    plt.savefig(panel_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\n[SUCCESS] Column-spanning directional bias row panel generated: {panel_path.name}")

if __name__ == "__main__":
    main()