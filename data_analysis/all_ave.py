import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import sys

def main():
    parser = argparse.ArgumentParser(description="Compare isolated global run averages across all 6 environments.")
    parser.add_argument('--folder', required=True, help="Root directory containing the 6 test condition subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Core experimental folder names
    target_subfolders = [
        'closer',
        'rotating_map',
        'standing_map_180deg',
        'standing_map_0deg',
        'varying_height_0deg',
        'varying_height_180deg'
    ]

    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if not valid_paths:
        print("[CRITICAL] None of the specified 6 subfolders were located.")
        sys.exit(1)

    # Column name alignment map for dashboard metrics
    rename_map = {
        'Average Point Count': 'Point_Count',
        'Average Density': 'Density: Pts Per m3',
        'Average Points Per Snapshot': 'Points Per Snapshot',
        'Average Points Per Time': 'Points Per Time',
        'Average Density Per Time': 'Density Per Time',
        'Average Density Per Snapshot': 'Density Per Snapshot'
    }

    dashboard_master_data = []

    print("Extracting global dashboard metrics across conditions...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            
            if 'Averages_Dashboard' in xls.sheet_names:
                df_dash = pd.read_excel(xls, sheet_name='Averages_Dashboard')
                df_dash = df_dash.rename(columns=rename_map)
                df_dash['Test_Environment'] = env_name
                dashboard_master_data.append(df_dash)
            else:
                print(f"  [WARNING] 'Averages_Dashboard' sheet missing in {file_to_open.name}")
                
        except Exception as e:
            print(f"  [ERROR] Failed to parse {file_to_open.name}: {e}")

    if not dashboard_master_data:
        print("[CRITICAL] No 'Averages_Dashboard' data could be compiled.")
        sys.exit(1)

    combined_dash_df = pd.concat(dashboard_master_data, ignore_index=True)

    metric_pairs = [
        ('Snap_Count', 'Point_Count', 'Snap Count vs Point Count'),
        ('Snap_Count', 'Density: Pts Per m3', 'Snap Count vs Density'),
        ('Time_s', 'Point_Count', 'Time vs Point Count'),
        ('Time_s', 'Density: Pts Per m3', 'Time vs Density'),
        ('Time_s', 'Snap_Count', 'Time vs Snap Count'),
        ('Point_Count', 'Density: Pts Per m3', 'Point Count vs Density'),
        ('Snap_Count', 'Points Per Snapshot', 'Snap Count vs Points per Snapshot'),
        ('Time_s', 'Points Per Time', 'Time vs Average Points per Time'),
        ('Time_s', 'Density Per Time', 'Time vs Average Density per Time'),
        ('Snap_Count', 'Density Per Snapshot', 'Snap Count vs Density per Snapshot')
    ]

    output_dir = base_dir / "Cross_Test_Global_Averages"
    output_dir.mkdir(exist_ok=True)
    print(f"\nGenerating isolated global graphs into: {output_dir.name}/")

    # High-contrast color mapping for the 6 environments
    env_colors = {
        'closer': '#1f77b4',
        'rotating_map': '#ff7f0e',
        'standing_map_0deg': '#2ca02c',
        'standing_map_180deg': '#d62728',
        'varying_height_0deg': '#9467bd',
        'varying_height_180deg': '#8c564b'
    }

    sns.set_theme(context="paper", style="whitegrid", font_scale=1.2)

    for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
        if x_col not in combined_dash_df.columns or y_col not in combined_dash_df.columns:
            continue

        plt.figure(figsize=(10, 6.5))

        if x_col == 'Point_Count':
            # Bivariate mapping: sort by X to keep lines from zig-zagging
            for env, env_df in combined_dash_df.groupby('Test_Environment'):
                sorted_df = env_df.sort_values(by='Point_Count')
                plt.plot(
                    sorted_df[x_col], sorted_df[y_col], 
                    color=env_colors.get(env, '#7f7f7f'),
                    linewidth=3, marker='s', markersize=6, label=env
                )
        else:
            # Standard sequential timeline plotting
            sns.lineplot(
                data=combined_dash_df, x=x_col, y=y_col, 
                hue='Test_Environment', palette=env_colors,
                linewidth=3, marker='s', markersize=6
            )

        plt.title(f"Global Run Comparison: {title}", fontsize=13, weight='bold', pad=12)
        plt.xlabel(x_col.replace('_', ' '))
        plt.ylabel(y_col.replace('_', ' '))
        plt.legend(loc='best', frameon=True, shadow=True)
        plt.tight_layout()

        safe_title = title.replace(' ', '_').replace(':', '')
        plt.savefig(output_dir / f"Global_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

    print(f"\n[SUCCESS] 10 standalone comparison figures exported to: {output_dir.absolute()}")

if __name__ == "__main__":
    main()