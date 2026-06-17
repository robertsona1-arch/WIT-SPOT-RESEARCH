import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import sys

def main():
    parser = argparse.ArgumentParser(description="Plot standalone cross-test percent errors for all 6 individual primitives.")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Isolated target environments (excluding varying_height_180deg)
    target_environments = [
        'closer',
        'rotating_map',
        'standing_map_0deg',
        'standing_map_180deg',
        'varying_height_0deg'
    ]

    object_data_compiled = []

    print("Extracting updated error profiles across the 5 test environments...")
    for env_name in target_environments:
        env_path = base_dir / env_name
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            sheets_to_ignore = ['master_data', 'averages_dashboard', 'regression_metrics']
            
            for sheet in xls.sheet_names:
                sheet_lower = sheet.lower()
                if sheet_lower in sheets_to_ignore or 'mid' in sheet_lower or 'center' in sheet_lower:
                    continue
                
                df = pd.read_excel(xls, sheet_name=sheet)
                if 'Percent_Error' not in df.columns:
                    continue
                    
                df['Test_Environment'] = env_name
                df['Box_Identifier'] = sheet  
                
                # Consistent classification across the entire testing suite
                if 'front' in sheet_lower and ('lf' in sheet_lower or 'left' in sheet_lower):
                    obj_class = 'Front Left Box'
                elif 'front' in sheet_lower and ('rt' in sheet_lower or 'right' in sheet_lower):
                    obj_class = 'Front Right Box'
                elif 'back' in sheet_lower and ('lf' in sheet_lower or 'left' in sheet_lower):
                    obj_class = 'Back Left Box'
                elif 'back' in sheet_lower and ('rt' in sheet_lower or 'right' in sheet_lower):
                    obj_class = 'Back Right Box'
                elif 'roomba' in sheet_lower:
                    obj_class = 'Roomba'
                elif 'chair' in sheet_lower or 'circ' in sheet_lower:
                    obj_class = 'Circ_chair'
                else:
                    obj_class = sheet
                    
                object_data_compiled.append(df)
                
        except Exception as e:
            print(f"[ERROR] Failed to parse data from folder {env_name}: {e}")

    if not object_data_compiled:
        print("[CRITICAL] No matching primitive tracking data compiled.")
        sys.exit(1)

    combined_df = pd.concat(object_data_compiled, ignore_index=True)

    output_dir = base_dir / "Individual_Primitive_Errors"
    output_dir.mkdir(exist_ok=True)

    # Strict High-Contrast IEEE Standards
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 13,
        'axes.labelweight': 'bold',
        'axes.titlesize': 14,
        'axes.titleweight': 'bold',
        'legend.frameon': True,
        'legend.edgecolor': '#000000',
        'legend.facecolor': '#ffffff'
    })

    env_colors = {
        'standing_map_0deg': '#000000',    # Black Baseline
        'closer': '#2ca02c',               # Green
        'rotating_map': '#ff7f0e',         # Orange
        'standing_map_180deg': '#1f77b4',  # Blue
        'varying_height_0deg': '#d62728'   # Red
    }

    env_markers = {
        'standing_map_0deg': 'o', 'closer': 's', 'rotating_map': '^',
        'standing_map_180deg': 'D', 'varying_height_0deg': 'v'
    }

    unique_objects = combined_df['Quadrant_Class' if 'Quadrant_Class' in combined_df.columns else 'Box_Identifier'].unique()
    # Fallback assignment alignment layer
    if 'Quadrant_Class' not in combined_df.columns:
        combined_df['Quadrant_Class'] = combined_df['Box_Identifier'].apply(
            lambda x: 'Front Left Box' if 'front_lf' in x.lower() else
                      ('Front Right Box' if 'front_rt' in x.lower() else
                       ('Back Left Box' if 'back_lf' in x.lower() else
                        ('Back Right Box' if 'back_rt' in x.lower() else
                         ('Roomba' if 'roomba' in x.lower() else 'Circ_chair'))))
        )
    
    unique_objects = combined_df['Quadrant_Class'].unique()

    print(f"\nGenerating 6 separate verification plots inside: {output_dir.name}/")

    for obj in unique_objects:
        obj_df = combined_df[combined_df['Quadrant_Class'] == obj]
        obj_df = obj_df.sort_values(by=['Test_Environment', 'Snap_Count'])
        
        fig, ax = plt.subplots(figsize=(10, 6.5))
        
        # Plot each test environment line individually to ensure accurate legend rendering
        for env, group_df in obj_df.groupby('Test_Environment'):
            env_label = env.replace('_map', '').replace('_deg', '°')
            ax.plot(
                group_df['Snap_Count'], group_df['Percent_Error'],
                color=env_colors.get(env, '#7f7f7f'),
                marker=env_markers.get(env, 'x'),
                linestyle='-' if env == 'standing_map_0deg' else '--',
                linewidth=2.5, markersize=7,
                label=env_label
            )
            
        ax.set_title(f"Volumetric Percent Error Profile: {obj}", pad=12)
        ax.set_xlabel("Snap Count (Snapshot Index)")
        ax.set_ylabel("Volume Percent Error (%)")
        ax.legend(loc='best', framealpha=1.0, edgecolor='black')
        plt.tight_layout()
        
        safe_name = obj.replace(' ', '_').replace('(', '').replace(')', '')
        plt.savefig(output_dir / f"Error_Trend_{safe_name}.png", dpi=300)
        plt.close()

    print(f"\n[SUCCESS] Completed tracking passes. All 6 standalone figures exported successfully.")

if __name__ == "__main__":
    main()