import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import sys

def main():
    parser = argparse.ArgumentParser(description="Analyze directional sensor bias using unique line colors for specific quadrants.")
    parser.add_argument('--folder', required=True, help="Root directory containing the test subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Root directory '{base_dir}' does not exist.")
        sys.exit(1)

    target_subfolders = ['standing_map_0deg', 'standing_map_180deg']
    
    valid_paths = {}
    for folder_name in target_subfolders:
        folder_path = base_dir / folder_name
        if folder_path.exists():
            valid_paths[folder_name] = folder_path
        else:
            print(f"[WARNING] Expected subfolder missing: {folder_name}")

    if len(valid_paths) < 2:
        print("[CRITICAL] Directional bias analysis requires BOTH standing_map_0deg and standing_map_180deg folders.")
        sys.exit(1)

    box_data_compiled = []

    print("Extracting specific box quadrant pairs from directional test files...")
    for env_name, env_path in valid_paths.items():
        excel_files = list(env_path.glob("Total_*.xlsx"))
        if not excel_files:
            continue
            
        file_to_open = excel_files[0]
        try:
            xls = pd.ExcelFile(file_to_open)
            sheets_to_ignore = ['master_data', 'averages_dashboard', 'regression_metrics']
            
            for sheet in xls.sheet_names:
                sheet_lower = sheet.lower()
                if sheet_lower in sheets_to_ignore:
                    continue
                
                # Exclude middle boxes immediately
                if 'mid' in sheet_lower or 'center' in sheet_lower:
                    continue
                
                # Explicit targeting filter for front/back left/right box quadrants
                if any(q in sheet_lower for q in ['front_lf', 'front_rt', 'back_lf', 'back_rt', 'front lf', 'front rt', 'back lf', 'back rt']):
                    df = pd.read_excel(xls, sheet_name=sheet)
                    df['Test_Environment'] = env_name
                    df['Box_Identifier'] = sheet  
                    
                    # Standardize names for clear legend presentation
                    if 'front' in sheet_lower and ('lf' in sheet_lower or 'left' in sheet_lower):
                        df['Quadrant_Class'] = 'Front Left Box'
                    elif 'front' in sheet_lower and ('rt' in sheet_lower or 'right' in sheet_lower):
                        df['Quadrant_Class'] = 'Front Right Box'
                    elif 'back' in sheet_lower and ('lf' in sheet_lower or 'left' in sheet_lower):
                        df['Quadrant_Class'] = 'Back Left Box'
                    elif 'back' in sheet_lower and ('rt' in sheet_lower or 'right' in sheet_lower):
                        df['Quadrant_Class'] = 'Back Right Box'
                    else:
                        df['Quadrant_Class'] = sheet
                        
                    box_data_compiled.append(df)
                    
        except Exception as e:
            print(f"[ERROR] Failed to parse box data from {file_to_open.name}: {e}")

    if not box_data_compiled:
        print("[CRITICAL] No matching quadrant box tabs found in the target files.")
        sys.exit(1)

    combined_box_df = pd.concat(box_data_compiled, ignore_index=True)

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

    output_dir = base_dir / "Directional_Bias_Analysis"
    output_dir.mkdir(exist_ok=True)
    print(f"\nGenerating discrete color-coded figures into: {output_dir.name}/")

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 13,
        'axes.labelweight': 'bold',
        'axes.titlesize': 14,
        'axes.titleweight': 'bold',
        'legend.frameon': True,
        'legend.edgecolor': '#000000'
    })

    # Strict Unique Color Palette for every line combination (No shared hues, solid fills)
    line_color_map = {
        ('Front Left Box', 'standing_map_0deg'): '#d62728',    # Crimson Red
        ('Front Left Box', 'standing_map_180deg'): '#ff9896',  # Light Pink (Will skip if absent)
        ('Front Right Box', 'standing_map_0deg'): '#ff7f0e',   # Dark Orange
        ('Front Right Box', 'standing_map_180deg'): '#fdbf6f',  # Light Yellow-Orange
        ('Back Left Box', 'standing_map_0deg'): '#1f77b4',     # Royal Blue
        ('Back Left Box', 'standing_map_180deg'): '#a6cee3',    # Light Blue
        ('Back Right Box', 'standing_map_0deg'): '#2ca02c',    # Forest Green
        ('Back Right Box', 'standing_map_180deg'): '#b2df8a'   # Light Green
    }

    # Distinct marker shapes per quadrant to guarantee scannability
    marker_map = {
        'Front Left Box': 'o',   # Circle
        'Front Right Box': 'v',  # Triangle Down
        'Back Left Box': 's',    # Square
        'Back Right Box': 'D'    # Diamond
    }

    for idx, (x_col, y_col, title) in enumerate(metric_pairs, 1):
        if x_col not in combined_box_df.columns or y_col not in combined_box_df.columns:
            continue

        fig, ax = plt.subplots(figsize=(11, 7))

        # Re-group by quadrant and test condition
        for (quadrant, env), group_df in combined_box_df.groupby(['Quadrant_Class', 'Test_Environment']):
            sorted_df = group_df.sort_values(by=x_col)
            
            # 0 deg = Solid line, 180 deg = Dashed line
            line_style = '-' if env == 'standing_map_0deg' else '--'
            env_label = "0°" if env == 'standing_map_0deg' else "180°"
            
            ax.plot(
                sorted_df[x_col], sorted_df[y_col],
                color=line_color_map.get((quadrant, env), '#7f7f7f'),
                linestyle=line_style,
                marker=marker_map.get(quadrant, 'x'),
                linewidth=2.5,
                markersize=7,
                label=f"{quadrant} ({env_label})"
            )

        ax.set_title(f"Directional Bias Assessment: Quadrant Metrics\n{title}", pad=12)
        ax.set_xlabel(x_col.replace('_', ' '))
        ax.set_ylabel(y_col.replace('_', ' '))
        
        ax.legend(loc='best', framealpha=1.0, edgecolor='black')
        plt.tight_layout()

        safe_title = title.replace(' ', '_').replace(':', '')
        plt.savefig(output_dir / f"Bias_{idx:02d}_{safe_title}.png", dpi=300)
        plt.close()

    print(f"\n[SUCCESS] Custom quadrant mapping finalized. Middle elements omitted successfully.")

if __name__ == "__main__":
    main()