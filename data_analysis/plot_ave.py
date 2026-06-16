import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="Generate IEEE-formatted plots for Spot SAR data.")
    parser.add_argument('--file', required=True, help="Path to the Total_Averages_Group_X.xlsx file")
    args = parser.parse_args()

    file_path = Path(args.file)
    
    # 1. Load Data & Convert to Long Format
    xls = pd.ExcelFile(file_path)
    all_data = []
    
    for sheet in xls.sheet_names:
        df = pd.read_excel(xls, sheet_name=sheet)
        df['Object'] = sheet  # Tag the data with the object name
        all_data.append(df)
        
    combined_df = pd.concat(all_data, ignore_index=True)

    # 2. Compute the Global Average across all objects per Snap_Count
    # We group by Snap_Count and Time_s, taking the mean of everything else
    global_avg = combined_df.groupby(['Snap_Count', 'Time_s']).mean(numeric_only=True).reset_index()
    global_avg['Object'] = 'Global Average'

    # Append the average back into the main DataFrame
    final_df = pd.concat([combined_df, global_avg], ignore_index=True)

    # 3. IEEE Plotting Standards Configuration
    sns.set_theme(context="paper", style="whitegrid", font_scale=1.2)
    # Define a custom color palette: colored lines for objects, strict black for the average
    objects = final_df['Object'].unique()
    palette = sns.color_palette("husl", len(objects) - 1)
    color_dict = {obj: color for obj, color in zip(xls.sheet_names, palette)}
    color_dict['Global Average'] = 'black'

    # Define line styles: solid for objects, dashed for the average
    style_dict = {obj: "" for obj in xls.sheet_names}
    style_dict['Global Average'] = (2, 2) # Dash format

    # 4. Generate Graph A: Density vs. Snapshot Count
    plt.figure(figsize=(8, 5))
    ax1 = sns.lineplot(
        data=final_df, x='Snap_Count', y='Density: Pts Per m3', 
        hue='Object', palette=color_dict, style='Object', dashes=style_dict, 
        linewidth=2.5, markers=True, markersize=8
    )
    plt.title("Volumetric Point Density vs. Snapshot Count")
    plt.xlabel("Snapshot Count (n)")
    plt.ylabel("Point Density (pts/m³)")
    plt.tight_layout()
    plt.savefig(file_path.parent / "Graph_A_Density_vs_Snap.png", dpi=300)
    plt.close()

    # 5. Generate Graph B: The SAR Efficiency Curve (Density vs Time)
    plt.figure(figsize=(8, 5))
    ax2 = sns.lineplot(
        data=final_df, x='Time_s', y='Density: Pts Per m3', 
        hue='Object', palette=color_dict, style='Object', dashes=style_dict,
        linewidth=2.5, markers=True, markersize=8
    )
    plt.title("SAR Efficiency: Point Density vs. Operational Latency")
    plt.xlabel("Total Map Compilation Time (seconds)")
    plt.ylabel("Point Density (pts/m³)")
    plt.tight_layout()
    plt.savefig(file_path.parent / "Graph_B_Efficiency_Curve.png", dpi=300)
    plt.close()
    
    print(f"[SUCCESS] Generated IEEE-compliant graphs in {file_path.parent.absolute()}")

if __name__ == "__main__":
    main()