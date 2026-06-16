import argparse
import pandas as pd
from pathlib import Path
import re
import sys

def main():
    parser = argparse.ArgumentParser(description="Aggregate Spot active stereo test averages per object and summary dashboards.")
    parser.add_argument('--folder', required=True, help="Root directory containing the subfolders")
    args = parser.parse_args()

    base_dir = Path(args.folder)
    if not base_dir.exists():
        print(f"[ERROR] Directory '{base_dir}' does not exist.")
        sys.exit(1)

    # Dictionary tree: test_groups[test_type][sheet_name] = [df1, df2, ...]
    test_groups = {'C': {}, 'V': {}, 'S': {}, 'R': {}}

    print(f"Scanning {base_dir.absolute()} and all subdirectories for test data...\n")

    # Only ignoring the raw logs and derived regressions; we now INCLUDE 'Averages_Dashboard'
    sheets_to_ignore = ['Regression_Metrics', 'Master_Data']
    files_found = 0

    for file_path in base_dir.rglob("*.xlsx"):
        if file_path.name.startswith("Total_Averages") or file_path.name.startswith("~"):
            continue
            
        files_found += 1
        
        # Match test group identifiers (C, V, S, or R followed by run 1-5)
        match = re.search(r'([CVSR])[1-5]', file_path.as_posix(), re.IGNORECASE)
        
        if match:
            test_type = match.group(1).upper()
            print(f"  [MATCH] Found '{file_path.name}' in '{file_path.parent.name}' -> Group {test_type}")
            try:
                xls = pd.ExcelFile(file_path)
                
                for sheet in xls.sheet_names:
                    if sheet in sheets_to_ignore:
                        continue
                        
                    df = pd.read_excel(xls, sheet_name=sheet)
                    
                    if sheet not in test_groups[test_type]:
                        test_groups[test_type][sheet] = []
                        
                    test_groups[test_type][sheet].append(df)
            except Exception as e:
                print(f"  [WARNING] Could not read from {file_path.name}: {e}")
        else:
            print(f"  [IGNORED] '{file_path.name}' in '{file_path.parent.name}' (Missing C, V, S, or R + 1-5)")

    if files_found == 0:
        print("\n[ERROR] No .xlsx files were found. Verify your target execution directory.")
        sys.exit(1)

    print("\nExecuting comprehensive aggregations (Objects + Dashboard summaries)...\n")
    for test_type, sheets_dict in test_groups.items():
        if not sheets_dict:
            continue

        print(f"Processing Group {test_type}...")
        output_filename = base_dir / f"Total_Averages_Group_{test_type}.xlsx"
        
        with pd.ExcelWriter(output_filename, engine='openpyxl') as writer:
            # We sort keys to ensure object sheets write first, and the Dashboard summary writes last
            for sheet_name in sorted(sheets_dict.keys(), key=lambda x: x == 'Averages_Dashboard'):
                combined_df = pd.concat(sheets_dict[sheet_name], ignore_index=True)
                
                # Compute the mean for numeric data grouped by snapshot count
                averaged_df = combined_df.groupby('Snap_Count').mean(numeric_only=True).reset_index()
                averaged_df.to_excel(writer, index=False, sheet_name=sheet_name)
            
        print(f"  [SUCCESS] Exported all metrics and summary dashboards to: {output_filename.name}")

    print("\nData aggregation complete. Ready for Seaborn visualization.")

if __name__ == "__main__":
    main()