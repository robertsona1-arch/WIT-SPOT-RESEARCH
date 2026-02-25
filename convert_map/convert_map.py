"""
python3 convert_map.py /path/to/downloaded/map --output converted.ply
"""
import sys
import os
import argparse
import struct
from bosdyn.api.graph_nav import map_pb2

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('map_dir', help='Path to the folder containing the downloaded map')
    parser.add_argument('--output', help='Name of the output file (defaults to converted.ply)')
    options = parser.parse_args(argv)

    if not os.path.exists(options.map_dir):
        print(f"[ERROR] Directory not found: {options.map_dir}")
        sys.exit(1)

    snap_dir = os.path.join(options.map_dir, 'waypoint_snapshots')
    if not os.path.exists(snap_dir):
        print(f"[ERROR] Could not find 'waypoint_snapshots' inside {options.map_dir}")
        sys.exit(1)

    all_points = []
    files = os.listdir(snap_dir)
    print(f"Found {len(files)} snapshots. Extracting points...")

    for filename in files:
        if filename == '.DS_Store': 
            continue # Ignore Mac hidden files
            
        file_path = os.path.join(snap_dir, filename)
        
        # 1. Initialize an empty Protobuf object
        snapshot = map_pb2.WaypointSnapshot()
        
        # 2. Open the binary file and parse it
        with open(file_path, 'rb') as f:
            snapshot.ParseFromString(f.read())
            
        # 3. Extract the point cloud float data
        cloud = snapshot.point_cloud
        if not cloud.data: 
            continue
            
        iter_points = struct.iter_unpack('<3f', cloud.data)
        for p in iter_points:
            all_points.append(p)

    print(f"Total points extracted: {len(all_points)}")
    
    # 4. Write to PLY
    out_file = options.output if options.output else os.path.join(options.map_dir, "converted.ply")
    write_ply(out_file, all_points)

def write_ply(filename, points):
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")
            
    print(f"Success! Saved to: {filename}")

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)