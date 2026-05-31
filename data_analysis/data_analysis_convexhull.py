import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull

def calculate_xy_volume(ply_path, x_range, y_range):
    """
    Loads a point cloud, isolates points within an X-Y bounding box, 
    and calculates the geometric volume of that cluster.
    Ranges should be tuples: (min, max) in meters.
    """
    # 1. Load the data
    print(f"Loading {ply_path}...")
    pcd = o3d.io.read_point_cloud(ply_path)
    points = np.asarray(pcd.points)
    
    # 2. Filter by the X-Y boundary (ignoring Z height initially)
    mask_x = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])
    mask_y = (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])
    
    filtered_points = points[mask_x & mask_y]
    
    # 3. Validation Check
    # A 3D Convex Hull requires a minimum of 4 non-coplanar points
    if len(filtered_points) < 4:
        print("Error: Not enough points in this X-Y area to form a 3D volume.")
        return 0.0
        
    # 4. Calculate the Volume
    # SciPy handles the complex Delaunay triangulation math automatically
    hull = ConvexHull(filtered_points)
    volume = hull.volume
    
    print(f"Total points in X-Y boundary: {len(filtered_points)}")
    print(f"Calculated Volume: {volume:.4f} cubic meters")
    
    return volume

# Example Execution: Look at a 2x2 meter square directly on the ground
calculate_xy_volume("spot_test_run_1.ply", x_range=(0.0, 2.0), y_range=(-1.0, 1.0))