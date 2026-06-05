import open3d as o3d
import numpy as np
import os

def get_rot_matrix(theta_x, theta_y, theta_z):
    # Convert degrees to radians
    tx, ty, tz = np.radians([theta_x, theta_y, theta_z])
    
    Rx = np.array([
        [1, 0, 0], 
        [0, np.cos(tx), -np.sin(tx)], 
        [0, np.sin(tx), np.cos(tx)]
    ])
    Ry = np.array([
        [np.cos(ty), 0, np.sin(ty)], 
        [0, 1, 0], 
        [-np.sin(ty), 0, np.cos(ty)]
    ])
    Rz = np.array([
        [np.cos(tz), -np.sin(tz), 0], 
        [np.sin(tz), np.cos(tz), 0], 
        [0, 0, 1]
    ])
    
    # Standard Yaw-Pitch-Roll matrix multiplication: Rz * Ry * Rx
    return Rz @ Ry @ Rx 

def main():
    # Insert the path to one of your RAW, un-rotated PLY files here
    input_file = "/Users/adamrobertson/Documents/spot_tests/6:3:26/lock test/normal/test_n_02/converted_n_2.ply" 
    
    if not os.path.exists(input_file):
        print(f"Error: Cannot find {input_file}")
        print("Please provide a path to a raw baseline PLY file.")
        return
        
    pcd = o3d.io.read_point_cloud(input_file)
    raw_points = np.asarray(pcd.points)
    
    out_dir = "/Users/adamrobertson/Documents/spot_tests/6:3:26/lock test/converted_n_2_rotations"
    os.makedirs(out_dir, exist_ok=True)
    print(f"Generating 64 rotation permutations in ./{out_dir}/ ...")
    
    angles = [0, 90, 180, 270]
    
    for x in angles:
        for y in angles:
            for z in angles:
                R = get_rot_matrix(x, y, z)
                
                # Apply the 3x3 rotation matrix to the Nx3 point array
                rotated_points = raw_points @ R.T 
                
                new_pcd = o3d.geometry.PointCloud()
                new_pcd.points = o3d.utility.Vector3dVector(rotated_points)
                
                # Tag the file with its exact rotation constants
                filename = f"rot_X{x}_Y{y}_Z{z}.ply"
                o3d.io.write_point_cloud(os.path.join(out_dir, filename), new_pcd)
                
    print("Done. Open CloudCompare, load the folder, and visually isolate the perfect match.")

if __name__ == "__main__":
    main()