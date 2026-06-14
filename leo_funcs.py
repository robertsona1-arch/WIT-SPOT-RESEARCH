"""
leo_funcs.py
This file contains all the functions used in any script. 
At the top are all the imports 
Below this is how to use 
"""

"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert, Ryan Staley
Date Created: 4/1/2026
Last Updated: 5/17/2026
"""

"""
# Dynamically point Python to the root repository folder
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# Now you can import the functions cleanly
from leo_funcs import fine_align

"""
import argparse
import logging
import os
import struct #ply conversion
import sys
import time
import traceback
import math
from datetime import datetime
from unittest.mock import MagicMock
import numpy as np
import open3d as o3d
import csv
import json
import glob
import pandas 
import re 
import seaborn
import matplotlib.pyplot as plt
import openpyxl

#bd specific imports
import google.protobuf.timestamp_pb2
#import graph_nav_util
import bosdyn.client.channel 
import bosdyn.client.util
import bosdyn.client.graph_nav 
import bosdyn.client

from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2, robot_command_pb2 as generic_robot_command_pb2, trajectory_pb2, world_object_pb2, basic_command_pb2
from bosdyn.api.gps import gps_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2, map_processing_pb2, recording_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client import map_processing
from bosdyn.client.robot import Robot
from bosdyn.client.lease import LeaseKeepAlive
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_se2_a_tform_b, BODY_FRAME_NAME, get_a_tform_b, VISION_FRAME_NAME
#from bosdyn.client.graph_nav_recording import GraphNavRecordingClient # Standalone in 5.x
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.world_object import WorldObjectClient

#google imports
import grpc

from google.protobuf import wrappers_pb2 as wrappers

def check_batt_perc(robot_state_client,limit=20.0):
    """
    Check battery percentage using protobuf path:
    state.power_state.locomotion_charge_percentage.value
    """
    state=robot_state_client.get_robot_state()

    #check if field exists
    if not state.power_state.HasField('locomotion_charge_percentage'):
        print("\nBattery percentage field not found, assuming sufficient charge\n")
        return True
    
    #Access .value 
    charge= state.power_state.locomotion_charge_percentage.value

    print(f"\nBattery check, charge: {charge:.2f}%\n")

    if charge < limit:
        return False
    return True

def turn_relative(command_client,robot_state_client,yaw_deg):
    yaw_rad=math.radians(yaw_deg)
    transforms=robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    odom_t_body=get_se2_a_tform_b(transforms, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
    new_yaw=odom_t_body.angle+yaw_rad

    #set speed limits using geometry api first
    speed_limit=geometry_pb2.SE2VelocityLimit(
        max_vel=geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=0.5,y=0.5),angular=1.0
        )
    )

    #initialize params and attach speed limits
    params=spot_command_pb2.MobilityParams(vel_limit=speed_limit)

    se2_pose = geometry_pb2.SE2Pose(
        position=geometry_pb2.Vec2(
            x=odom_t_body.x, y=odom_t_body.y)
            ,angle=new_yaw)

    duration=abs(yaw_rad)/0.8
    if duration<2.0: duration=2.0

    cmd = RobotCommandBuilder.synchro_se2_trajectory_command(
        se2_pose,
        frame_name=ODOM_FRAME_NAME,
        params=params  # This is the object built on standalone lines earlier
    )
    command_client.robot_command(cmd, end_time_secs=time.time()+duration)

    time.sleep(duration+0.5)

def convert_map_to_ply(map_dir, output_file,n, transform_matrix=None): #none means its optional 
    """Extracts points directly from the raw Protobuf files and saves a .PLY file"""
    #removed the tranformation from frame 1, don't need it
    snap_dir = os.path.join(map_dir, 'waypoint_snapshots')
    if n!=1:
        graph_path=os.path.join(map_dir, 'graph')
    
    
    if not os.path.exists(snap_dir):
        print(f"  [ERROR] Could not find 'waypoint_snapshots' inside {map_dir}")
        return
    
    if n!=1:
        if not os.path.exists(graph_path):
            print(f"\n graph address error\n")
            return

    if n!=1:
        #New, read graph to get transformations
        graph=map_pb2.Graph()
        with open(graph_path,'rb') as f:
            graph.ParseFromString(f.read())

        #Map each waypoint ID to its specifc KO transform
        waypoint_transforms={}
        for wp in graph.waypoints:
            #waypoint_tform_ko takes points from KO and puts them in Waypoint. 
            #we need the inverse: takes points from Waypoint and puts them in KO, so we invert the transform
            #kinematic odometry (KO) is the robot's internal estimate of its position, so we want to transform the point cloud from the waypoint frame back to the KO frame for consistency across snapshots
            wp_tform_ko=math_helpers.SE3Pose.from_proto(wp.waypoint_tform_ko)
            ko_tform_wp=wp_tform_ko.inverse()#take the inverse
            waypoint_transforms[wp.snapshot_id]=ko_tform_wp #changed from wp.id because it couldnt find the graphs

    all_points = []
    
    try:
        files = os.listdir(snap_dir)
        for filename in files:
            # Ignore macOS hidden system files that crash the binary parser
            if filename == '.DS_Store':
                continue
                
            file_path = os.path.join(snap_dir, filename)
            snapshot = map_pb2.WaypointSnapshot()
            
            with open(file_path, 'rb') as f:
                snapshot.ParseFromString(f.read())
                
            cloud = snapshot.point_cloud
            if not cloud.data:
                continue

            if n!=1:
                #get specific transform for this snapshot
                if snapshot.id not in waypoint_transforms:
                    print(f"\nNo tranform found for waypoint {snapshot.id}, skipping this snapshot\n")
                    continue
                ko_tform_wp=waypoint_transforms[snapshot.id]

            #unpack and transform points
            iter_points = struct.iter_unpack('<3f', cloud.data)
            for p in iter_points:
                if n!=1:
                    #apply transformation matrix to align the frame w/ origin frame
                    #transformation_point handles 3d vector rotation+translation
                    global_p=ko_tform_wp.transform_point(p[0],p[1],p[2])
                    all_points.append(global_p)
                else:
                    all_points.append(p)

        # Write to PLY format
        with open(output_file, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(all_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            for p in all_points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
                
    except Exception as e:
        print(f"  [CRITICAL ERROR] Conversion failed: {e}")

def convert_map_to_ply_2(map_dir, output_file, n, transform_matrix=None): #none means its optional 
    """Extracts points directly from the raw Protobuf files and saves a .PLY file"""
    #removed the tranformation from frame 1, don't need it
    snap_dir = os.path.join(map_dir, 'waypoint_snapshots')
    if n!=1:
        graph_path=os.path.join(map_dir, 'graph')
    
    if not os.path.exists(snap_dir):
        print(f"  [ERROR] Could not find 'waypoint_snapshots' inside {map_dir}")
        return
    
    if n!=1:
        if not os.path.exists(graph_path):
            print(f"\n graph address error\n")
            return

    if n!=1:
        #New, read graph to get transformations
        graph=map_pb2.Graph()
        with open(graph_path,'rb') as f:
            graph.ParseFromString(f.read())

        #Map each waypoint ID to its specifc KO transform
        waypoint_transforms={}
        for wp in graph.waypoints:
            #waypoint_tform_ko takes points from KO and puts them in Waypoint. 
            #we need the inverse: takes points from Waypoint and puts them in KO, so we invert the transform
            #kinematic odometry (KO) is the robot's internal estimate of its position, so we want to transform the point cloud from the waypoint frame back to the KO frame for consistency across snapshots
            wp_tform_ko=math_helpers.SE3Pose.from_proto(wp.waypoint_tform_ko)
            ko_tform_wp=wp_tform_ko.inverse()#take the inverse
            waypoint_transforms[wp.snapshot_id]=ko_tform_wp #changed from wp.id because it couldnt find the graphs

    all_points = []
    
    try:
        files = os.listdir(snap_dir)
        for filename in files:
            # Ignore macOS hidden system files that crash the binary parser
            if filename == '.DS_Store':
                continue
                
            file_path = os.path.join(snap_dir, filename)
            snapshot = map_pb2.WaypointSnapshot()
            
            with open(file_path, 'rb') as f:
                snapshot.ParseFromString(f.read())
                
            cloud = snapshot.point_cloud
            if not cloud.data:
                continue

            if n!=1:
                #get specific transform for this snapshot
                if snapshot.id not in waypoint_transforms:
                    print(f"\nNo tranform found for waypoint {snapshot.id}, skipping this snapshot\n")
                    continue
                ko_tform_wp=waypoint_transforms[snapshot.id]

            #unpack and transform points
            iter_points = struct.iter_unpack('<3f', cloud.data)
            for p in iter_points:
                if n!=1:
                    #apply transformation matrix to align the frame w/ origin frame
                    #transformation_point handles 3d vector rotation+translation
                    global_p=ko_tform_wp.transform_point(p[0],p[1],p[2])
                    all_points.append(global_p)
                else:
                    all_points.append(p)

        #new: apply the 4x4 transform matrix to the entire point cloud at once
        if transform_matrix is not None and len(all_points) > 0:
            #new: convert list to a numpy array for fast C-backend matrix math
            pts = np.array(all_points)
            
            #new: pad the array with a column of 1s to allow 4x4 homogeneous multiplication
            pts_homo = np.hstack((pts, np.ones((pts.shape[0], 1))))
            
            #new: apply the transformation matrix (transpose handles the correct axis broadcasting)
            transformed_pts = pts_homo.dot(transform_matrix.T)
            
            #new: extract the shifted X, Y, and Z columns and overwrite the original list
            all_points = transformed_pts[:, :3].tolist()

        # Write to PLY format
        with open(output_file, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(all_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            for p in all_points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
                
    except Exception as e:
        print(f"  [CRITICAL ERROR] Conversion failed: {e}")

def convert_map_to_ply_3(map_dir, output_file, n, transform_matrix=None):
    """Extracts points from Protobuf files, maps them to Odom, and applies global transforms."""
    snap_dir = os.path.join(map_dir, 'waypoint_snapshots')
    graph_path = os.path.join(map_dir, 'graph')
    
    if not os.path.exists(snap_dir) or not os.path.exists(graph_path):
        print(f"  [ERROR] Missing snapshots or graph inside {map_dir}")
        return

    # Read graph to get transformations for ALL snapshots
    graph = map_pb2.Graph()
    with open(graph_path, 'rb') as f:
        graph.ParseFromString(f.read())

    waypoint_transforms = {}
    for wp in graph.waypoints:
        wp_tform_ko = math_helpers.SE3Pose.from_proto(wp.waypoint_tform_ko)
        ko_tform_wp = wp_tform_ko.inverse() 
        waypoint_transforms[wp.snapshot_id] = ko_tform_wp 

    all_points = []
    
    try:
        files = os.listdir(snap_dir)
        for filename in files:
            if filename == '.DS_Store':
                continue
                
            file_path = os.path.join(snap_dir, filename)
            snapshot = map_pb2.WaypointSnapshot()
            
            with open(file_path, 'rb') as f:
                snapshot.ParseFromString(f.read())
                
            cloud = snapshot.point_cloud
            if not cloud.data:
                continue

            if snapshot.id not in waypoint_transforms:
                print(f"\nNo transform found for waypoint {snapshot.id}, skipping.\n")
                continue
                
            ko_tform_wp = waypoint_transforms[snapshot.id]

            # Unpack and transform ALL points to the Kinematic Odometry frame
            iter_points = struct.iter_unpack('<3f', cloud.data)
            for p in iter_points:
                global_p = ko_tform_wp.transform_point(p[0], p[1], p[2])
                all_points.append(global_p)

        # Apply the global 4x4 Anchor matrix if provided
        if transform_matrix is not None and len(all_points) > 0:
            pts = np.array(all_points)
            pts_homo = np.hstack((pts, np.ones((pts.shape[0], 1))))
            transformed_pts = pts_homo.dot(transform_matrix.T)
            all_points = transformed_pts[:, :3].tolist()

        # Write to PLY format
        with open(output_file, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(all_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            for p in all_points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
                
    except Exception as e:
        print(f"  [CRITICAL ERROR] Conversion failed: {e}")

def nav_to_fid(robot, tag_id, dist_m):
    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    
    # 1. Localize (Snaps the Seed Frame to the Real World)
    print(f"Localizing to Fiducial ID: {tag_id}...")
    empty_guess = nav_pb2.Localization()
    try:
        graph_nav_client.set_localization(
            initial_guess_localization=empty_guess,
            fiducial_init=4,
            use_fiducial_id=int(tag_id)
        )
        print("Localization successful.")
    except Exception as e:
        print(f"Localization failed: {e}")
        return False

    # 2. Get Current State (Seed -> Body)
    localization_state = graph_nav_client.get_localization_state()
    seed_tform_body = SE3Pose.from_proto(localization_state.localization.seed_tform_body)

    # 3. Ask Perception for the Tag's location (Body -> Fiducial)
    world_objects = world_object_client.list_world_objects(
        object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
    ).world_objects

    fiducial_obj = next((obj for obj in world_objects if obj.apriltag_properties.tag_id == int(tag_id)), None)
    print(f"\nPerception reports {len(world_objects)} AprilTags in view.\n")
    if not fiducial_obj:
        print("Error: Tag not currently visible to cameras.")
        return False

    body_tform_fiducial = get_a_tform_b(
        fiducial_obj.transforms_snapshot, 
        BODY_FRAME_NAME, 
        fiducial_obj.apriltag_properties.frame_name_fiducial
    )

    #project tag into global map frame
    seed_tform_fiducial = seed_tform_body * body_tform_fiducial

    # 4. Push dist_m straight out from the tag (No rotation yet)
    tag_z_out = SE3Pose(x=0.0, y=0.0, z=dist_m, rot=Quat())
    raw_seed_goal = seed_tform_fiducial * tag_z_out

    # 5. Calculate absolute map heading to look AT the tag
    dy = seed_tform_fiducial.y - raw_seed_goal.y
    dx = seed_tform_fiducial.x - raw_seed_goal.x
    heading_rads = math.atan2(dy, dx)

    # 6. Build the final Seed Pose (Grounded Z + Trigonometric Yaw)
    seed_tform_goal = SE3Pose(
        x=raw_seed_goal.x,
        y=raw_seed_goal.y,
        z=seed_tform_body.z, 
        rot=Quat.from_yaw(heading_rads) 
    )
    
    # --- DIAGNOSTIC PRINT BLOCK ---
    print("\n--- MATRIX DIAGNOSTICS ---")
    print(f"Robot Current Pose (Seed Frame):  X: {seed_tform_body.x:.2f}, Y: {seed_tform_body.y:.2f}, Z: {seed_tform_body.z:.2f}")
    print(f"Calculated Goal    (Seed Frame):  X: {seed_tform_goal.x:.2f}, Y: {seed_tform_goal.y:.2f}, Z: {seed_tform_goal.z:.2f}")
    
    # Calculate the straight-line distance between where we are and where we want to go
    dx = seed_tform_goal.x - seed_tform_body.x
    dy = seed_tform_goal.y - seed_tform_body.y
    distance_to_goal = math.sqrt(dx**2 + dy**2)
    print(f"Delta (Distance to Goal):         {distance_to_goal:.2f} meters")
    print("--------------------------\n")
    
    # Capture the navigation ID when you issue the command
    print("Navigating to target coordinate...")
    nav_id = graph_nav_client.navigate_to_anchor(
        seed_tform_goal=seed_tform_goal.to_proto(),
        cmd_duration=30.0
    )

    print("Monitoring navigation status...")
    while True:
        feedback = graph_nav_client.navigation_feedback(nav_id)
        
        # We explicitly use the Protobuf variables now, no integers
        if feedback.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            print("\nGraphNav success: Arrived in front of fiducial.")
            break
            
        elif feedback.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_FOLLOWING_ROUTE:
            # This is what '1' actually meant. Now we just let it keep walking.
            print("Robot is walking to the target...", end="\r")
             
        elif feedback.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_ROUTE:
            print("\nERROR: Path Planner cannot find a safe route to the target.")
            return False
            
        elif feedback.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print("\nERROR: Robot got lost. Check map alignment.")
            return False
            
        elif feedback.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print("\nERROR: Robot got stuck. Check for physical obstacles.")
            return False
            
        time.sleep(1.0)
    
    return True

def fine_align(robot, tag_id, dist,iter):
    print("\n--- INITIATING PHASE 2: CLOSED-LOOP ALIGNMENT ---")
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    a=0
    dist_thrsh=0.075
    deg_thrsh=2.5

    for attempt in range(iter):
        if a%5==0 and a!=0:
            dist_thrsh+=0.025
            deg_thrsh+=1.0

        print(f"\nAlignment Pass {attempt + 1}/{iter}")
        
        # 1. Take a fresh picture of the tag
        world_objects = world_object_client.list_world_objects(
            object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
        ).world_objects
        
        fiducial_obj = next((obj for obj in world_objects if obj.apriltag_properties.tag_id == int(tag_id)), None)
        if not fiducial_obj:
            print("Error: Lost sight of tag during visual servoing.")
            return None,None

        # 2. Extract Transform to ODOM
        tag_frame_name = fiducial_obj.apriltag_properties.frame_name_fiducial
        odom_tform_fiducial = get_a_tform_b(fiducial_obj.transforms_snapshot, ODOM_FRAME_NAME, tag_frame_name)

        # 3. Calculate target and heading
        tag_z_out = SE3Pose(x=0.0, y=0.0, z=dist, rot=Quat())
        raw_goal_odom = odom_tform_fiducial * tag_z_out
        
        dy = odom_tform_fiducial.y - raw_goal_odom.y
        dx = odom_tform_fiducial.x - raw_goal_odom.x
        heading_rads = math.atan2(dy, dx)

        # 4. Calculate Current Error vs Goal
        odom_tform_body = get_a_tform_b(fiducial_obj.transforms_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)
        
        dist_error = math.sqrt((raw_goal_odom.x - odom_tform_body.x)**2 + (raw_goal_odom.y - odom_tform_body.y)**2)
        yaw_error = abs(heading_rads - odom_tform_body.rot.to_yaw())
        
        print(f"Current Error -> Distance: {dist_error:.4f}m | Heading: {math.degrees(yaw_error):.2f} deg")
        
        #manual check
        while True:
            user_input=input("\nIs this starting spot accetable?\ny or n\n").strip().lower()
            if user_input in ['y','yes']:
                print("\nManually set to go, beginning\n")
                return dist_error, yaw_error
            elif user_input in ['n','no']:
                print("\nRetrying\n")
                break
            else:
                print("\nInvalid input, please type y or n then press enter\n")
                break
        # 5. The Threshold Check (e.g., within 1.5cm and 1.5 degrees)
        if dist_error < dist_thrsh and math.degrees(yaw_error) < deg_thrsh:
            print("SUCCESS: Experimental alignment tolerances achieved.")
            return dist_error, yaw_error

        # 6. Execute Correction Command
        goal_se2 = SE2Pose(raw_goal_odom.x, raw_goal_odom.y, heading_rads)
        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goal_se2.x, goal_y=goal_se2.y, goal_heading=goal_se2.angle,
            frame_name=ODOM_FRAME_NAME, params=RobotCommandBuilder.mobility_params(stair_hint=False)
        )
        
        cmd_id = command_client.robot_command(command, end_time_secs=time.time() + 15.0)
        
        # Block until the micro-adjustment settles before taking the next picture
        start_time = time.time()
        while time.time() - start_time < 10.0:
            feedback = command_client.robot_command_feedback(cmd_id)
            status = feedback.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
            if status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL:
                time.sleep(1.0) # Let the camera physically stabilize
                break
            time.sleep(0.5)
        a+=1

    print("WARNING: Max iterations reached without hitting strict tolerances. Proceeding with best effort.")
    return None,None

def upload_map(graph_nav_client, map_dir):
    # 1. Load and Upload the Graph (The Skeleton)
    with open(os.path.join(map_dir, "graph"), "rb") as f:
        graph_data = f.read()
    graph = map_pb2.Graph()
    graph.ParseFromString(graph_data)
    
    print("Uploading graph...")
    graph_nav_client.upload_graph(graph=graph)

    # 2. Upload Waypoint Snapshots (The Muscle)
    snapshot_dir = os.path.join(map_dir, "waypoint_snapshots")
    waypoint_ids = [wp.snapshot_id for wp in graph.waypoints]

    for snapshot_id in waypoint_ids:
        snapshot_path = os.path.join(snapshot_dir, snapshot_id)
        if os.path.exists(snapshot_path):
            with open(snapshot_path, "rb") as f:
                snapshot_data = f.read()
            
            snapshot = map_pb2.WaypointSnapshot()
            snapshot.ParseFromString(snapshot_data)
            
            print(f"Uploading snapshot: {snapshot_id[:8]}...")
            # Using the standard call; async is better for large maps
            graph_nav_client.upload_waypoint_snapshot(snapshot)

    print("Map upload complete.")

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

def control_height(command_client,height,robot_state_client):

    #0.0 is neutral, 0.1 is high, -0.1 is low, height in meters
    z_offset=height

    #build the pose (position+rotation), w=1 is neutral quaternion
    footprint_R_body=geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0.0,y=0.0,z=z_offset),
        rotation=geometry_pb2.Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
    )

    #wrap pose in trajectory point
    point=trajectory_pb2.SE3TrajectoryPoint(
        pose=footprint_R_body,
        time_since_reference=google.protobuf.duration_pb2.Duration(seconds=0) #0 second to reach the target height immediately
    )

    #create trajectory w/single point
    traj=trajectory_pb2.SE3Trajectory(points=[point])

    #create control parameters
    body_control=spot_command_pb2.BodyControlParams(
        base_offset_rt_footprint=traj
    )

    # #create control parameters
    # body_control=spot_command_pb2.BodyControlParams(
    #     base_offset_rt_footprint=footprint_R_body
    # )

    #create mobility params and attach body control
    mobility_params=spot_command_pb2.MobilityParams(body_control=body_control)

    #build and send stand command  
    stand_cmd=RobotCommandBuilder.synchro_stand_command(params=mobility_params)
    command_client.robot_command(stand_cmd)

    #wait for stabilization
    time.sleep(2.0)

def log_test_metrics(map_dir, test_name, duration_secs, dist_error_m, yaw_error_deg=None, batt_used_p=None):
    """
    Appends execution time and final kinematic error to a continuous log file.
    """
    # 1. Ensure the target directory actually exists
    os.makedirs(map_dir, exist_ok=True)
    
    # 2. Build the exact file path
    log_filepath = os.path.join(map_dir, "alignment_metrics_log.txt")
    
    # 3. Generate a human-readable timestamp for the exact moment the test finished
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    
    # 4. Format the data string
    if yaw_error_deg is not None:
        log_entry = f"[{timestamp}] Test: {test_name:<10} | Time: {duration_secs:>5.2f}s | Dist Error: {dist_error_m:>6.4f}m | Yaw Error: {yaw_error_deg:>5.2f} deg\n"
    else:
        log_entry = f"[{timestamp}] Test: {test_name:<10} | Time: {duration_secs:>5.2f}s | Dist Error: {dist_error_m:>6.4f}m\n"
    
    if batt_used_p is not None:
        log_entry += f" | Battery Used: {batt_used_p:>5.2f}%\n"
    else:
        log_entry += "\n"

    # 5. Write to the file
    with open(log_filepath, "a") as file:
        file.write(log_entry)
        
    print(f"Data successfully appended to {log_filepath}")

def ensure_recording_stopped(robot):
    """
    Checks if the robot has an orphaned recording session running and forcefully kills it.
    """
    print("Checking Map Recording state...")
    recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
    
    try:
        # Poll the physical robot for its current state
        status = recording_client.get_record_status()
        
        if status.is_recording:
            print("WARNING: Orphaned recording session detected on robot hardware.")
            print("Forcefully stopping the previous session...")
            recording_client.stop_recording()
            print("Recording state cleared. Ready for new test.")
        else:
            print("Recording state clear. No active sessions.")
            
        return True
        
    except Exception as e:
        print(f"CRITICAL: Failed to communicate with Map Recording Client: {e}")
        return False
    
def calculate_aabb_volume(points, x_range, y_range):
    """
    Filters a pre-loaded numpy point array by an X-Y area and calculates 
    the AABB volume and point count.
    """
    # 1. Isolate the points within the defined X-Y column
    mask_x = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])
    mask_y = (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])
    
    filtered_points = points[mask_x & mask_y]
    
    if len(filtered_points) == 0:
        return 0.0, 0
        
    # 2. Find the absolute boundaries (This automatically finds the highest/lowest Z)
    min_bounds = np.min(filtered_points, axis=0)
    max_bounds = np.max(filtered_points, axis=0)
    
    # 3. Calculate volume (Length * Width * Height)
    dimensions = max_bounds - min_bounds
    volume = dimensions[0] * dimensions[1] * dimensions[2]
    
    return volume, len(filtered_points)

def parse_alignment_log(log_path):
    """Parses the text log and extracts metrics using Regex."""
    metrics = {}
    if not os.path.exists(log_path):
        print(f"Warning: Log file not found at {log_path}. Metrics will be blank.")
        return metrics
        
    # Regex pattern to capture the digits from your specific string format
    pattern = r"Test:\s*test_n_(\d+)\s*\|\s*Time:\s*([\d\.]+)s\s*\|\s*Dist Error:\s*([\d\.]+)m\s*\|\s*Yaw Error:\s*([-\d\.]+)\s*deg"
    
    with open(log_path, 'r') as f:
        for line in f:
            match = re.search(pattern, line)
            if match:
                test_num = int(match.group(1))
                metrics[test_num] = {
                    'Time_s': float(match.group(2)),
                    'Dist_Error_m': float(match.group(3)),
                    'Yaw_Error_deg': float(match.group(4))
                }
    return metrics

def create_plot(df, x_col, y_col, title, x_label, y_label, output_path, color='blue'):
    """Standardizes plot layout configuration, grid parameters, and clears memory."""
    plt.figure(figsize=(10, 6))
    seaborn.lineplot(data=df, x=x_col, y=y_col, marker='o', linewidth=2.5, color=color)
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel(x_label, fontsize=12)
    plt.ylabel(y_label, fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()

def create_plot_tl(df, x_col, y_col, title, x_label, y_label, output_path, color='blue'):
    """Standardizes plot layout, calculates a linear trendline and its R² value."""
    plt.figure(figsize=(10, 6))
    
    clean_df = df[[x_col, y_col]].dropna()
    clean_df = clean_df[np.isfinite(clean_df[x_col]) & np.isfinite(clean_df[y_col])]
    
    if len(clean_df) < 2:
        return

    x_data = clean_df[x_col].to_numpy()
    y_data = clean_df[y_col].to_numpy()

    # Plot raw data
    seaborn.lineplot(data=df, x=x_col, y=y_col, marker='o', linewidth=2, color=color, label='Raw Data')

    # Linear Fit
    slope, intercept = np.polyfit(x_data, y_data, 1)
    y_pred = slope * x_data + intercept

    # Calculate R²
    ss_res = np.sum((y_data - y_pred) ** 2)
    ss_tot = np.sum((y_data - np.mean(y_data)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    # Plot Trendline
    x_trend = np.linspace(x_data.min(), x_data.max(), 100)
    y_trend = slope * x_trend + intercept
    trend_label = f'Linear Fit (m={slope:.4f}, R²={r_squared:.4f})'
    plt.plot(x_trend, y_trend, linestyle='--', color='black', linewidth=1.5, label=trend_label)

    # Formatting
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel(x_label, fontsize=12)
    plt.ylabel(y_label, fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()

def create_plot_expd(df, x_col, y_col, title, x_label, y_label, output_path, color='blue'):
    """Standardizes plot layout, calculates an exponential decay line and its R² value."""
    plt.figure(figsize=(10, 6))
    
    clean_df = df[[x_col, y_col]].dropna()
    clean_df = clean_df[(np.isfinite(clean_df[x_col])) & (clean_df[y_col] > 0)]
    
    if len(clean_df) < 2:
        return

    x_data = clean_df[x_col].to_numpy()
    y_data = clean_df[y_col].to_numpy()

    # Plot raw data
    seaborn.lineplot(data=df, x=x_col, y=y_col, marker='o', linewidth=2, color=color, label='Raw Data')

    # Exponential Fit: ln(y) = b*x + ln(a)
    b, ln_a = np.polyfit(x_data, np.log(y_data), 1)
    a = np.exp(ln_a)
    
    # Predict values on the original scale to compute the correct physical residuals
    y_pred = a * np.exp(b * x_data)

    # Calculate R² based on true physical deviations
    ss_res = np.sum((y_data - y_pred) ** 2)
    ss_tot = np.sum((y_data - np.mean(y_data)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    # Plot Exponential Curve
    x_trend = np.linspace(x_data.min(), x_data.max(), 100)
    y_trend = a * np.exp(b * x_trend)
    trend_label = f'Exp Fit (y={a:.2f}*e^({b:.4f}x), R²={r_squared:.4f})'
    plt.plot(x_trend, y_trend, linestyle='--', color='black', linewidth=1.5, label=trend_label)

    # Formatting
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel(x_label, fontsize=12)
    plt.ylabel(y_label, fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()

def calculate_aabb_volume_xz(points, x_range, z_range):
    """
    Filters a pre-loaded numpy point array by an X-Z area and calculates 
    the AABB volume and point count. The Y-axis acts as the height.
    """
    # 1. Isolate the points within the defined X-Z plane
    # points[:, 0] is the X-axis
    # points[:, 2] is the Z-axis
    mask_x = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])
    mask_z = (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    
    filtered_points = points[mask_x & mask_z]
    
    if len(filtered_points) == 0:
        return 0.0, 0
        
    # 2. Find the absolute boundaries (This automatically finds the highest/lowest Y)
    min_bounds = np.min(filtered_points, axis=0)
    max_bounds = np.max(filtered_points, axis=0)
    
    # 3. Calculate volume (X_length * Y_height * Z_width)
    dimensions = max_bounds - min_bounds
    volume = dimensions[0] * dimensions[1] * dimensions[2]
    
    return volume, len(filtered_points)

def analyze_and_plot(df, x_col, y_col, title, x_label, y_label, output_path, sheet_name, color='tab:blue'):
    """Calculates both linear and exponential models, plots them, and returns the metrics for Excel export."""
    plt.figure(figsize=(10, 6))
    extracted_metrics = []

    # 1. Clean data for Linear Regression
    clean_df = df[[x_col, y_col]].dropna()
    clean_df = clean_df[np.isfinite(clean_df[x_col]) & np.isfinite(clean_df[y_col])]
    
    if len(clean_df) < 2:
        plt.close()
        return extracted_metrics

    x_data = clean_df[x_col].to_numpy()
    y_data = clean_df[y_col].to_numpy()

    # Plot Raw Data
    seaborn.lineplot(data=df, x=x_col, y=y_col, marker='o', linewidth=2, color=color, label='Raw Data')

    # ---------------------------------------------------------
    # 2. LINEAR REGRESSION (y = mx + b)
    # ---------------------------------------------------------
    slope, intercept = np.polyfit(x_data, y_data, 1)
    y_pred_lin = slope * x_data + intercept
    
    ss_res_lin = np.sum((y_data - y_pred_lin) ** 2)
    ss_tot_lin = np.sum((y_data - np.mean(y_data)) ** 2)
    r2_lin = 1 - (ss_res_lin / ss_tot_lin) if ss_tot_lin > 0 else 0

    x_trend = np.linspace(x_data.min(), x_data.max(), 100)
    plt.plot(x_trend, slope * x_trend + intercept, linestyle='--', color='black', linewidth=1.5, 
             label=f'Linear Fit (R²={r2_lin:.4f})')

    extracted_metrics.append({
        'Sheet_Source': sheet_name,
        'X_Axis': x_col,
        'Y_Axis': y_col,
        'Model_Type': 'Linear',
        'Equation': f"y = {slope:.6f}x + {intercept:.6f}",
        'R_Squared': round(r2_lin, 4),
        'Param_1_(Slope_or_Decay)': round(slope, 6),
        'Param_2_(Intercept_or_Amp)': round(intercept, 6)
    })

    # ---------------------------------------------------------
    # 3. EXPONENTIAL DECAY (y = a * e^(bx))
    # ---------------------------------------------------------
    # Exponential models mathematically fail if Y values are <= 0. Filter them strictly.
    exp_mask = y_data > 0
    if np.sum(exp_mask) >= 2:
        x_exp = x_data[exp_mask]
        y_exp = y_data[exp_mask]
        
        b, ln_a = np.polyfit(x_exp, np.log(y_exp), 1)
        a = np.exp(ln_a)
        
        y_pred_exp = a * np.exp(b * x_exp)
        ss_res_exp = np.sum((y_exp - y_pred_exp) ** 2)
        ss_tot_exp = np.sum((y_exp - np.mean(y_exp)) ** 2)
        r2_exp = 1 - (ss_res_exp / ss_tot_exp) if ss_tot_exp > 0 else 0

        plt.plot(x_trend, a * np.exp(b * x_trend), linestyle=':', color='tab:red', linewidth=2, 
                 label=f'Exp Fit (R²={r2_exp:.4f})')

        extracted_metrics.append({
            'Sheet_Source': sheet_name,
            'X_Axis': x_col,
            'Y_Axis': y_col,
            'Model_Type': 'Exponential',
            'Equation': f"y = {a:.6f} * e^({b:.6f}x)",
            'R_Squared': round(r2_exp, 4),
            'Param_1_(Slope_or_Decay)': round(b, 6),
            'Param_2_(Intercept_or_Amp)': round(a, 6)
        })

    # 4. Chart Output Formatting
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel(x_label, fontsize=12)
    plt.ylabel(y_label, fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()

    return extracted_metrics