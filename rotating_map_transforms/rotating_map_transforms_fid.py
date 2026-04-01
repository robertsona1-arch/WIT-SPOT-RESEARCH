"""
rotating_map_transforms_fid.py

python3 rotating_map_transforms_fid.py <USERNAME> <PASSWORD> <DIRECTORY> <MASTER_MAP_DIR> <DISTANCE_IN_METERS> <START_N> --end_n <END_N>
use dist=3.5

Use ctrl+c to stop the script at any time, the robot will stop and sit safely, and any completed maps will be saved

This script records a map with the robot making N turns
It will begin with <START_N> turns, then increments by factors of 360 until it reaches the battery check
After each set of rotations is complete, the robot will find the fiducial again, creating a reference point
This script is an updated version of rotating_map that will track the rotation of the snapshots and use the transformation matrix
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
This script pulls significant portions of code from the Boston Dynamics recording_command_line.py & view_map.py 
Minimal AI was used to aid in syntax and structure
"""


"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert
Date Created: 3/24/2026
Last Updated: 3/31/2026
"""

from bosdyn.client.math_helpers import SE2Pose #new
from bosdyn.api import world_object_pb2, basic_command_pb2#new
from bosdyn.client.world_object import WorldObjectClient

import argparse
import logging
import os
import sys
import time
import struct #added for ply conversion
import traceback
import math

import google.protobuf.timestamp_pb2
#import graph_nav_util
import grpc
from google.protobuf import wrappers_pb2 as wrappers

import bosdyn.client.channel 
import bosdyn.client.util
from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2
import bosdyn.client.graph_nav 
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient #check this
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2
from bosdyn.api.gps import gps_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2

#following not in sdk examples
import bosdyn.client
from bosdyn.client import map_processing
from bosdyn.client.robot import Robot

from bosdyn.client.lease import LeaseKeepAlive
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
# 1. CLIENTS (The "Doing" part)

#from bosdyn.client.graph_nav_recording import GraphNavRecordingClient # Standalone in 5.x
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.api import geometry_pb2
from bosdyn.client.map_processing import MapProcessingServiceClient

# 2. APIS/PROTOS (The "Data" part)
from bosdyn.api import robot_command_pb2 as generic_robot_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

from bosdyn.client import math_helpers
from bosdyn.api import geometry_pb2, trajectory_pb2 

ROBOT_IP ="192.168.80.3"
tag_id=1


def main(argv):
    #1. setup positional arguments
    parser=argparse.ArgumentParser()

    #positional args
    parser.add_argument('username',help='Spot Username')
    parser.add_argument('password',help='Spot Password')
    parser.add_argument('map_dir',help='Directory to save maps to')
    parser.add_argument('mast_dir',help='Master Map Directory')
    parser.add_argument('dist',type=float,help='distance in meters to fiducial')
    parser.add_argument('start_n',type=int,help='Number of initial rotations to perform')

    #optional end N
    parser.add_argument('--end_n',type=int,help='Number of maximum rotations to perform',default=8)

    options=parser.parse_args(argv)
    if options.start_n<1:
        options.start_n=1

    if options.end_n<options.start_n:
        options.end_n=options.start_n-1
    

    #2. create sdk & authenticate
    sdk = bosdyn.client.create_standard_sdk('RotatingMapExample')

    #create robot object since
    robot=sdk.create_robot(ROBOT_IP)
    robot.authenticate(options.username,options.password)

    print("Authenticating...")
    robot.time_sync.wait_for_sync()

    #3. create clients
    lease_client=robot.ensure_client('lease')
    recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
    graph_nav_client=robot.ensure_client(GraphNavClient.default_service_name)
    command_client=robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state_client=robot.ensure_client('robot-state')
    map_processing_client = robot.ensure_client(MapProcessingServiceClient.default_service_name)

    world_object_client=robot.ensure_client(bosdyn.client.world_object.WorldObjectClient.default_service_name)

    #create directory
    if not os.path.exists(options.map_dir):
        os.makedirs(options.map_dir)

    #forcefully take the lease:
    lease_client.take()
    with LeaseKeepAlive(lease_client, must_acquire=False, return_at_exit=True):
        print("\nbeginning\n")
        #clear estops and power on motors locally
        robot.time_sync.wait_for_sync()
        if not robot.is_powered_on():
            print("\nPowering on leo\n")
            robot.power_on(timeout_sec=20)

        #Command the robot to stand
        print("\nCommanding robot to stand...\n")
        stand=RobotCommandBuilder.synchro_stand_command()
        command_client.robot_command(stand)
        time.sleep(2)

        # Upload the map to the robot
        upload_map(graph_nav_client, options.mast_dir)
        print(f"\nMaster map uploaded\n")
        nav_to_fid(robot,tag_id, dist=options.dist)
        fine_align(robot, tag_id, options.dist, iter=100)
        print(f"\nNavigating to fiducial\n")

        for a in range(options.start_n, options.end_n+1):
            start_time = time.time()
            #battery check, won't run if less than 20%
            if not check_batt_perc(robot_state_client,limit=20.0):
                print(f"\nBattery below 20%. Stopping at N={a}.")
                break
            nav_to_fid(robot,tag_id, dist=options.dist)
            fine_align(robot, tag_id, options.dist, iter=100)
            if 360 % a ==0:
                degPT=360.0/a
                fold_name=f"test_n_{a:02d}"
                full_path=os.path.join(options.map_dir,fold_name)

                print(f"\nStarting mapping with N={a} rotations, {degPT:.2f} degrees per rotation\n")

                if not os.path.exists(full_path):
                    os.makedirs(full_path)
                
                #graph_nav_client.clear_graph() got error saying call stop recording first
                recording_client.start_recording()
                print("\nStarting Recording\n")
                time.sleep(0.1)

                for b in range(a):
                    print(f"\n[N={a} Step{b+1}/{a}] Rotating {degPT:.2f} degrees\n")

                    #snapshot
                    recording_client.create_waypoint(waypoint_name=f"N{a}_Snap{b+1}")
                    #time.sleep(2)#need to have this so it goes on when its ready, I remember needing this but don't have it written down anywhere, test if its actually needed
                    print("\nCreating Waypoint\n")
                    #turn
                    turn_relative(command_client,robot_state_client,degPT)
                    time.sleep(3)

                #stop and download
                recording_client.stop_recording()
                time.sleep(0.5)
                
                # Use the module-level helper, passing the directory and the client
                graph_nav_client.write_graph_and_snapshots(full_path)

                #convert
                print(f"\n[N{a}]Converting to ply...\n")
                ply_name=os.path.join(full_path,f"converted_n_{a}.ply")
                convert_map_to_ply(full_path,ply_name,a)
                graph_nav_client.clear_graph()
                end_time = time.time()
                elapsed_time = end_time - start_time
                print(f"\n[N={a}] Map with {a} rotations completed in {elapsed_time/60:.2f} minutes\n")
            else:
                print(f"\nN={a} is not a factor of 360, skipping to next N\n")
                continue

        command_client.robot_command(RobotCommandBuilder.synchro_sit_command())
                  
#Functions
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

def convert_map_to_ply(map_dir, output_file,n): 
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

    # 4. Push 1.5m straight out from the tag (No rotation yet)
    tag_z_out = SE3Pose(x=0.0, y=0.0, z=dist_m, rot=Quat())
    raw_seed_goal = seed_tform_body * tag_z_out

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
            return False

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
        
        # 5. The Threshold Check (e.g., within 1.5cm and 1.5 degrees)
        if dist_error < dist_thrsh and math.degrees(yaw_error) < deg_thrsh:
            print("SUCCESS: Experimental alignment tolerances achieved.")
            return True

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
    return True

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

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)