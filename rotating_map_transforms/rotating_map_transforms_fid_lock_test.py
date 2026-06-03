"""
rotating_map_transforms_fid_lock_test.py

mac - python3 rotating_map_transforms_fid_lock_test.py <USERNAME> <PASSWORD> --map_dir "DIRECTORY" --mast_dir "MASTER_MAP_DIR" --dist <DISTANCE_IN_METERS> --start_n <START_N> --end_n <END_N>

windows - python rotating_map_transforms_fid_lock_test.py <USERNAME> <PASSWORD> --map_dir "DIRECTORY" --mast_dir "MASTER_MAP_DIR" --dist <DISTANCE_IN_METERS> --start_n <START_N> --end_n <END_N>
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
Date Created: 6/2/2026
Last Updated: 6/2/2026
"""

#general stuff
import argparse
import logging
import os
import struct #ply conversion
import sys
import time
import traceback
import math
from datetime import datetime
import unittest
from unittest.mock import MagicMock, patch
import numpy as np
import open3d as o3d
import csv
import json
import glob
import pandas  

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
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_se2_a_tform_b, BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b, VISION_FRAME_NAME
#from bosdyn.client.graph_nav_recording import GraphNavRecordingClient # Standalone in 5.x
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.world_object import WorldObjectClient

#google imports
import grpc

from google.protobuf import wrappers_pb2 as wrappers

ROBOT_IP ="192.168.80.3"
tag_id=1

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

#from leo_funcs import check_batt_perc, turn_relative, convert_map_to_ply, turn_relative, nav_to_fid, upload_map, fine_align, log_test_metrics,ensure_recording_stopped

import rotating_map_transforms_fid_lock

class TestRotatingMapLockMain(unittest.TestCase):

    # Update all target strings to match your file's exact name
    @patch('rotating_map_transforms_fid_lock.os.path.exists')
    @patch('rotating_map_transforms_fid_lock.os.makedirs')
    @patch('rotating_map_transforms_fid_lock.time.sleep') 
    @patch('rotating_map_transforms_fid_lock.time.time')  
    @patch('rotating_map_transforms_fid_lock.convert_map_to_ply_2')
    @patch('rotating_map_transforms_fid_lock.log_test_metrics')
    @patch('rotating_map_transforms_fid_lock.fine_align')
    @patch('rotating_map_transforms_fid_lock.nav_to_fid')
    @patch('rotating_map_transforms_fid_lock.upload_map')
    @patch('rotating_map_transforms_fid_lock.turn_relative')
    @patch('rotating_map_transforms_fid_lock.check_batt_perc')
    @patch('rotating_map_transforms_fid_lock.get_a_tform_b')
    @patch('rotating_map_transforms_fid_lock.bosdyn.client.create_standard_sdk')
    def test_main_successful_n1_execution(self, mock_create_sdk, mock_get_tform, 
                                          mock_check_batt, mock_turn, mock_upload, 
                                          mock_nav, mock_fine_align, mock_log, 
                                          mock_convert, mock_time, mock_sleep, 
                                          mock_makedirs, mock_exists):
        
        # 1. MOCK SETUP: Boston Dynamics SDK
        mock_sdk = MagicMock()
        mock_robot = MagicMock()
        mock_create_sdk.return_value = mock_sdk
        mock_sdk.create_robot.return_value = mock_robot
        
        mock_robot.is_powered_on.return_value = False

        # 2. MOCK SETUP: The State Client and Deep Battery Attributes
        mock_state_client = MagicMock()
        mock_robot_state = MagicMock()
        
        mock_battery = MagicMock()
        mock_battery.charge_percentage.value = 85.0
        mock_robot_state.battery_states = [mock_battery]
        
        mock_state_client.get_robot_state.return_value = mock_robot_state

        def client_side_effect(client_name):
            if client_name == 'robot-state':
                return mock_state_client
            return MagicMock()
        
        mock_robot.ensure_client.side_effect = client_side_effect

        # 3. MOCK SETUP: Custom Functions and Frame Math
        mock_fine_align.return_value = (0.02, 0.005) 
        mock_check_batt.return_value = True
        mock_time.return_value=100.0
        
        mock_tform = MagicMock()
        mock_tform.inverse().to_matrix.return_value = np.eye(4) 
        mock_get_tform.return_value = mock_tform

        mock_exists.return_value = False

        # 4. EXECUTION
        test_args = [
            'admin_user', 
            'admin_password', 
            '--map_dir', '/fake/output/dir', 
            '--mast_dir', '/fake/master/dir', 
            '--dist', '1.5', 
            '--start_n', '1', 
            '--end_n', '1' 
        ]
        
        # Execute the main function inside your named script
        rotating_map_transforms_fid_lock.main(test_args)

        # 5. ASSERTIONS
        mock_robot.authenticate.assert_called_once_with('admin_user', 'admin_password')
        mock_robot.power_on.assert_called_once()
        mock_upload.assert_called_once()
        mock_nav.assert_called_once()
        mock_fine_align.assert_called_once()
        mock_get_tform.assert_called_once()
        mock_makedirs.assert_any_call('/fake/output/dir')
        
        # Verify the converter received the correct N and the matrix parameter
        mock_convert.assert_called_once()
        args, kwargs = mock_convert.call_args
        self.assertEqual(args[2], 1) 
        np.testing.assert_array_equal(args[3], np.eye(4)) 

        mock_log.assert_called_once()

if __name__ == '__main__':
    unittest.main()