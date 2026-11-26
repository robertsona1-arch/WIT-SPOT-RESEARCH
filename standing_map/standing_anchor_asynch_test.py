
"""
standing_anchor_asynch.py

mac - python3 standing_anchor_asynch.py <USERNAME> <PASSWORD> --map_dir "DIRECTORY" --mast_dir "MAST_DIR" -dist 3.5 --end_n 10 --or_deg <0 or 180>

windows - python standing_anchor_asynch.py <USERNAME> <PASSWORD> --map_dir "DIRECTORY" --mast_dir "MAST_DIR" -dist 3.5 --end_n 10 --or_deg <0 or 180>

This script records maps with the robot standing. The amount of snapshots per map will increase by 2 starting from 1. 
The robot will navigate to the fiducial before starting recording. 
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
This script pulls significant portions of code from the Boston Dynamics recording_command_line.py & view_map.py 
Minimal AI was used to aid in syntax and structure

Used to test standing asynch map. 
"""

"""
Written by Adam Robertson, Wentworth Institude of Technology, Douglas D. Schumman School of Engineerning
WIT SPOT Research Group
Advisors: Prof. Tahmid Latif, Prof. Afsaneh Ghanavati
Contributors: Adam Robertson, Sean Derenas
Date Created: 11/25/2026
Last Updated: 11/25/2026
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
from unittest.mock import MagicMock
from requests import patch
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
from openpyxl.utils import get_column_letter
from pathlib import Path
from scipy.optimize import curve_fit
import warnings
from scipy.spatial import ConvexHull, QhullError

#bd specific imports
import google.protobuf.timestamp_pb2
import google.protobuf.duration_pb2
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
from bosdyn.client.image import ImageClient, save_images_as_files

#google imports
import grpc

from google.protobuf import wrappers_pb2 as wrappers
import unittest
from unittest.mock import MagicMock, patch



LEO_IP ="192.168.80.3"

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from leo_funcs import *

#can also import entire scripts
import standing_anchor_asynch

class TestRotatingMapLockMain(unittest.TestCase):

    # Update all target strings to match your file's exact name
    @patch('standing_anchor_asynch.os.path.exists')
    @patch('standing_anchor_asynch.os.makedirs')
    @patch('standing_anchor_asynch.time.sleep') 
    @patch('standing_anchor_asynch.time.time')  
    @patch('standing_anchor_asynch.bosdyn.client.create_standard_sdk')

    #grab specific functions you want to use, must be in script
    @patch('standing_anchor_asynch.ensure_recording_stopped')

    @patch('standing_anchor_asynch.upload_map_a')
    @patch('standing_anchor_asynch.log_test_metrics')
    @patch('standing_anchor_asynch.fine_align')
    @patch('standing_anchor_asynch.nav_to_fid')
    @patch('standing_anchor_asynch.turn_relative')
    @patch('standing_anchor_asynch.check_batt_perc')
    @patch('standing_anchor_asynch.convert_map_to_ply_3')

    def test_main_successful_n1_execution(self, mock_convert, 
                                          mock_check_batt, mock_turn, 
                                          mock_nav, mock_fine_align, mock_log, 
                                          mock_upload_map, mock_ensure_recording, mock_create_sdk, mock_time, mock_sleep, 
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

        mock_exists.return_value = False

        # 4. EXECUTION
        test_args = [
            'admin_user', 
            'admin_password', 
            '--map_dir', '/fake/output/dir', 
            '--mast_dir', '/fake/master/dir', 
            '--dist', '1.5', 
            '--end_n', '1', 
            '--or_deg', '1' 
        ]
        
        # Execute the main function inside your named script
        standing_anchor_asynch.main(test_args)

        # 5. ASSERTIONS
        mock_robot.authenticate.assert_called_once_with('admin_user', 'admin_password')
        mock_robot.power_on.assert_called_once()
        mock_upload_map.assert_called_once()
        mock_nav.assert_called_once()
        mock_fine_align.assert_called_once()
        mock_makedirs.assert_any_call('/fake/output/dir')
        
        # Verify the converter received the correct N and the matrix parameter
        mock_convert.assert_called_once()
        args, kwargs = mock_convert.call_args
        self.assertEqual(args[2], 1) 
        np.testing.assert_array_equal(args[3], np.eye(4)) 

        mock_log.assert_called_once()

if __name__ == '__main__':
    unittest.main()