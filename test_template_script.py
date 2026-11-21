#template test script
"""
The use case of these scripts is for simple debugging PRIOR to lab time. This script does not contain any real world simulation.
"""


"""
name_temp_test.py

mac - python3 name_temp_test.y <USERNAME> <PASSWORD> --dir "DIRECTORY" --int <NUM> --float <FLOAT>

windows - python name_temp_test.y <USERNAME> <PASSWORD> --dir "DIRECTORY" --int <NUM> --float <FLOAT>

DESC...

Use ctrl+c to stop the script any time, the robot will stop and sit safetly, and any completed maps will be saved
Please maintian a safe distance from the robot at all time
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET READY TO STOP THE ROBOT IF NEEDED

This script pulls significant portions of code from the Boston Dynamics <blank> files and Google API <blank> files

Minimail AI was used in syntax and structure
"""

"""
Written by <Your name here>, Wentworth Institude of Technology, Douglas D. Schumman School of Engineerning
WIT SPOT Research Group
Advisors: Prof. Tahmid Latif, Prof. Afsaneh Ghanavati
Contributors: Adam Robertson, <other name>, <other name 2>
Date Created: XX/XX/XXXX
Last Updated: XX/XX/XXXX
"""

#Copy ENTIRE list of imports from LIST OF LIBRARIES, !!DELETE ME!!
#if you have imported everything correctly, all the warnings will resolve

#imports

LEO_IP ="192.168.80.3"

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

#import all neccesary functions from leo_funcs !!DELETE ME!!
from requests import patch

from leo_funcs import *

#can also import entire scripts
import template_script

class TestRotatingMapLockMain(unittest.TestCase):

    # Update all target strings to match your file's exact name
    @patch('template_script.os.path.exists')
    @patch('template_script.os.makedirs')
    @patch('template_script.time.sleep') 
    @patch('template_script.time.time')  
    @patch('template_script.bosdyn.client.create_standard_sdk')

    #grab specific functions you want to use, must be in script
    @patch('template_script.convert_map_to_ply_2')
    @patch('template_script.log_test_metrics')
    @patch('template_script.fine_align')
    @patch('template_script.nav_to_fid')
    @patch('template_script.upload_map')
    @patch('template_script.turn_relative')
    @patch('template_script.check_batt_perc')
    @patch('template_script.get_a_tform_b')

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
        template_script.main(test_args)

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