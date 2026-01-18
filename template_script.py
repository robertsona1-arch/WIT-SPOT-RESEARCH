"""
Boston Dynamics Spot - Manual Argument Parsing Fix
Usage:
  python3 template_script_fixed.py <ROBOT_IP> --username <USER> --password <PASS>
"""

import sys
import argparse  # We import this explicitly now
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.lease import LeaseKeepAlive

def main(argv):
    # --- FIX START ---
    # Instead of asking util to "get" a parser, we make one ourselves.
    parser = argparse.ArgumentParser()
    
    # We use the function that DOES exist in your list
    bosdyn.client.util.add_common_arguments(parser)
    
    # Parse the inputs
    options = parser.parse_args(argv)
    # --- FIX END ---

    # 2. SETUP SDK & ROBOT
    sdk = bosdyn.client.create_standard_sdk('SpotProjectClient')
    robot = bosdyn.client.util.create_robot_from_options(options, sdk)
    
    # 3. AUTHENTICATE
    bosdyn.client.util.authenticate(robot)
    
    # 4. START TIME SYNC
    print("Syncing time...")
    robot.time_sync.wait_for_sync()

    # 5. CHECK POWER
    if not robot.is_powered_on():
        print("Robot is not powered on. Exiting.")
        return

    # 6. ESTABLISH CLIENTS
    lease_client = robot.ensure_client('lease')
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    # 7. EXECUTE BEHAVIOR
    print(f"Connected to {robot.id.nickname} (Serial: {robot.id.serial_number})")
    
    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        print("Taking ownership and standing...")
        blocking_stand(command_client, timeout_sec=10)
        print("Success.")

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)