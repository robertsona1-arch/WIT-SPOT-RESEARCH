"""
mission_follower.py
Usage: python3 mission_follower.py <ROBOT_IP> --username <USER> --password <REDACTED> --map-dir ./my_lab_map
This script records a map, then downloads to a user specified folder
This script requires the user to walk the robot around the area to be mapped
THIS SCRIPT DOES NOT USE ESTOP, HAVE THE TABLET HANDY TO STOP THE ROBOT IF NEEDED
This script pulls significant portions of code from the Boston Dynamics Graph Nav Example & Recording Example
Minimal AI was used to aid in syntax and structure
"""


"""
Written by Adam Robertson, Wentworth Institute of Technology, School of Engineering
WIT SPOT Research Group
Prof. Latif 
Contributors: Patrick Woolf, Geoffery Siebert
Date Created: 1/6/2026
Last Updated: 1/6/2026
"""
import sys
import time
import argparse
import bosdyn.client
from bosdyn.client.map_processing import MapProcessingServiceClient #check this
from bosdyn.client.graph_nav import GraphNavRecordingClient
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand 
from bosdyn.client.robot_command import MobilityParams 

def main(argv):
    parser=argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('--map-dir',required=True,help='Folder w/ map to follow')
    options=parser.parse_args(argv)

    sdk=bosdyn.client.create_standard_sdk('MissionLooper')
    robot=bosdyn.client.util.create_robot_from_options(options,sdk)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    #client creation
    graph_nav_client=robot.ensure_client(GraphNavClient.default_service_name)
    map_processing_client=robot.ensure_client(MapProcessingServiceClient.default_service_name)
    lease_client=robot.ensure_client('lease')
    command_client=robot.ensure_client(RobotCommandClient.default_service_name)

    graph_nav_client.clear_graph()

    #upload map
    print(f"\nLoading map from folder {options.map_dir}...\n")
    from bosdyn.client.graph_nav import upload_graph_and_snapshots
    upload_graph_and_snapshots(graph_nav_client, options.map_dir)

    #initialize localization
    print("\nlocalizing start\n")
    graph = graph_nav_client.download_graph()
    stPT=graph.waypoints[0].id
    endPT=graph.waypoints[-1].id

    #force robot to think its at start
    graph_nav_client.set_localization(initial_guess_localization=None,ko_tform_body=None,max_distance=None,max_yaw=None,fiducial_init=None,refine_fiducial_result_with_icp=False,do_ambiguity_check=False,refine_with_visual_features=True,verify_visual_features=False)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        print("\nPower on...\n")
        robot.power_on(timeout_sec=5)
        blocking_stand(command_client, timeout_sec=5)
        time.sleep(5)

        total_laps=3

        for a in range(total_laps):
            if a==0:

            print(f"\nStarting lap {a+1}\n")

            #go to end pt
            print(f"\nmoving to end pt {endPT[:5]}\n")
            run_navigation(graph_nav_client,endPT)
            print("\nreached end\n")
            time.sleep(5)

            #return to start, just completes circle if circle, walks back if line
            print(f"\nreturning to start pt {stPT[:5]}\n")
            run_navigation(graph_nav_client,stPT)
            print("\n at start pt\n")

            time.sleep(5)
        print("\nMission Complete, sitting down\n")

def run_navigation(graph_nav_client, destination_id): 
    cmd_id=client.navigate_to(destination_id,1.6) #1.6 is max speed

    while True:
        feedback=client.navigation_feedback(cmd_id)

        if feedback.status==1: #reach goal
            return
        elif feedback.status==2: #status lost
            print("\n robot signal lost\n")
            sys.exit(1)
        elif feedback.status==3: #stuck
            print("\n robot stuck\n")
        else:
            print("\n other error\n")

        time.sleep(1)

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)