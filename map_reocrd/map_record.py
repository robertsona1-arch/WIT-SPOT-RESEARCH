"""
map_record.py
Usage: python3 record_map.y <ROBOT_IP> --username <USER> --password <REDACTED>
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
Date Created: 1/5/2026
Last Updated: 1/5/2026
"""

import sys
import os
import argparse
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.graph_nav import GraphNavRecordingClient
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient #check this

def main(argv):
    parser=argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('--download-folder',name='./my_spot_map',help='Folder to save the map')
    options=parser.parse_args(argv)

    #Create SDK & Robot Object
    sdk = bosdyn.client.create_standard_sdk('MapRecorder')
    robot = bosdyn.client.util.create_robot_from_options(options,sdk)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    #Create Clients
    recording_client = robot.ensure_client(GraphNavRecordingClient.default_service_name)
    graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
    map_processing_client=robot.ensure_client(MapProcessingServiceClient.default_service_name)
    lease_client=robot.ensure_client('lease')

    map_processing_client=robot.ensure_client(MapProcessingServiceClient.default_service_name)
    
    if not os.path.exists(options.download_folder):
        os.makedirs(options.download_folder)

    #start recording, prompt user to start
    print("\nPrepare to begin recording.\nGrab tablet to drive\nStand Robot\nPress enter to begin\n")
    input()

    #clear left over map data
    graph_nav_client.clear_graph()

    #start recording
    recording_client.start_recording()
    print("\nRecording started, drive around\nPress enter to stop recording")
    input()

    #stop recording
    print("\nStopping recording")
    recording_client.stop_recording()

    print("\nProcessing map, be patient\n")
    print("\nProcessing topolgy\n")
    map_processing_client.process_topology(param=None) #closes loops and created edges
    print("\nProcessing anchoring\n")
    map_processing_client.process_anchoring(param=None) #final waypoint and map adjustments


    #download map
    print(f"Downloading map to {options.download_folder}...")

    #pull graph and snapshot to folder
    from bosdyn.client.graph_nav import download_graph_and_snapshots
    download_graph_and_snapshots(graph_nav_client, options.download_folder)

    print("Success! Map saved.")

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)