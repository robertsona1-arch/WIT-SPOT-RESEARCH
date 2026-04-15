"""
name_temp.py

mac - python3 name_temp.y <USERNAME> <PASSWORD> --dir "DIRECTORY" --int <NUM> --float <FLOAT>
windows - python name_temp.y <USERNAME> <PASSWORD> --dir "DIRECTORY" --int <NUM> --float <FLOAT>

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
from leo_funcs import check_batt_perc 

def main (argv):
    #parser takes info from terminal script call
    parser=argparse.ArgumentParser()

    parser.add_argument('username',help='Spot Username')
    parser.add_argument('password',help='Spot Password')
    parser.add_argument('--dir', help='Directory example')
    parser.add_argument('--int', type=int, help="integer example",default=0)
    parser.add_argument('--float', type=float, help="float example", default=0.0)

    options=parser.parse_args(argv)

    #create sdk, robot and clients
    print("\nAuthenticating...\n")
    sdk = bosdyn.client.create_standard_sdk('ScriptName') #create and authenticate sdk

    leo=sdk.create_robot(LEO_IP) #create robot object using the IP

    leo.authenticate(options.username,options.password)

    leo.time_sync.wait_for_sync() #syncs LEO's clock to computer

    #each client service will have its own lease client
    lease_client=leo.ensure_client('lease')
    recording_client = leo.ensure_client(GraphNavRecordingServiceClient.default_service_name)
    leo_state_client=leo.ensure_client('robot-state')
    
    print("\nAuthentication complete\n")

    if not leo.is_powered_on():
        print("\nLeo is powered off, please turn him onexiting...\n")
        sys.exit(1)
    

    #forcefully take the lease:
    lease_client.take()
    with LeaseKeepAlive(lease_client,must_acquire=False, return_at_exit=True):
        #main body of code goes here, the lease will automatically be released when the block is exited, even if an error occurs
        print("\nLease acquired, executing main body of code...\n")

        #example function call from leo_funcs, replace with your own function calls
        check_batt_perc(leo_state_client)
        time.sleep(2)

        #Command the robot to stand
        print("\nCommanding robot to stand...\n")
        stand=RobotCommandBuilder.synchro_stand_command()
        command_client.robot_command(stand)
        time.sleep(3)

        #rest of code goes here

if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)