map record readme

This script will record a map, automatically create the default waypoint, create edges, and fix errors.
It will save the map to a folder of your choosing. 
The user is required to drive the robot in the desired path. 
Please finish close to the start point. 

running:
    mac/linux:
        absolute path: python3 record_map.py 192.168.80.3 --username user --password password --download-folder /path/ 
        relative path: python3 record_map.py 192.168.80.3 --username user --password password --download-folder 
    windows:
        aboslute path: python record_map.py 192.168.80.3 --username user --password password --download-folder C:\path\ 
        relative path: python record_map.py 192.168.80.3 --username user --password password --download-folder 
Have a folder set before hand, if you don't it will default, if you default it twice then it will delete work
To find the folder path, find the folder, double click, copy file path

Notes: 
    argv: 
        when python3 record_map.py 192.168.80.3 --username user
        python sees ['record_map.py','192.168.80.3','--username','user']
        main(argv[1:]) passes in everything from that except the 0 index

    parser:
        parser is an object from argparse library that knows what to expect from inputs and defines rules for inputs
        parser section add rules for IP, user and password
        user can optionally give a download directory

    options=parser.parse_args(argv) matches argv against parser
    download-folder becomes options.download_folder
    can access using options.hostname or username etc