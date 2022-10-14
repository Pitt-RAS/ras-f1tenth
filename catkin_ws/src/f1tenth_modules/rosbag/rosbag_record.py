import os
import argparse
from time import sleep
import keyboard
from signal import signal, SIGINT

def handler(sig_received, frame):
    # Signal received, stop rosbag
    print("Rosbag record completed...exiting")
    sleep(1)
    exit(0)

if __name__ == '__main__':
    ## TODO Check if roscore is active ##
    
    signal(SIGINT, handler)

    ap = argparse.ArgumentParser()

    # Parameters
    ap.add_argument('-t', '--topics', nargs='+', type=str, required=True, help='List topics to record')
    ap.add_argument('-o', '--outfile', nargs=1, type=str, default='nameMe', help='Output filename')

    args = ap.parse_args()

    input("Press enter to start recording...")
    
    
    # Create list of topics for command line
    topicList = ""
    
    for topics in args.topics:
        topicList += topics + " "
    
    # Start recosbag recording
    os.system(f"rosbag record -O {args.outfile} {topics}")
    
    print("Press CTRL+C to stop recording...")
    
    while(True):
        pass