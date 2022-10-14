import os
import rosbag
import argparse

if __name__ == '__main__':
    ## TODO Check if roscore is active ##

    ap = argparse.ArgumentParser()

    # Parameters
    ap.add_argument('-t', '--topics', nargs='+', type=str, required=True, help='List topics to record')
    ap.add_argument('-o', '--outfile', nargs=1, type=str, default='nameMe', help='Output filename')

    args = ap.parse_args()

    input("Press enter to start")
    
    topicList = ""
    
    for topics in args.topics:
        topicList += topics
        
    print(topicList)
    exit()
    
    # os.system(f"rosbag record -O {args.outfile} {}")