import os
import argparse

if __name__ == '__main__':
    ## TODO Check if roscore is active ##

    ap = argparse.ArgumentParser()

    # Parameters
    ap.add_argument('-t', '--topics', nargs='+', type=str, required=True, help='List topics to record')
    ap.add_argument('-o', '--outfile', nargs=1, type=str, required=True, help='Output filename')

    args = ap.parse_args()
    
    # Check if bag output name exists
    while(f"{args.outfile[0]}.bag" in os.listdir()):
        args.outfile[0] = input("Filename already exists. Please change the name: ")

    input("Press enter to start recording...")
    
    # Create list of topics for command line
    topicList = ""
    
    for topics in args.topics:
        topicList += topics + " "
        
    print("Press CTRL+C to stop recording...")
        
    # Start recosbag recording
    os.system(f"rosbag record -O {args.outfile[0]}.bag {topicList}")