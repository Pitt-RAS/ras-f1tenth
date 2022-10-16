import os
import argparse
import rostopic

if __name__ == '__main__':
    ap = argparse.ArgumentParser()

    # Parameters
    ap.add_argument('-t', '--topics', nargs='+', type=str, required=True, help='List topics to record')
    ap.add_argument('-o', '--outfile', nargs=1, type=str, required=True, help='Output filename')

    args = ap.parse_args()
    
    # Check if bag output name exists
    while(f"{args.outfile[0]}.bag" in os.listdir()):
        args.outfile[0] = input("Filename already exists. Please change the name: ")
        
    # Check if roscore is running
    rosIsRunning = False

    while(not rosIsRunning):
        input("Press enter to start recording...")
        
        try:
            rostopic.get_topic_class('/rosout')
            rosIsRunning = True
        except rostopic.ROSTopicIOException as e:
            rosIsRunning = False
            print("Roscore needs to be active.")
    
    # Create list of topics for command line
    topicList = ""
    
    for topics in args.topics:
        topicList += topics + " "
        
    print("Press CTRL+C to stop recording...")
        
    # Start recosbag recording
    os.system(f"rosbag record -O {args.outfile[0]}.bag {topicList}")