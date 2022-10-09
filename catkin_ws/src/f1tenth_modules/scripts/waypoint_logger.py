import rospy
import numpy as np ### for logging csv
import atexit

from os.path import expanduser
from time import gmtime, strftime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

home = expanduser('~')
file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

def save_waypoint():
    # Write way points to a file
    return

def listener():
    rospy.init_node("waypoint_logger", anonymous=True)
    rospy.Subscriber('/gt_pose', Point, save_waypoint)
    rospy.spin()

def shutdown():
    file.close()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('logging waypoints...')

    try:
        listener()
    except rospy.ROSInterruptException:
        pass