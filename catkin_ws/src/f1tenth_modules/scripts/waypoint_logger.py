#!/usr/bin/env python3
import rospy
import os
import math
import sys
from geometry_msgs.msg import PoseStamped
from f1tenth_modules.msg import JoyButtons
from time import gmtime, strftime

class WaypointLogger:
    def __init__(self):
        self.waypoints = []
        self.threshold = 0.05
        self.enabled = False

        rospy.Subscriber('/gt_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/joy_buttons', JoyButtons, self.button_cb)

    def writeFile(self):
        print(os.getcwd())
        dir = os.getcwd()
        file = open(strftime(dir + '/log/latest/wp-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')
        for point in self.waypoints:
            file.write('%f, %f\n' %(point[0], point[1]))
        file.close()

    def filterPoints(self):
        ctr = 0
        for i, p in enumerate(self.waypoints):
            if i != len(self.waypoints)-1:
                for j, q in enumerate(self.waypoints[i+1:]):
                    dist = math.sqrt(math.pow(q[0] - p[0], 2) + math.pow(q[1] - p[1], 2))
                    if dist < self.threshold:
                        self.waypoints.pop(j)
                        ctr+=1

        rospy.logdebug(f"(WAYPOINT LOGGER) Filtered out {ctr} points")


    def pose_cb(self, msg):

        if self.enabled:
            x = msg.pose.position.x
            y = msg.pose.position.y

            if len(self.waypoints) == 0:
                self.waypoints.append((x, y))
                return

            dist = math.sqrt(math.pow(x - self.waypoints[-1][0], 2)
                                        + math.pow(y - self.waypoints[-1][1], 2))
            if dist > self.threshold:
                self.waypoints.append((x, y))

    def button_cb(self, msg):
        ## Toggle enabled
        if msg.a:
            self.enabled = not self.enabled
            if self.enabled:
                rospy.loginfo("(WAYPOINT LOGGER) recording waypoints")
            else:
                rospy.loginfo("(WAYPOINT LOGGER) recording pause")

        if msg.b and not self.enabled:
            rospy.loginfo("(WAYPOINT LOGGER) writing waypoints to disk")
            self.filterPoints()
            self.writeFile()
            self.waypoints.clear()

        if msg.y:
            rospy.logdebug(f"(WAYPOINT LOGGER) length of list {len(self.waypoints)}")

def main(args):
    logger = WaypointLogger()
    rospy.init_node('waypoint_logger', anonymous=True, log_level=rospy.DEBUG)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)