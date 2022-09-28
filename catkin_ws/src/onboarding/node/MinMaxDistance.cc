// Include the ros library
#include <ros/ros.h> 

#include <onboarding/PointDist.h>
// (todo) Include the LaserScan message header from the sensor_msgs library


class ScanDist
{
private:
    ros::NodeHandle n;
    ros::Subscriber scanSub;
    ros::Publisher minPub, maxPub;

    PointDist minPoint, maxPoint;

public:
    // Class constructor
    ScanDist()
    {
        n = ros::NodeHandle("~");

        // (todo) Initialize the scanSub and subscribe to 
        // the topic "/scan" with a buffer size of 1
        // using the callback scan_cb

        // (todo) Initialize the minPub publisher with 
        // the message type of onboarding::PointDist with a 
        // queue size of 1
        
        // (todo) Initialize the maxPub publisher with 
        // the message type of onboarding::PointDist with a 
        // queue size of 1
    }

    void scan_cb(const sensor_msgs::LaserScan &msg)
    {
        // (todo) Find the min value and it's angle
        // Save it to the pre-declared minPoint variable.


        // (todo) Find the max value and it's angle
        // Save it to the pre-declared maxPoint variable.

        // (todo) Publish those values to their respective
        // publishers

    }
};

int main(int argc, char **argv)
{
    // (todo) Initialized this file as a ros node
    
    ScanDist sd;
    ros::spin();

    return 0;
}   