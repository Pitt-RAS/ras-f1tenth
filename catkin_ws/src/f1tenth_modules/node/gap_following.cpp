/**
 * @file gap_following.cpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief This file follows lab 4 of the F1Tenth lab modules (https://f1tenth.org/learn.html)
 * @version 0.1
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Int32MultiArray.h>

#include <f1tenth_modules/f1tenthUtils.hpp>

class GapFollowing
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub;
        ros::Subscriber scanSub, muxSub;

        ackermann_msgs::AckermannDriveStamped drive;
        std::string driveTopic;

        lidarIntrinsics lidarData;

        int muxIdx;
        int startIdx, endIdx;
        double rb;
        bool enabled;

    public:

    GapFollowing():
        enabled(false),
        n(ros::NodeHandle("~"))
    {
        lidarData = getLidarInfoFromTopic(n, "/scan");
        if (!lidarData.valid)
            exit(-1);

        n.getParam("gap_follow_idx", muxIdx);
        n.getParam("gap_follow_topic", driveTopic);
        n.getParam("rb", rb);

        // pubs
        drivePub = n.advertise<ackermann_msgs::AckermannDriveStamped>(driveTopic, 1);

        // subs
        scanSub = n.subscribe("/scan", 1, &GapFollowing::scan_cb, this);
        muxSub = n.subscribe("mux", 1, &GapFollowing::mux_cb, this);

        startIdx = getScanIdx((-pi/2.0), lidarData);
        endIdx = getScanIdx((pi/2.0), lidarData);
    }

    void mux_cb(const std_msgs::Int32MultiArray &msg)
    {
        enabled = msg.data[muxIdx];
    }

    //
    // Do we want a copy or a reference of the LaserScan object?
    //
    void scan_cb(sensor_msgs::LaserScan &msg)
    {
        auto closestPoint = std::make_pair(-1, msg.range_max);

        // Limit scans from -pi/2 -> pi/2
        for(size_t i = startIdx; i <= endIdx; i++)
        {
            if (msg.ranges[i] < closestPoint.second)
            {
                closestPoint.first = i;
                closestPoint.second = msg.ranges[i];
            }
        }

        if(closestPoint.first < 0)
            return;

        // calculate bubble ranges
        auto theta = std::acos(rb/closestPoint.second);
        auto bubbleStartIdx = getScanIdx(closestPoint.first*lidarData.scan_inc + theta, lidarData);
        auto bubbleEndIdx = getScanIdx(closestPoint.first*lidarData.scan_inc - theta, lidarData);

    }
};


int main(int argc, char **argv)
{

}

