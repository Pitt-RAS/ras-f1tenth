/**
 * @file f1tenthUtils.hpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief Utility header file for the f1tenth_modules
 * @version 0.1
 * @date 2022-07-27
 *
 * @copyright Copyright (c) 2022
 */
#ifndef F1TENTH_UTILS_
#define F1TENTH_UTILS_

#include <cmath>

#define pi M_PI // lazily avoiding uppercase variables for science

struct pidGains
{
    double kp;
    double ki;
    double kd;
};

struct lidarIntrinsics
{
    int num_scans;
    double min_angle;
    double max_angle;
    double scan_inc;
    bool valid;
};

lidarIntrinsics getLidarInfoFromTopic(ros::NodeHandle &n ,const std::string &topic)
{
    boost::shared_ptr<const sensor_msgs::LaserScan>
                tmpScan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n, ros::Duration(10.0));
    lidarIntrinsics lidarData;

        if(tmpScan != NULL)
        {
            lidarData.scan_inc = tmpScan->angle_increment;
            lidarData.min_angle = tmpScan->angle_min;
            lidarData.max_angle = tmpScan->angle_max;
            lidarData.num_scans =
                (int)ceil((lidarData.max_angle - lidarData.min_angle)/lidarData.scan_inc);

            ROS_INFO("");
            ROS_INFO("Min Angle:\t%f", lidarData.min_angle);
            ROS_INFO("Max Andgle:\t%f", lidarData.max_angle);
            ROS_INFO("Scan Incr:\t%f", lidarData.scan_inc);
            ROS_INFO("Num scans:\t%d", lidarData.num_scans);
            ROS_INFO("");
            lidarData.valid = true;
        } else
        {
            ROS_INFO_ONCE("Couldn't extract lidar instrinsics... ");
            lidarData.valid = false;
            return lidarData;
        }
    return lidarData;
}

#endif