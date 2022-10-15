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

// Message Headers
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt8.h>

#include <f1tenth_modules/RvizWrapper.hh>
#include <f1tenth_modules/F1tenthUtils.hh>
#include <f1tenth_modules/States.hh>

class GapFollowing
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub;
        ros::Subscriber scanSub, muxSub;

        ackermann_msgs::AckermannDriveStamped drive;
        std::string driveTopic;

        lidarIntrinsics lidarData;
        pointScan closestPoint,
                  furthestPoint;

        double rb;
        double dispThreshold, dispBufferAngle;
        int muxIdx;
        int scanStartIdx, scanEndIdx;

        bool enabled;
        bool use_simulator;

        std::unique_ptr<RvizPoint> bubble;
        std::unique_ptr<RvizPoint> cp;
        std::unique_ptr<RvizLine> fp;
        std::unique_ptr<RvizLineList> bufferPoints;

    public:

    GapFollowing():
        enabled(false), use_simulator(false),
        n(ros::NodeHandle("~"))
    {
        lidarData = getLidarInfoFromTopic(n, "/scan");
        if (!lidarData.valid)
            exit(-1);

        n.getParam("gap_follow_idx", muxIdx);
        n.getParam("autonomous_drive_topic", driveTopic);
        n.getParam("rb", rb);
        n.getParam("disparity_threshold", dispThreshold);
        n.getParam("disparity_buffer_angle", dispBufferAngle);

        // pubs
        drivePub = n.advertise<ackermann_msgs::AckermannDriveStamped>(driveTopic, 1);

        // subs
        scanSub = n.subscribe("/scan", 1, &GapFollowing::scan_cb, this);

        if (use_simulator)
        {
            muxSub = n.subscribe("/mux", 1, &GapFollowing::mux_cb, this);
            ROS_INFO("(GAP FOLLOWING): not ussing simulator.");
        }
        else
        {
            muxSub = n.subscribe("/input", 1, &GapFollowing::key_input, this);
            ROS_INFO("(GAP FOLLOWING): not using simulator.");
        }

        scanStartIdx = getScanIdx((-M_PI/3.0), lidarData);
        scanEndIdx = getScanIdx((M_PI/3.0), lidarData);

        ROS_INFO("");
        ROS_INFO("Start index of scan : %d", scanStartIdx);
        ROS_INFO("End index of scan: %d", scanEndIdx);
        ROS_INFO("");

        // Rviz configuration
        geometry_msgs::Pose pose;
        geometry_msgs::Vector3 scale;

        pose.orientation.w = 1.0;
        scale.x = scale.y = 0.1;

        if (use_simulator)
        {
            rvizOpts opts = {
                            .color=0xff0000,
                            .frame_id="laser",
                            .ns="point",
                            .pose=pose,
                            .scale=scale,
                            .topic="/dynamic_viz"
                        };

            // These points will be green
            opts.color=0x00ff00;
            cp = std::make_unique<RvizPoint>(n, opts);
            bufferPoints = std::make_unique<RvizLineList>(n, opts);

            opts.color=0xff0000;
            fp = std::make_unique<RvizLine>(n, opts);
            bubble = std::make_unique<RvizPoint>(n, opts);

            cp->addTransformPair("base_link", "laser");
            fp->addTransformPair("base_link", "laser");
            bubble->addTransformPair("base_link", "laser");
            bufferPoints->addTransformPair("base_link", "laser");
        }
    }

    void mux_cb(const std_msgs::Int32MultiArray &msg)
    {
        enabled = msg.data[muxIdx];
    }

    void key_input(const std_msgs::UInt8 &msg)
    {
        if (msg.data == States::GapFollowing::INPUT_CHAR)
        {
            ROS_INFO("Gap Following enabled");
            enabled = true;
        }
        else
            enabled = false;
    }

    void scan_cb(const sensor_msgs::LaserScan &msg)
    {
        std::vector<geometry_msgs::Point> bubble_point_vector;
        std::pair<size_t, size_t> max_sequence_indices;
        std::vector<size_t> zeros_indices;

        std::vector<float> scan_cp;
        scan_cp.reserve(msg.ranges.size());

        scan_cp = msg.ranges;

        pointScan point_scan;
        geometry_msgs::Point point;

        point.z = 0.0;

        double r = rb;
        double max_sequence{0.0};
        auto min_point = std::make_pair(-1, msg.range_max);
        unsigned bubble_start_idx, bubble_end_idx;

        for (size_t i = scanStartIdx; i <= scanEndIdx; i++)
        {
            if (scan_cp[i] < min_point.second)
            {
                min_point.first = i;
                min_point.second = scan_cp[i];
            }
        }

        if (min_point.first < 0)
            return;

        closestPoint = {
                .dist = min_point.second,
                .angle = min_point.first*msg.angle_increment+msg.angle_min,
            };

        // Rviz
        if (use_simulator)
        {
            closestPoint.p.x = closestPoint.dist*std::cos(closestPoint.angle);
            closestPoint.p.y = closestPoint.dist*std::sin(closestPoint.angle);
            closestPoint.p.z = 0.0;
            cp->addTranslation(closestPoint.p);
        }

        // calculate start and end range of the bubble within the scan
        if (closestPoint.dist < rb)
        {
            bubble_start_idx = scanStartIdx;
            bubble_end_idx = scanEndIdx;
        } else
        {
            auto theta = std::asin(rb/closestPoint.dist);
            bubble_start_idx = getScanIdx(closestPoint.angle - theta, lidarData);
            bubble_end_idx = getScanIdx(closestPoint.angle + theta, lidarData);
        }


        // Check all points in the scan range of the bubble
        for (size_t i = bubble_start_idx; i <= bubble_end_idx; i++)
        {
            if (i < scanStartIdx || i > scanEndIdx)
                continue;

            zeros_indices.push_back(i);
        }

        //
        // Checking for the largest non-zero sequence
        //

        // from the start of the scan to the first indexed zero of the bubble
        if (zeros_indices.front()-1 - scanStartIdx > max_sequence)
        {
            max_sequence_indices.first = scanStartIdx;
            max_sequence_indices.second = zeros_indices.front()-1;
            max_sequence = zeros_indices.front() - scanStartIdx;
        }

        // This should be thought out. If the angular
        // sweep of the bubble is greater than 50%(?) that
        // of the scan then this code is necessary
        for (size_t i = 1; i < zeros_indices.size(); i++)
        {
            if (zeros_indices[i] < scanStartIdx)
                continue;
            else if (zeros_indices[i] >= scanEndIdx)
                break;

            if (zeros_indices[i]-zeros_indices[i-1] > max_sequence)
            {
                max_sequence_indices.first = zeros_indices[i-1];
                max_sequence_indices.second = zeros_indices[i];
                max_sequence = zeros_indices[i] - zeros_indices[i-1];
            }
        }

        // Max Sequence
        // last indexed zero to the end of the scan
        if (scanEndIdx-zeros_indices.back() > max_sequence)
        {
            max_sequence_indices.first = zeros_indices.back();
            max_sequence_indices.second = scanEndIdx;
            max_sequence = scanEndIdx-scanStartIdx-zeros_indices.back();
        }

        // Virtualize the points based on disparity
        std::vector<float> max_sequence_vector;
        max_sequence_vector.reserve(max_sequence_indices.second - max_sequence_indices.first + 1);

        // Make a subvector of points that correspond to the max
        // sequence on non-zero points.
        for (size_t i = max_sequence_indices.first; i <= max_sequence_indices.second; i++)
        {
            max_sequence_vector.push_back(scan_cp[i]);
        }

        // DISPARITIES
        geometry_msgs::Point p;
        p.z = 0.0;
        std::vector<geometry_msgs::Point> bp;

        for (size_t i = 1; i < max_sequence_vector.size(); i++)
        {
            auto min_point = std::min(max_sequence_vector[i], max_sequence_vector[i-1]);
            auto disparity = max_sequence_vector[i] - max_sequence_vector[i-1];

            if (std::fabs(disparity) >= dispThreshold)
            {
                if (disparity < 0)
                {
                    auto end_idx = (int)round((i*lidarData.scan_inc - dispBufferAngle)/lidarData.scan_inc);

                    if (end_idx < 0)
                        end_idx = 0;

                    for (size_t j = i; j > end_idx; j--)
                    {
                        max_sequence_vector[j] = min_point;
                        //Rviz
                        p.x = max_sequence_vector[j]*std::cos((j + max_sequence_indices.first)*lidarData.scan_inc + lidarData.min_angle);
                        p.y = max_sequence_vector[j]*std::sin((j + max_sequence_indices.first)*lidarData.scan_inc + lidarData.min_angle);
                        bp.push_back(p);
                    }
                }

                if (disparity > 0)
                {
                    auto end_idx = (int)round((i*lidarData.scan_inc + dispBufferAngle)/lidarData.scan_inc);

                    if (end_idx >= max_sequence_vector.size())
                        end_idx = max_sequence_vector.size()-1;

                    for (size_t j = i; j <= end_idx; j++)
                    {
                        max_sequence_vector[j] = min_point;
                        //Rviz
                        p.x = max_sequence_vector[j]*std::cos((j + max_sequence_indices.first)*lidarData.scan_inc + lidarData.min_angle);
                        p.y = max_sequence_vector[j]*std::sin((j + max_sequence_indices.first)*lidarData.scan_inc + lidarData.min_angle);
                        bp.push_back(p);
                    }
                    i = end_idx + 1;
                }
            }
        }

        if (use_simulator)
            bufferPoints->addTranslation(bp);
        //////////////

        // Find the largest point away from us within the max sequence
        auto max_point = std::make_pair(-1, msg.range_min);
        for (size_t i = 0; i < max_sequence_vector.size(); i++)
        {
            if (max_sequence_vector[i] > msg.range_max || std::isinf(max_sequence_vector[i]))
                continue;

            if (max_sequence_vector[i] >= max_point.second)
            {
                max_point.first = max_sequence_indices.first + i;
                max_point.second = max_sequence_vector[i];
            }
        }

        if (max_point.first < 0)
            return;

        furthestPoint = {
            .dist = max_point.second,
            .angle = max_point.first*msg.angle_increment + msg.angle_min,
        };

        // Rviz
        if (use_simulator)
        {
            furthestPoint.p.x = furthestPoint.dist*std::cos(furthestPoint.angle);
            furthestPoint.p.y = furthestPoint.dist*std::sin(furthestPoint.angle);
            fp->addTranslation(furthestPoint.p);
        }

        // Set the steering angle to the farthest point
        // (TODO) Set this up to be a helper function with custom structs
        drive.header.stamp = ros::Time::now();
        drive.header.frame_id = "drive_gap_following";
        drive.drive.steering_angle = max_point.first*msg.angle_increment + msg.angle_min;
        drive.drive.steering_angle_velocity = 0.0;
        drive.drive.speed = 1.5;

        if (enabled)
            drivePub.publish(drive);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gap_follow");
    GapFollowing g;

    ros::spin();
    return 0;
}

