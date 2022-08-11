/**
 * @file wall_follow.cpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief This file follows lab 3 of the F1Tenth lab modules (https://f1tenth.org/learn.html)
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <f1tenth_modules/f1tenthUtils.hpp>

/**
 * (todo)
 * - log P controller values
 * - need to test and tune PID controller
 */

class WallFollowing
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub, markerPub;
        ros::Subscriber scanSub, muxSub;

        tf2_ros::Buffer tBuffer;
        tf2_ros::TransformListener tfListener;
        visualization_msgs::Marker point;
        ackermann_msgs::AckermannDriveStamped drive;
        geometry_msgs::TransformStamped baseLinkTf;

        std::string driveTopic;
        pidGains gains;
        lidarIntrinsics lidarData;

        int muxIdx;
        int aIdx, bIdx;

        double sp, prevErr, err;
        double p,i,d;
        double L;
        double dt = 1/60.0;
        double theta = 70.0*pi/180.0; // [theta = 20 deg] (0 < theta < 70deg)

        bool enabled, done;

    public:
        WallFollowing() = delete;
        WallFollowing(double rate):
            dt(1/rate),
            enabled(false), done(false),
            prevErr(0.0),
            p(0.0), i(0.0), d(0.0),
            tfListener(tBuffer),
            n(ros::NodeHandle("~"))
        {
            // Extract  lidar info from one message
            lidarData = getLidarInfoFromTopic(n, "/scan");
            if (!lidarData.valid)
                exit(-1);

            n.getParam("wall_follow_idx", muxIdx);
            n.getParam("wall_follow_topic", driveTopic);
            n.getParam("kp", gains.kp);
            n.getParam("ki", gains.ki);
            n.getParam("kd", gains.kd);
            n.getParam("sp", sp);

            ROS_INFO("");
            ROS_INFO("\tkp: %f\tki: %f\tkd: %f", gains.kp, gains.ki, gains.kd);
            ROS_INFO("");

            // pubs
            drivePub = n.advertise<ackermann_msgs::AckermannDriveStamped>(driveTopic, 1);
            markerPub = n.advertise<visualization_msgs::Marker>("/dynamic_viz", 10);

            // subs
            scanSub = n.subscribe("/scan", 1, &WallFollowing::lidar_cb, this);
            muxSub = n.subscribe("/mux", 1, &WallFollowing::mux_cb, this);

            // We want this index the angle thats orthogonally
            // to the left of the front of the car _|
            // bIdx = (int)round((pi/2.0-lidarData.min_angle)/lidarData.scan_inc);
            // aIdx = (int)round((((pi/2.0)-theta)-lidarData.min_angle)/lidarData.scan_inc);
            bIdx = getScanIdx(pi/2.0, lidarData);
            aIdx = getScanIdx((pi/2.0)-theta, lidarData);
            ROS_INFO("Scanning data at angles %f - %f",
                lidarData.min_angle + (lidarData.scan_inc*aIdx),
                lidarData.min_angle + (lidarData.scan_inc*bIdx));

            // Update theta to be MORE accurate due to rounding errors in finding our idx
            theta = lidarData.scan_inc*(bIdx - aIdx);
            drive.drive.speed=0.0;

            //rviz visualization
            point.header.frame_id = "laser";
            // point.header.stamp = ros::Time::now();
            point.ns = "point";
            // point.action = visualization_msgs::Marker::ADD;
            point.pose.orientation.w = 1.0;
            point.id = 0;
            // point.type = visualization_msgs::Marker::POINTS;
            point.scale.x = point.scale.y = 0.2;
            point.color.g = 1.0f;
            point.color.a = 1.0;
        }


        void mux_cb(const std_msgs::Int32MultiArray &msg)
        {
            // Set the mux idx to verify wether to
            //  turn the PID controller on/off.
            enabled = msg.data[muxIdx];
            done = !enabled;

            if(enabled)
                ROS_INFO("PID node enabled.");
            else
                ROS_INFO("PID node disabled.");

            // (TODO) Maybe reset the PID values when "else"
        }

        void lidar_cb(const sensor_msgs::LaserScan &msg)
        {
            /////!!!!! NEED TO FILTER FOR BAD DISTANCES (inf &&&& <0)
            auto a = msg.ranges[aIdx];
            auto b = msg.ranges[bIdx];

            // Display extracted points on rviz
            try
            {
                baseLinkTf = tBuffer.lookupTransform("base_link", "laser_model", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }

            //
            // (TODO) This needs cleaned up especially since the point object is in the scope of the class
            //

            geometry_msgs::Point p;
            point.header.stamp = ros::Time::now();

            auto a_angle = aIdx*msg.angle_increment + msg.angle_min;
            p.x = baseLinkTf.transform.translation.x + msg.ranges[aIdx]*std::cos(a_angle);
            p.y = baseLinkTf.transform.translation.y + msg.ranges[aIdx]*std::sin(a_angle);
            p.z = 0.0;
            point.points.clear();
            point.points.push_back(p);
            markerPub.publish(point);

            auto alpha = std::atan((a*std::cos(theta)-b)/(a*std::sin(theta)));
            auto dist_1 = (b*std::cos(alpha)) + (drive.drive.speed*dt)*std::sin(alpha);

            err = sp - dist_1;

            if(enabled)
                pid_control(err);

            prevErr = err;
        }

        void pid_control(const double &err)
        {
            p = err;
            i += err; // may need to be clamped
            d = (err-prevErr)/dt;

            const auto steer_angle = -(gains.kp*p + gains.ki*i + gains.kd*d);
            const auto steer_ang_deg = steer_angle*(180.0/pi);
            const auto abs_steer_ang_deg = std::abs(steer_ang_deg);

            //
            // TODO: Change these limits to compare against radians to minimize conversions
            //
            if(abs_steer_ang_deg >= 0.0 && abs_steer_ang_deg<10.0)
                drive.drive.speed = 1.5;
            else if(abs_steer_ang_deg>=10.0 && abs_steer_ang_deg<=20.0)
                drive.drive.speed = 1.0;
            else
                drive.drive.speed = 0.5;

            drive.header.stamp = ros::Time::now();
            drive.header.frame_id = "drive";
            drive.drive.steering_angle = steer_angle;
            drive.drive.steering_angle_velocity = 0.0;

            drivePub.publish(drive);
        }

        ros::Rate getRate() const
        {
            return 1/dt;
        }

        bool isDone() const
        {
            return done;
        }

        void spin()
        {
            ros::spinOnce();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_follow");
    WallFollowing w(60.0);
    ros::Rate rate(w.getRate());

    while(!w.isDone())
    {
        w.spin();
        rate.sleep();
    }
}
