#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#include <cmath>

#include <f1tenth_modules/F1tenthUtils.hh>

#define PI M_PI

//
// (TODO) [nmm] Add a library that has predifined macros and structs
//
struct lidar_intrinsics
{
    double scan_inc,
           min_angle,
           max_angle;
    int num_scans;
};

std::vector<double> compute_car_perim(
    const carIntrinsics& car_data, const lidar_intrinsics& lidar_data)
{
    std::vector<double> car_perim = std::vector<double>();
    car_perim.reserve(lidar_data.num_scans);

    auto angle = lidar_data.min_angle;
    for( size_t i = 0; i < lidar_data.num_scans; i++ )
    {
        if(angle > 0.0) // left side of the car
        {
            if(angle < PI/2.0) // 0 -> pi/2
            {
                auto left_side = (car_data.width/2.0)/std::sin(angle);
                auto top_left = (car_data.wheelbase - car_data.base_link)/std::cos(angle);
                car_perim.push_back(std::min(left_side, top_left));
            }
            else // pi/2 -> pi
            {
                auto left_side = (car_data.width/2.0)/std::cos(angle - (PI/2.0));
                auto bottom_left = (car_data.base_link)/std::sin(angle - (PI/2.0));
                car_perim.push_back(std::min(left_side, bottom_left));
            }
        }
        else // right side of the car
        {
            if(angle < -PI/2.0) // pi -> 3pi/2
            {
                auto right_side = (car_data.width/2.0)/std::cos(-angle - (PI/2.0));
                auto bottom_right = (car_data.base_link)/std::sin(-angle - (PI/2.0));
                car_perim.push_back(std::min(right_side, bottom_right));
            }
            else // 3pi/2 -> 2pi
            {
                auto right_side = (car_data.width/2.0)/std::sin(-angle);
                auto top_right = (car_data.wheelbase - car_data.base_link)/std::cos(-angle);
                car_perim.push_back(std::min(top_right, right_side));
            }
        }
        angle += lidar_data.scan_inc;
    }
    return car_perim;
}

// The class that handles emergency braking
class Safety {
private:
    ros::NodeHandle n;

    ros::Subscriber scan_sub, odom_sub;
    ros::Publisher brake_pub, speed_pub;

    // Info to perform emergency braking
    std::vector<double> car_perimeter;
    lidar_intrinsics lidar;
    carIntrinsics car;
    double ttc_threshold = 0.2;
    double speed;

    // Data to publish
    struct {
        std_msgs::Bool brake;
        ackermann_msgs::AckermannDriveStamped speed;
    } brake_msg;

public:
    Safety() :
        n(ros::NodeHandle("~"))
    {
        ROS_INFO("Initializing emergency brake configs.");
        speed = 0.0;

        // Initialize brake message
        brake_msg.brake.data = false;
        brake_msg.speed.drive.speed = 0.0;

        // Listening to one scan message to grab LIDAR instrinsics
        boost::shared_ptr<const sensor_msgs::LaserScan>
            shared = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n, ros::Duration(10));

        if (shared != NULL)
        {
            lidar.scan_inc = shared->angle_increment;
            lidar.max_angle = shared->angle_max;
            lidar.min_angle = shared->angle_min;

            //
            // TODO(nmm) make these extrinsics automated and organize
            //
            n.getParam("scan_beams", lidar.num_scans);

            ROS_INFO("");
            ROS_INFO("Min Angle:\t%f", lidar.min_angle);
            ROS_INFO("Max Andgle:\t%f", lidar.max_angle);
            ROS_INFO("Scan Incr:\t%f", lidar.scan_inc);
            ROS_INFO("Num scans:\t%d", lidar.num_scans);
            ROS_INFO("");
        }

        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        //  Pubs 
        brake_pub = n.advertise<std_msgs::Bool>("/brake_bool", 1);
        speed_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);

        //  Subs 
        scan_sub = n.subscribe("/scan", 1, &Safety::scan_callback, this);
        odom_sub = n.subscribe("/odom", 1, &Safety::odom_callback, this);

        // Extract parameters
        n.getParam("width", car.width);
        n.getParam("scan_distance_to_base_link", car.base_link);
        n.getParam("wheelbase", car.wheelbase);
        n.getParam("scan_beams", lidar.num_scans);

        // Compute the perimeter of the car
        car_perimeter = compute_car_perim(car, lidar);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        speed = odom_msg->twist.twist.linear.x; // Update current speed.
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        if( speed != 0)
        {
            // If the array sizes don't match then we won't continue with the scan
            if(scan_msg->ranges.size() != car_perimeter.size())
            {
                ROS_INFO_ONCE("Scan size does match precomputed size(%d != %d)",
                    scan_msg->ranges.size(), car_perimeter.size());
                return;
            }

            // Calculating TTC for each scan increment
            for( size_t i = 0 ; i < scan_msg->ranges.size(); i++ )
            {
                auto r_hat = speed*std::cos(i*scan_msg->angle_increment + scan_msg->angle_min);
                auto ttc = (scan_msg->ranges[i] - car_perimeter[i])/r_hat;

                if( ttc < ttc_threshold  && (ttc>=0.0))
                {
                    brake_msg.brake.data = true;
                    brake_pub.publish(brake_msg.brake);
                    
                    // TODO(NMM) : change this to a service 
                    auto i = 0;    
                    while (i++ < 50)
                    {
                        speed_pub.publish(brake_msg.speed);
                    }
            
                    ROS_INFO("E-BRAKE:\t(angle)%f", scan_msg->angle_min +i*scan_msg->angle_increment);
                }
            }
        }
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}