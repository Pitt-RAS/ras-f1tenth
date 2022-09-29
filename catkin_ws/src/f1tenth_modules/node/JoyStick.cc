#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>

/**
 * @brief Class to subscribe and convert joystick commands
 *          into an ackermann drive message
 */
class JoyStick
{
private:

    ros::NodeHandle n;
    ros::Subscriber joySub;
    ros::Publisher joyPub;

    ackermann_msgs::AckermannDriveStamped drive;
    std::string id {"joy_node"};
    std::string pubTopic {"/manual"};
    std::string subTopic {"/joy"};

    float maxSpeed;
    float maxSteeringAngle;

public:
    JoyStick() :
        n(ros::NodeHandle("~"))
    {
        // Pub-Sub initialization
        joyPub = n.advertise<ackermann_msgs::AckermannDriveStamped>(pubTopic, 1);
        joySub = n.subscribe(subTopic, 1, &JoyStick::joy_callback, this);

        // Creating an initial, full brake drive message
        drive.header.stamp  = ros::Time::now();
        drive.header.frame_id = id;

        drive.drive.steering_angle_velocity = 0.0;
        drive.drive.steering_angle = 0.0;
        drive.drive.speed = 0.0;

        // TODO: rosparam these
        maxSpeed = 7.0;
        maxSteeringAngle = 0.4189;
    }

    void joy_callback(const sensor_msgs::Joy &msg)
    {
        // Speed
        auto x = msg.axes[1];
        // Steering
        auto y = msg.axes[3];

        drive.header.frame_id = id;
        drive.header.stamp = ros::Time::now();

        this->drive.drive.speed = x*maxSpeed;
        this->drive.drive.steering_angle = y*maxSteeringAngle;
        joyPub.publish(drive);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_stick");
    JoyStick j;
    ros::spin();

    return 0;
}