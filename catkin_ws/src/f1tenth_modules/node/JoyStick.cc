#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>
#include <f1tenth_modules/JoyButtons.h>

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
    ros::Publisher buttonPub;

    ackermann_msgs::AckermannDriveStamped drive;
    std::string id {"joy_node"};
    std::string pubTopic {"/manual_drive"};
    std::string buttonTopic {"/joy_buttons"};
    std::string subTopic {"/joy"};

    float maxSpeed;
    float maxSteeringAngle;

    f1tenth_modules::JoyButtons buttons;

public:
    JoyStick() :
        n(ros::NodeHandle("~"))
    {
        // Pub-Sub initialization
        joyPub = n.advertise<ackermann_msgs::AckermannDriveStamped>(pubTopic, 1);
        buttonPub = n.advertise<f1tenth_modules::JoyButtons>(buttonTopic, 1);
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

        // Set button states
        buttons.a = false;
        buttons.b = false;
        buttons.x = false;
        buttons.y = false;

        buttonPub.publish(buttons);
    }

    void joy_callback(const sensor_msgs::Joy &msg)
    {
        // Publish any changes that we get from the buttons
        if (msg.buttons[0] != buttons.x || msg.buttons[1] != buttons.a
            || msg.buttons[2] != buttons.b || msg.buttons[3] != buttons.y)
        {
            buttons.x = static_cast<bool>(msg.buttons[0]);
            buttons.a = static_cast<bool>(msg.buttons[1]);
            buttons.b = static_cast<bool>(msg.buttons[2]);
            buttons.y = static_cast<bool>(msg.buttons[3]);

            buttonPub.publish(buttons);
        }

        // Speed
        auto x = msg.axes[1];
        // Steering
        auto y = msg.axes[2];

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