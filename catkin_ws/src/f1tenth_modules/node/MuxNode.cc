// MAIN FUNCTION FOR MUX NODE
#include <ros/ros.h>

// Message Headers
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <f1tenth_modules/States.hh>

class Mux
{
private:
    // ROS
    ros::NodeHandle n;

    ros::Publisher muxOut;
    ros::Subscriber muxIn;
    ros::Subscriber muxController;
    ros::Subscriber eBrake;

    std::string currState, masterDriveTopic;
    bool useSimulator;

    // main drive topic publisher
    void muxIn_cb(const ackermann_msgs::AckermannDriveStamped &msg)
    {
        muxOut.publish(msg);
    }

    void switch_cb(const std_msgs::UInt8 &msg)
    {
        // For now, need to manually add a case statement
        // for every new state added to the system.
        ROS_INFO("Current State: %s", currState.c_str());
        switch(msg.data)
        {
            case States::Off::INPUT_CHAR:
                if (currState == States::Off::NAME)
                    break;

                muxIn.shutdown();
                brake();
                ROS_INFO("Switching to state %s", States::Off::NAME.c_str());
                currState = States::Off::NAME;

                break;
            case States::Manual::INPUT_CHAR:
                if (currState == States::Manual::NAME)
                    break;

                muxIn.shutdown();
                muxIn = n.subscribe(States::Manual::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
                ROS_INFO("Switching to state %s", States::Manual::NAME.c_str());
                currState = States::Manual::NAME;

                break;
            case States::WallFollowing::INPUT_CHAR:
                if (currState == States::WallFollowing::NAME)
                    break;

                muxIn.shutdown();
                muxIn = n.subscribe(States::Autonmous::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
                ROS_INFO("Switching to state %s", States::WallFollowing::NAME.c_str());
                currState = States::WallFollowing::NAME;

                break;
            case States::GapFollowing::INPUT_CHAR:
                if (currState == States::GapFollowing::NAME)
                    break;

                muxIn.shutdown();
                muxIn = n.subscribe(States::GapFollowing::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
                ROS_INFO("Switching to state %s", States::GapFollowing::NAME.c_str());
                currState = States::GapFollowing::NAME;

                break;
            default:
                muxIn.shutdown();
                brake();
                ROS_WARN("Unkown state change : (%c)", static_cast<char>(msg.data));
                currState = "ERR";

                break;
        }
    }

    // This listens to the "brake_bool" topic that publishes a boolean value.
    // If we get a true value, we shutdown the muxIn subscriber and we
    //  publish a break message
    // If it's false, do not override the muxIn subscriber.
    void brake_cb(const std_msgs::Bool &msg)
    {
        if(msg.data)
        {
            muxIn.shutdown();
            brake();
        }
    }

public:
    Mux()
    {
        // ROS NODE
        n = ros::NodeHandle("~");

        // Subcribers
        muxController = n.subscribe("/input", 1, &Mux::switch_cb, this);
        muxIn = n.subscribe("", 1, &Mux::muxIn_cb, this);
        eBrake = n.subscribe("/brake_bool", 1, &Mux::brake_cb, this);

        n.param("useSimulator", useSimulator, false);

        if (useSimulator)
        {
            masterDriveTopic = "/drive";
            ROS_INFO("MUX: Using simulator drive topic /drive");
        }
        else
            masterDriveTopic = "vesc_cmd";

        // Publishers
        muxOut = n.advertise<ackermann_msgs::AckermannDriveStamped>(masterDriveTopic, 1);
    }

    void brake()
    {
        ackermann_msgs::AckermannDriveStamped drive;
        drive.header.stamp = ros::Time::now();
        drive.header.frame_id = States::Off::NAME;

        drive.drive.speed = 0.0;

        auto i = 0;
        while (i++<50)
        {
            muxOut.publish(drive);
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mux_controller");
    Mux m;
    ros::spin();
    return 0;
}