// MAIN FUNCTION FOR MUX NODE
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
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

    std::string currState;
    bool isBrakeSet;

    // main drive topic publisher
    void muxIn_cb(const ackermann_msgs::AckermannDriveStamped &msg)
    {
        if(!isBrakeSet)
            muxOut.publish(msg);
        else
            brake();
    }

    void switch_cb(const std_msgs::UInt8 &msg)
    {
        // For now, need to manually add a case statement
        // for every new state added to the system.
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
                muxIn = n.subscribe(States::WallFollowing::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
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

    void setBrake(const std_msgs::Bool &msg)
    {
        isBrakeSet = msg.data;
    }

public:
    Mux() :
        isBrakeSet(true)
    {
        // ROS NODE
        n = ros::NodeHandle("~");

        // Subcribers
        muxController = n.subscribe("/input", 1, &Mux::switch_cb, this);
        muxIn = n.subscribe("", 1, &Mux::muxIn_cb, this);
        eBrake = n.subscribe("/brake_bool", 1, &Mux::setBrake, this);

        // Publishers
        muxOut = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc_cmd", 1);
    }

    void brake()
    {
        ackermann_msgs::AckermannDriveStamped drive;
        drive.header.stamp = ros::Time::now();
        drive.header.frame_id = States::Off::NAME;

        drive.drive.speed = 0.0;
        muxOut.publish(drive);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mux_controller");
    Mux m;
    ros::spin();
    return 0;
}