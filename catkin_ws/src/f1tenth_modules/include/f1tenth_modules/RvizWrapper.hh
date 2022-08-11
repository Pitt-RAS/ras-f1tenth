#ifndef RVIZ_WRAPPER_HH
#define RVIZ_WRAPPER_HH

#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <f1tenth_modules/f1tenthUtils.hh>

class RvizWrapper
{
protected:
    visualization_msgs::Marker marker;
public:
    static int32_t id;
    RvizWrapper::RvizWrapper();
    virtual void RvizWrapper::changeColor(const uint32_t &);
    virtual void RvizWrapper::changeScale(const geometry_msgs::Vector3 &);
    virtual void RvizWrapper::addTransformPair(const std::string &, const std::string &);
};

int32_t RvizWrapper::id = 0;

class RvizPoint : public RvizWrapper
{
private:
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::pair<std::string, std::string> transformPair;
public:
    RvizPoint::RvizPoint() = delete;
    RvizPoint::RvizPoint(const rvizOpts &);

    // These should just stay defined in the base class?
    void RvizPoint::changeColor(const uint32_t);
    void RvizPoint::addTransformPair(const std::string &, const std::string &);
    void RvizPoint::addTranslation(const geometry_msgs::Vector3 &);
};


#endif