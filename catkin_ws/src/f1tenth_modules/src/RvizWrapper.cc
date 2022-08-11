#pragma once

#include <f1tenth_modules/RvizWrapper.hh>
#include <f1tenth_modules/f1tenthUtils.hh>


RvizPoint::RvizPoint(const rvizOpts &opts):
    tfBuffer(nullptr), tfListener(nullptr)
{
    marker.action = visualization_msgs::Marker::POINTS;

    marker.header.frame_id = opts.frame_id;
    marker.ns = opts.ns;
    marker.pose = opts.pose;
    marker.scale = opts.scale;
    marker.id = id++;

    changeColor(opts.color);
}

void RvizPoint::changeColor(const uint32_t color)
{

}

void RvizPoint::addTransformPair(const std::string &a, const std::string &b)
{
    if (tfBuffer == nullptr && tfListener == nullptr)
    {
        tfBuffer = std::make_unique<tf2_ros::Buffer>();
        tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
    }

    transformPair.first = a;
    transformPair.second = b;
}


