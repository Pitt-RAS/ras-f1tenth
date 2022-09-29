#include <f1tenth_modules/RvizWrapper.hh>

/////////////////////////////////////////////////////////////////////
// RvizPoint

RvizPoint::RvizPoint(ros::NodeHandle &n, const rvizOpts &opts)
{
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;

    marker.header.frame_id = opts.frame_id;
    marker.ns = opts.ns;
    marker.pose = opts.pose;
    marker.scale = opts.scale;
    marker.id = id++;

    markerPub = n.advertise<visualization_msgs::Marker>(opts.topic, 10);
    changeColor(opts.color);
}

/////////////////////////////////////////////////////////////////////
// RvizLine

RvizLine::RvizLine(ros::NodeHandle &n, const rvizOpts &opts)
{
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;

    marker.header.frame_id = opts.frame_id;
    marker.ns = opts.ns;
    marker.pose = opts.pose;
    marker.scale = opts.scale;
    marker.id = id++;

    markerPub = n.advertise<visualization_msgs::Marker>(opts.topic, 10);
    changeColor(opts.color);
}

void RvizLine::addTranslation(const geometry_msgs::Point &v)
{
    geometry_msgs::Point origin, p;

    try
    {
        auto tf = tfBuffer->lookupTransform(transformPair.first, transformPair.second, ros::Time(0));
        origin.x = tf.transform.translation.x;
        origin.y = tf.transform.translation.y;
        origin.z = tf.transform.translation.z;
    } catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    p.x = origin.x + v.x;
    p.y = origin.y + v.y;
    p.z = origin.z + v.z;

    marker.points.clear();

    marker.points.push_back(origin);
    marker.points.push_back(p);

    markerPub.publish(marker);
}

/////////////////////////////////////////////////////////////////////
// RvizLineList
RvizLineList::RvizLineList(ros::NodeHandle &n, const rvizOpts &opts)
{
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.header.frame_id = opts.frame_id;
    marker.ns = opts.ns;
    marker.pose = opts.pose;
    marker.scale = opts.scale;
    marker.id = id++;

    markerPub = n.advertise<visualization_msgs::Marker>(opts.topic, 10);
    changeColor(opts.color);
}

void RvizLineList::addTranslation(const std::vector<geometry_msgs::Point> &v)
{
    if (v.empty())
        return;

    geometry_msgs::Point origin, p;
    marker.points.clear();

    try
    {
        auto tf = tfBuffer->lookupTransform(transformPair.first, transformPair.second, ros::Time(0));

        origin.x = tf.transform.translation.x;
        origin.y = tf.transform.translation.y;
        origin.z = tf.transform.translation.z;

        for (const auto &el : v)
        {
            p.x = origin.x + el.x;
            p.y = origin.y + el.y;
            p.z = origin.z + el.z;

            marker.points.push_back(origin);
            marker.points.push_back(p);
        }
    } catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    marker.header.stamp = ros::Time::now();
    markerPub.publish(marker);
}

 void RvizLineList::changeScale(const geometry_msgs::Vector3 &v)
 {
    marker.scale.x = v.x;
 }

/////////////////////////////////////////////////////////////////////
// RvizWrapper
int32_t RvizWrapper::id = 0;

RvizWrapper::RvizWrapper():
    tfBuffer(nullptr), tfListener(nullptr)
{
}

void RvizWrapper::changeColor(uint32_t color)
{
    auto b = (color & 0xff)/255.0;
    color = color>>8;
    auto g = (color & 0xff)/255.0;
    color = color>>8;
    auto r = (color & 0xff)/255.0;

    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.g = g;
}

void RvizWrapper::addTransformPair(const std::string &a, const std::string &b)
{
    if (tfBuffer == nullptr)
        tfBuffer = std::make_unique<tf2_ros::Buffer>();

    if (tfListener == nullptr)
        tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

    transformPair.first = a;
    transformPair.second = b;
}

void RvizWrapper::changeScale(const geometry_msgs::Vector3 &scale)
{
}

void RvizWrapper::addTranslation(const geometry_msgs::Point &v)
{

    geometry_msgs::Point p;

    try
    {
        auto tf = tfBuffer->lookupTransform(transformPair.first, transformPair.second, ros::Time(0));
        p.x = tf.transform.translation.x + v.x;
        p.y = tf.transform.translation.y + v.y;
        p.z = tf.transform.translation.y + v.z;
    } catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    //
    // TODO (nmm) figure out some sort of functionality to convert
    // from vector <-> point [geometry_msgs]
    //

    marker.points.clear();
    marker.points.push_back(p);
    marker.header.stamp = ros::Time::now();
    markerPub.publish(marker);
}

void RvizWrapper::addTranslation(const std::vector<geometry_msgs::Point> &v)
{
    try
    {
        geometry_msgs::Point point;
        auto tf = tfBuffer->lookupTransform(transformPair.first, transformPair.second, ros::Time(0));
        marker.points.clear();

        for(const auto &el : v)
        {
            point.x = tf.transform.translation.x + el.x;
            point.y = tf.transform.translation.y + el.y;
            point.z = tf.transform.translation.z + el.z;

            marker.points.push_back(point);
        }
    } catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    marker.header.stamp = ros::Time::now();
    markerPub.publish(marker);
}
