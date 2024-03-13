#ifndef ros_tools_CONVERTIONS_H
#define ros_tools_CONVERTIONS_H

#ifdef MPC_PLANNER_ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace RosTools
{
    geometry_msgs::Quaternion angleToQuaternion(double angle);

    double quaternionToAngle(const geometry_msgs::Pose &pose);
    double quaternionToAngle(geometry_msgs::Quaternion q);

}

#else

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_listener.h>

namespace RosTools
{
    geometry_msgs::msg::Quaternion angleToQuaternion(double angle);

    double quaternionToAngle(const geometry_msgs::msg::Pose &pose);
    double quaternionToAngle(geometry_msgs::msg::Quaternion q);

}

#endif

#endif // ros_tools_CONVERTIONS_H
