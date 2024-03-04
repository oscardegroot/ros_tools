#ifndef ros_tools_CONVERTIONS_H
#define ros_tools_CONVERTIONS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace RosTools
{
    geometry_msgs::Quaternion angleToQuaternion(double angle);

    double quaternionToAngle(const geometry_msgs::Pose &pose);
    double quaternionToAngle(geometry_msgs::Quaternion q);

}

#endif // ros_tools_CONVERTIONS_H
