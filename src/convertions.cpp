#include "ros_tools/convertions.h"

#include <tf/transform_listener.h>

namespace RosTools
{
    geometry_msgs::Quaternion angleToQuaternion(double angle)
    {
        tf::Quaternion q = tf::createQuaternionFromRPY(0., 0., angle);
        geometry_msgs::Quaternion result;
        result.x = q.getX();
        result.y = q.getY();
        result.z = q.getZ();
        result.w = q.getW();

        return result;
    }

    double quaternionToAngle(const geometry_msgs::Pose &pose)
    {
        double ysqr = pose.orientation.y * pose.orientation.y;
        double t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
        double t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);

        return atan2(t3, t4);
    }

    double quaternionToAngle(geometry_msgs::Quaternion q)
    {
        double ysqr, t3, t4;

        // Convert from quaternion to RPY
        ysqr = q.y * q.y;
        t3 = +2.0 * (q.w * q.z + q.x * q.y);
        t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);

        return std::atan2(t3, t4);
    }

}