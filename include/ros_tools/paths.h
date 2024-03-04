#ifndef ros_tools_PATHS_H
#define ros_tools_PATHS_H

// Your code here
#define MPC_PLANNER_ROS 1

#include <string>

#if MPC_PLANNER_ROS == 1
#include <ros/package.h>

std::string getPackagePath(const std::string &&package_name)
{
    return ros::package::getPath(package_name);
}

std::string getPackagePath(const std::string &package_name)
{
    return ros::package::getPath(package_name);
}

#elif MPC_PLANNER_ROS == 2
#endif

#endif // ros_tools_PATHS_H
