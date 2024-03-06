#ifndef __ROS_TOOLS_PATHS_H__
#define __ROS_TOOLS_PATHS_H__

// Your code here
#define MPC_PLANNER_ROS 1

#include <string>

#if MPC_PLANNER_ROS == 1
#include <ros/package.h>

inline std::string getPackagePath(const std::string &&package_name)
{
    return ros::package::getPath(package_name);
}

inline std::string getPackagePath(const std::string &package_name)
{
    return ros::package::getPath(package_name);
}

#elif MPC_PLANNER_ROS == 2

#endif

#endif // __ROS_TOOLS_PATHS_H__
