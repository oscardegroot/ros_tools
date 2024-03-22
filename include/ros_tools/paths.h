#ifndef __ROS_TOOLS_PATHS_H__
#define __ROS_TOOLS_PATHS_H__

#include <string>

#ifdef MPC_PLANNER_ROS
#include <ros/package.h>

inline std::string getPackagePath(const std::string &&package_name)
{
    return ros::package::getPath(package_name) + "/";
}

inline std::string getPackagePath(const std::string &package_name)
{
    return ros::package::getPath(package_name) + "/";
}

#else
#include <ament_index_cpp/get_package_share_directory.hpp>

inline std::string getPackagePath(const std::string &&package_name)
{
    return ament_index_cpp::get_package_share_directory(package_name) + "/";
}

inline std::string getPackagePath(const std::string &package_name)
{
    return ament_index_cpp::get_package_share_directory(package_name) + "/";
}

#endif

#endif // __ROS_TOOLS_PATHS_H__
