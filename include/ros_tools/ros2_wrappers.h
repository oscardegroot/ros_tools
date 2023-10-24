#ifndef __ROSTOOLS_ROSTWO_WRAPPERS__
#define __ROSTOOLS_ROSTWO_WRAPPERS__

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>

namespace RosTools
{

    inline std::string GetSharedPath(std::string &&package_name)
    {
        return ament_index_cpp::get_package_share_directory(package_name) + "/";
    }
    inline std::string GetSharedPath(std::string &package_name)
    {
        return ament_index_cpp::get_package_share_directory(package_name) + "/";
    }

    inline std::string GetPackagePath(std::string &&package_name)
    {
        // Finds the share directory (ws/install/share/<package_name>)
        return ament_index_cpp::get_package_share_directory(package_name) + "/../../../src/" + package_name + "/";
    }

    inline std::string GetPackagePath(const std::string &package_name)
    {
        // Finds the share directory (ws/install/share/<package_name>)
        return ament_index_cpp::get_package_share_directory(package_name) + "/../../../src/" + package_name + "/";
    }

};

#endif // __ROSTOOLS_ROSTWO_WRAPPERS__