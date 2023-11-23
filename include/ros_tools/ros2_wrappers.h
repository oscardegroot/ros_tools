#ifndef __ROSTOOLS_ROSTWO_WRAPPERS__
#define __ROSTOOLS_ROSTWO_WRAPPERS__

#include <rclcpp/rclcpp.hpp>
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

    template <typename T>
    void callServiceWithoutResponse(const typename rclcpp::Client<T>::SharedPtr client, rclcpp::Logger logger)
    {
        auto req = std::make_shared<typename T::Request>();

        RCLCPP_INFO(logger, "client request");

        if (!client->service_is_ready())
        {
            RCLCPP_INFO(logger, "client is unavailable");
            return;
        }

        client->async_send_request(req, [logger](typename rclcpp::Client<T>::SharedFuture result)
                                   { RCLCPP_INFO(
                                         logger, "Status: %d, %s", result.get()->status.code,
                                         result.get()->status.message.c_str()); });
    }
};

#endif // __ROSTOOLS_ROSTWO_WRAPPERS__