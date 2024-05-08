#ifndef ROS_TOOLS_BASE_CONFIGURATION_H
#define ROS_TOOLS_BASE_CONFIGURATION_H

#include <string>

#ifdef MPC_PLANNER_ROS
#include <ros/ros.h>

namespace RosTools
{

    class BaseConfiguration
    {
    protected:
        /**
         * @brief Retrieve a parameter from the ROS parameter server, return false if it failed
         *
         * @tparam T Variable type
         * @param nh nodehandle
         * @param name Name of the parameter on the server
         * @param value Variable to store the read value in
         * @return true If variable exists
         * @return false If variable does not exist
         */
        template <class T>
        bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
        {
            if (!nh.getParam(name, value))
            {
                ROS_ERROR_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
                return false;
            }
            else
            {
                return true;
            }
        }

        /**
         * @brief Retrieve a parameter from the ROS parameter server, otherwise use the default value
         *
         * @tparam T Variable type
         * @param nh nodehandle
         * @param name Name of the parameter on the server
         * @param value Variable to store the read value in
         * @param default_value Default value to use if the variable does not exist
         */
        template <class T>
        void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
        {
            if (!nh.getParam(name, value))
            {
                ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str()
                                               << " -> using default value: " << default_value);
                value = default_value;
            }
        }

        template <class L>
        void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, std::vector<L> &value,
                               const std::vector<L> &default_value)
        {
            if (!nh.getParam(name, value))
            {
                ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str()
                                               << " -> using default value with size: " << default_value.size());
                value = default_value;
            }
        }
    };
}
#else

#include <rclcpp/rclcpp.hpp>

namespace RosTools
{
    class BaseConfiguration
    {
    protected:
        bool declared_ = false;

    public:
        std::string fixROS2Name(const std::string &name)
        {
            std::string s = name;
            std::replace(s.begin(), s.end(), '/', '.'); // replace all 'x' to 'y'
            return s;
        }

        /* Retrieve paramater, if it doesn't exist return false */
        template <class T>
        bool retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, T &value)
        {
            std::string ros2_name = fixROS2Name(name);
            if (!declared_)
                node->declare_parameter<T>(ros2_name);

            return node->get_parameter(ros2_name, value);
        }

        template <class T>
        bool retrieveParameter(rclcpp::Node *node, const std::string &name, T &value)
        {
            std::string ros2_name = fixROS2Name(name);

            if (!declared_)
                node->declare_parameter<T>(ros2_name);

            return node->get_parameter(ros2_name, value);
        }

        /* Retrieve parameter, if it doesn't exist use the default */
        template <class T>
        void retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, T &value, const T &default_value)
        {
            std::string ros2_name = fixROS2Name(name);

            if (!declared_)
                node->declare_parameter<T>(ros2_name, default_value);

            node->get_parameter(ros2_name, value);
        }

        /* Retrieve parameter, if it doesn't exist use the default */
        template <class T>
        void retrieveParameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
        {
            std::string ros2_name = fixROS2Name(name);

            if (!declared_)
                node->declare_parameter<T>(ros2_name, default_value);

            node->get_parameter(ros2_name, value);
        }

        template <class L>
        void retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
        {
            std::string ros2_name = fixROS2Name(name);

            if (!declared_)
                node->declare_parameter<std::vector<L>>(ros2_name, default_value);

            node->get_parameter(ros2_name, value);
        }

        template <class L>
        void retrieveParameter(rclcpp::Node *node, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
        {
            std::string ros2_name = fixROS2Name(name);

            if (!declared_)
                node->declare_parameter<std::vector<L>>(ros2_name, default_value);

            node->get_parameter(ros2_name, value);
        }
    };
};
#endif
#endif // ROS_TOOLS_BASE_CONFIGURATION_H
