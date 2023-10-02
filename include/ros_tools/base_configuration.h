/**
 * @file base_configuration.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Can be used for generalizing configuration files throughout the package in the future
 * @version 0.1
 * @date 2023-02-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __BASE_CONFIGURATION_H__
#define __BASE_CONFIGURATION_H__


#include <rclcpp/rclcpp.hpp>
#include <string>

namespace RosTools{

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
  bool retrieveParameter(const ros::NodeHandle& nh, const std::string& name, T& value)
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
  void retrieveParameter(const ros::NodeHandle& nh, const std::string& name, T& value, const T& default_value)
  {
    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str()
                                     << " -> using default value: " << default_value);
      value = default_value;
    }
  }

  template <class L>
  void retrieveParameter(const ros::NodeHandle& nh, const std::string& name, std::vector<L>& value,
                         const std::vector<L>& default_value)
  {
    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str()
                                     << " -> using default value with size: " << default_value.size());
      value = default_value;
    }
  }
};
};
#endif  // __BASE_CONFIGURATION_H__