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

namespace RosTools
{

  class BaseConfiguration
  {
  protected:
    bool declared_ = false;

    /* Retrieve paramater, if it doesn't exist return false */
    template <class T>
    bool retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, T &value)
    {
      if (!declared_)
        node->declare_parameter<T>(name);

      return node->get_parameter(name, value);
    }

    template <class T>
    bool retrieveParameter(rclcpp::Node *node, const std::string &name, T &value)
    {
      if (!declared_)
        node->declare_parameter<T>(name);

      return node->get_parameter(name, value);
    }

    /* Retrieve parameter, if it doesn't exist use the default */
    template <class T>
    void retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, T &value, const T &default_value)
    {
      if (!declared_)
        node->declare_parameter<T>(name, default_value);

      node->get_parameter(name, value);
    }

    /* Retrieve parameter, if it doesn't exist use the default */
    template <class T>
    void retrieveParameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
    {
      if (!declared_)
        node->declare_parameter<T>(name, default_value);

      node->get_parameter(name, value);
    }

    template <class L>
    void retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
    {
      if (!declared_)
        node->declare_parameter<std::vector<L>>(name, default_value);

      node->get_parameter(name, value);
    }

    template <class L>
    void retrieveParameter(rclcpp::Node *node, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
    {
      if (!declared_)
        node->declare_parameter<std::vector<L>>(name, default_value);

      node->get_parameter(name, value);
    }
  };
};
#endif // __BASE_CONFIGURATION_H__