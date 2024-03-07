#ifndef __UPDATE_PARAM_H__
#define __UPDATE_PARAM_H__

#include <rclcpp/rclcpp.hpp>

/** @brief from Autoware */
template <class T>
bool updateParam(const std::vector<rclcpp::Parameter> &params, const std::string &name, T &value)
{
    const auto itr = std::find_if(
        params.cbegin(), params.cend(),
        [&name](const rclcpp::Parameter &p)
        { return p.get_name() == name; });

    // Not found
    if (itr == params.cend())
    {
        return false;
    }

    value = itr->template get_value<T>();
    return true;
}

#endif