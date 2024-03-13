#ifndef __UPDATE_PARAM_H__
#define __UPDATE_PARAM_H__

#include <rclcpp/rclcpp.hpp>

#define STATIC_NODE_POINTER StaticNodePointer::getInstance()
#define GET_STATIC_NODE_POINTER() StaticNodePointer::getInstance().getNodePointer()

/** @brief get a globally accessible node reference */
class StaticNodePointer
{
public:
    static StaticNodePointer &getInstance()
    {
        static StaticNodePointer instance;

        return instance;
    }

    void init(rclcpp::Node *node)
    {
        _node = node;
    }

    rclcpp::Node *getNodePointer()
    {
        if (_node == nullptr)
            throw std::runtime_error("StaticNodePointer was not initialized (run STATIC_NODE_POINTER.init(node) first!)");

        return _node;
    }

private:
    StaticNodePointer() = default;
    ~StaticNodePointer() = default;

    rclcpp::Node *_node{nullptr};
};

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