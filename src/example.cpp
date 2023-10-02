/** @brief This is an example of the functionality in ros_tools */

#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>
#include <ros_tools/profiling.h>
#include <ros_tools/data_saver.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    RosTools::Instrumentor::Get().BeginSession("ros_tools");

    PROFILE_FUNCTION();

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("rostools_example");

    rclcpp::Rate rate(1.);
    while (rclcpp::ok())    
    {
        PROFILE_SCOPE("Loop()");
        
        // Publish a line in RViz


        rclcpp::spin_some(node);
        rate.sleep();

    }

    RosTools::Instrumentor::Get().EndSession();


    rclcpp::shutdown();

    return 0;

}