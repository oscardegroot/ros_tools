
#include <ros/ros.h>
#include <risk_computation.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    RiskComputation risk_;

    // ros::spin();

    return 0;
}