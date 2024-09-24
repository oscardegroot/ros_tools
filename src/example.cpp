#include "ros/ros.h"

#include <ros_tools/visuals.h>

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "rostools_example");

    VISUALS.init();
    auto &publisher = VISUALS.getPublisher("ros_tools/example");
    auto &cube = publisher.getNewPointMarker("CUBE");
    cube.setColorInt(0, 5);                           // Color by index 0 out of 5
    cube.setScale(0.25, 0.25, 0.25);                  // Scale in all directions
    cube.addPointMarker(Eigen::Vector3d(0., 0., 0.)); // Cube position

    auto &line = publisher.getNewLine();
    line.setColor(0., 1., 0., 0.8);
    line.setScale(0.3, 0.3);
    line.addLine(Eigen::Vector2d(1., 0.), Eigen::Vector2d(1., 5.), 1.);

    auto &model = publisher.getNewModelMarker("package://ros_tools/models/walking.dae");
    model.setColorInt(1, 5);
    model.addPointMarker(Eigen::Vector3d(3., 0., 1.0));

    auto &text = publisher.getNewTextMarker();
    text.setText("This is an example");
    text.setColorInt(4, 5);
    text.addPointMarker(Eigen::Vector3d(5., 0., 3.));

    publisher.publish();

    ros::Duration(5.).sleep();

    return 0;
}
