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

    // For visuals!
    RosTools::ROSMarkerPublisher marker_publisher(node, "ros_tools/example/visuals", "map", 5);

    // For Datasaving!
    RosTools::DataSaver data_saver;
    int counter = 0; // Dummy for data

    rclcpp::Rate rate(1.);
    while (rclcpp::ok())
    {
        PROFILE_SCOPE("Loop()");

        {
            PROFILE_SCOPE("Visuals");

            /** @note Visuals */
            // Publish two cubes in RViz
            auto &cube = marker_publisher.getNewPointMarker("CUBE");
            cube.setColorInt(0, 5); // index 0/5
            cube.setScale(0.25, 0.25, 0.25);
            cube.addPointMarker(Eigen::Vector3d(0., 0., 0.));

            // A bigger cube
            cube.setColorInt(1, 5, 0.5); // Transparency of 0.5 and index 1/5
            cube.setScale(0.5, 0.5, 0.5);
            cube.addPointMarker(Eigen::Vector3d(0., 0., 1.));

            // A sphere
            auto &sphere = marker_publisher.getNewPointMarker("SPHERE");
            sphere.setColorInt(2, 5);
            sphere.setScale(0.25, 0.25, 0.25);
            sphere.addPointMarker(Eigen::Vector3d(0., 0., 2.));

            // Draw a line upward
            auto &line = marker_publisher.getNewLine();
            line.setScale(0.1); // Line thickness
            line.setColorInt(3, 5);

            geometry_msgs::msg::Point p1, p2;
            p1.x = 1.;
            p1.y = 0.;
            p1.z = 0.;
            p2 = p1;
            p2.z = 5.;

            line.addLine(p1, p2);

            // Text
            auto &text = marker_publisher.getNewTextMarker();
            text.setText("Ros Tools Example!");
            text.setColorInt(5, 5);
            text.addPointMarker(Eigen::Vector3d(-1, 0., 0.));
        }

        /** @note Datasaver */
        {
            PROFILE_SCOPE("Data saving");
            data_saver.AddData("counter", counter); // Adds a data point to the "counter" variable

            data_saver.SaveData("example_data"); // Saves data with this name

            counter++;
        }

        // Publish all visuals
        marker_publisher.publish();

        rclcpp::spin_some(node);
        rate.sleep();
    }

    RosTools::Instrumentor::Get().EndSession();

    rclcpp::shutdown();

    return 0;
}