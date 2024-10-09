#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from ros_visuals import ROSMarkerPublisher
import time

def main():
    rclpy.init()

    node = Node("rostools_example")
    marker_publisher = ROSMarkerPublisher(node, "ros_tools/example", "map", 5)
    counter = 0  # Dummy for data

    rate = node.create_rate(1.0)

    while rclpy.ok():
        # Publish two cubes in RViz
        cube = marker_publisher.get_cube()
        cube.set_color(0, 1.0)  # index 0/5
        cube.set_scale(0.25, 0.25, 0.25)
        cube.add_marker(Point(x=0.0, y=0.0, z=0.0))

        # A bigger cube
        cube.set_color(1, 0.5)  # Transparency of 0.5 and index 1/5
        cube.set_scale(0.5, 0.5, 0.5)
        cube.add_marker(Point(x=0.0, y=0.0, z=1.0))

        # A sphere
        sphere = marker_publisher.get_sphere()
        sphere.set_color(2, 0.5)
        sphere.set_scale(0.25, 0.25, 0.25)
        sphere.add_marker(Point(x=0.0, y=0.0, z=2.0))

        # Draw a line upward
        line = marker_publisher.get_line()
        line.set_scale(0.1)  # Line thickness
        line.set_color(3, 0.7)

        p1 = Point(x=1.0, y=0.0, z=0.0)
        p2 = Point(x=1.0, y=0.0, z=5.0)
        line.add_line(p1, p2)



        counter += 1

        # Publish all visuals
        marker_publisher.publish()

        rclpy.spin_once(node)
        rate.sleep()

    rclpy.shutdown()

if __name__ == "__main__":
    main()