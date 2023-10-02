# Overview
This package is a tool package for ROS2 that contains:
- Tools for visualization in ROS `ros_visuals.cpp`

- Tools for data saving `data_saver.h`

- Tools for profiling in `profiling.h`

- Various other tools in `helpers.h`

For usage see the [example](src/example.cpp). To launch the example use

```
ros2 launch ros_tools example.launch
```

---
## Visualization
We provide wrappers for visualization that simplifies drawing markers in rviz. For example, to draw a cube the following code is sufficient.

```cpp
auto& cube = marker_publisher.getNewPointMarker("CUBE");
cube.setColorInt(0, 5); // Color by index
cube.setScale(0.25, 0.25, 0.25); // Scale in all directions
cube.addPointMarker(Eigen::Vector3d(0., 0., 0.)); // Cube position
```

When running the example, you should see the following in `rviz2`:

<img src="docs/example.png" alt="example" width="400"/>

---

## Profiling
Profiling output is stored in the package that is selected in its initialization (see the example). To view the output, open chrome and go to `chrome://tracing/`. Click load in the top left and navigate and select `<your_package>/profiler.json`.

The output should be as follows:

<img src="docs/profiling_example.png" alt="example" width="800"/>

---

## Data Saving
Data is saved in `ros_tools/scripts/data/example_data.txt`. See the example for more details.

Saved data can be read to a python dictionary through `scripts/load_ros_data.py`.

---

# Installing dependencies
From your workspace directory:
```
rosdep install --from-paths src --ignore-src -r -y
```