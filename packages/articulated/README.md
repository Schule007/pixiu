# Articulated

## Build without ROS

Here `libfranka` is installed as package `ros-noetic-libfranka`. `Franka_DIR` contains `FrankaConfig.cmake`.
``` sh
mkdir _build
cd _build
cmake -DFranka_DIR=/opt/ros/noetic/share/franka/cmake/ ..
cmake --build .
```
