cmake_minimum_required(VERSION 3.10)
add_library(examples_common examples_common.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (Franka REQUIRED)
target_link_libraries(examples_common PUBLIC
  Eigen3::Eigen
  Franka::Franka
)
