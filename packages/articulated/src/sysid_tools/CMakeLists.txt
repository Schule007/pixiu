cmake_minimum_required(VERSION 3.10)
pybind11_add_module(sysid_tools
  python_binding.cpp
  exponential_swept_sine.cpp
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(sysid_tools PUBLIC
  Eigen3::Eigen
)
