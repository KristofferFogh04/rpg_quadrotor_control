cmake_minimum_required(VERSION 2.8.3)
project(rqt_quad_control_tuning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(SetParametersNode
        src/set_control_parameters.cpp
)

cs_add_executable(SetTrajectoryNode
        src/set_trajectory.cpp
)

cs_install()
cs_export()
