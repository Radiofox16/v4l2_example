cmake_minimum_required(VERSION 3.16)
project(v4l2_example)

set(CMAKE_CXX_STANDARD 17)

add_executable(v4l2_example main.cpp V4l2Camera.cpp V4l2Camera.hpp)