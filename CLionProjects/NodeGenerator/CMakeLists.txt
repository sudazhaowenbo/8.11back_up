cmake_minimum_required(VERSION 3.16)
project(NodeGenerator)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(NodeGenerator main.cpp pictureDisplay.cpp pictureDisplay.h)

target_link_libraries(NodeGenerator ${OpenCV_LIBS})