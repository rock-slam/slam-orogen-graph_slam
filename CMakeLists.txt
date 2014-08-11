cmake_minimum_required(VERSION 2.6)
find_package(Rock)

add_definitions(-std=c++0x)
add_definitions(-std=gnu++0x)

rock_init(graph_slam 0.1)



rock_find_pkgconfig(ENVIRE-VIZ envire-viz REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
include(graph_slamBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

