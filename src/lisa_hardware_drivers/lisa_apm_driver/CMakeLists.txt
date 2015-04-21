cmake_minimum_required(VERSION 2.8.3)
project(lisa_apm_driver)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs nav_msgs std_msgs roscpp tf)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(lisa_apm_driver lisa_apm_driver.cpp)
target_link_libraries(lisa_apm_driver ${catkin_LIBRARIES})


