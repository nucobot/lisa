cmake_minimum_required(VERSION 2.8.3)
project(lisa_odometry)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation message_runtime
                                        sensor_msgs std_msgs roscpp tf)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(lisa_odometry lisa_odometry.cpp)
target_link_libraries(lisa_odometry ${catkin_LIBRARIES})


