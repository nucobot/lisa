cmake_minimum_required(VERSION 2.8.3)
project(lisa_move)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf_conversions
                                        sensor_msgs std_msgs roscpp tf)
find_package(OpenCV REQUIRED)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lisa_move lisa_move.cpp)
target_link_libraries(lisa_move ${catkin_LIBRARIES})


