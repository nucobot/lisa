cmake_minimum_required(VERSION 2.8.3)
project(lisa_move)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf_conversions
                                        sensor_msgs std_msgs roscpp tf
                                        actionlib actionlib_msgs)
find_package(OpenCV REQUIRED)


FILE(GLOB ACTION_SRCS RELATIVE ${CMAKE_CURRENT_LIST_DIR}/action action/*.action)
add_action_files(
  DIRECTORY action
  FILES ${ACTION_SRCS}
)  

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lisa_move lisa_move.cpp)
target_link_libraries(lisa_move ${catkin_LIBRARIES})
add_dependencies(lisa_move ${PROJECT_NAME}_gencpp)


