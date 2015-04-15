cmake_minimum_required(VERSION 2.8.3)
project(lisa_camera)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport geometry_msgs
                                        sensor_msgs std_msgs roscpp tf)
find_package(OpenCV REQUIRED)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS})

MESSAGE( "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}" )
MESSAGE( "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}" )
MESSAGE( "OpenCV_LIBS = ${OpenCV_LIBS}" )

add_executable(lisa_camera lisa_camera.cpp)
target_link_libraries(lisa_camera ${catkin_LIBRARIES} 
                                  ${OpenCV_LIBS})


