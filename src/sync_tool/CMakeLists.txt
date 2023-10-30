cmake_minimum_required(VERSION 2.8.3)
project(sync_tool)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
  message_filters
)

catkin_simple()

find_package(PCL REQUIRED)
include_directories( include
  ${PCL_INCLUDE_DIRS}
)
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

# make the executable
cs_add_executable(sync_node src/data_sync.cpp)
target_link_libraries(sync_node ${catkin_LIBARIES})

cs_add_executable(ts_sync_node src/ts_sync.cpp)
target_link_libraries(ts_sync_node ${catkin_LIBARIES})

cs_add_executable(event_sync_node src/event_sync.cpp)
target_link_libraries(event_sync_node ${catkin_LIBARIES})