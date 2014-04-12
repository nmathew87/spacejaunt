cmake_minimum_required(VERSION 2.8.3)
project(image_stitcher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
)

find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(image_stitcher_node src/image_stitcher_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(image_stitcher_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
