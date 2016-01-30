cmake_minimum_required(VERSION 2.8.3)
project(gatefindtest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 2.4.8 EXACT REQUIRED)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} "src")

add_executable(search_contours src/contour_search.cpp)

target_link_libraries(search_contours ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
