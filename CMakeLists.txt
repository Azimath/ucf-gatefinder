cmake_minimum_required(VERSION 2.8.3)
project(ucf_gatefindtest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  actionlib
  move_base_msgs
)

find_package(OpenCV 2.4.8 EXACT REQUIRED)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} "src")

add_executable(find_gate src/gate_search.cpp)
add_executable(turtlebot_drive src/turtlebot_drive.cpp)
add_executable(turtlebot_cmd_vel src/turtlebot_cmd_vel.cpp)

target_link_libraries(find_gate ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(turtlebot_drive ${catkin_LIBRARIES})
target_link_libraries(turtlebot_cmd_vel ${catkin_LIBRARIES})
