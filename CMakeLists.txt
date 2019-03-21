cmake_minimum_required(VERSION 2.8.3)
project(item2)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_publisher src/image_publisher.cpp)
target_link_libraries(image_publisher ${catkin_LIBRARIES})

add_executable(image_rotator src/image_rotator.cpp)
target_link_libraries(image_rotator ${catkin_LIBRARIES})
