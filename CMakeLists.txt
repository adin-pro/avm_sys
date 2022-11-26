cmake_minimum_required(VERSION 3.0.2)
project(avm_sys)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
)

catkin_package()

include(cmake/opencv.cmake)
include(cmake/glog.cmake)
include(cmake/YAML.cmake)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(front_cam_node src/front_cam_node.cpp)

target_link_libraries(front_cam_node
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)

add_executable(back_cam_node src/back_cam_node.cpp)

target_link_libraries(back_cam_node
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)

add_executable(left_cam_node src/left_cam_node.cpp)

target_link_libraries(left_cam_node
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)

add_executable(right_cam_node src/right_cam_node.cpp)

target_link_libraries(right_cam_node
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)

