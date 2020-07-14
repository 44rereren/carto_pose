cmake_minimum_required(VERSION 2.8.3)
project(carto_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  carto_pose_red
  src/red.cpp
)

target_link_libraries(
  carto_pose_red
  ${catkin_LIBRARIES}
)

add_executable(
  carto_pose_blue
  src/blue.cpp
)

target_link_libraries(
  carto_pose_blue
  ${catkin_LIBRARIES}
)

