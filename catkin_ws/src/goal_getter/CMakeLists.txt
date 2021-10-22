cmake_minimum_required(VERSION 3.1.3)
project(goal_getter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hebi_cpp_api
  message_generation
  std_msgs
  geometry_msgs
  tf
  gb_visual_detection_3d_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GoalPose.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coborg_move
  CATKIN_DEPENDS
  roscpp
  hebi_cpp_api
  message_runtime
  std_msgs
  tf
  gb_visual_detection_3d_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
add_executable(goal_getter src/goal_getter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(goal_getter
  ${catkin_LIBRARIES}
)

add_dependencies(goal_getter ${catkin_EXPORTED_TARGETS})
add_dependencies(goal_getter goal_getter_generate_messages_cpp)
# target_link_libraries(goal_getter ${catkin_LIBRARIES} ${Boost_LIBRARIES})