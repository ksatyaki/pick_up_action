cmake_minimum_required(VERSION 2.8.3)
project(pick_up_action)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  jaco_manipulation
  geometry_msgs
  roscpp
  std_msgs
  tf
  message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PickUp.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs jaco_manipulation geometry_msgs roscpp std_msgs tf message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(pick_up_action_server src/pick_up_action_server.cpp)
add_executable(pick_up_action_client src/pick_up_client_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pick_up_action_server
  ${catkin_LIBRARIES}  
)

target_link_libraries(pick_up_action_client
    ${catkin_LIBRARIES}
)
