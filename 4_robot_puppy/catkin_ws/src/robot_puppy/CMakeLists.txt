cmake_minimum_required(VERSION 3.0.2)
project(robot_puppy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files (
  FILES
  BallLocation.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
