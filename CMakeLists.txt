cmake_minimum_required(VERSION 3.0.2)
project(elsa_tiago_gym)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
)

# Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

catkin_python_setup()

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  CATKIN_DEPENDS 
  rospy
  actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
