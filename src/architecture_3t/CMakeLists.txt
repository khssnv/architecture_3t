cmake_minimum_required(VERSION 3.0.2)
project(architecture_3t)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
  uuid_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Order.msg
)

add_service_files(
  FILES
  CreateOrder.srv
)

add_action_files(
  FILES
  Order.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  uuid_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/planning
  nodes/execution
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   # myfile1
#   # launch/mylaunch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
