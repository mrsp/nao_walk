cmake_minimum_required(VERSION 2.8.3)
project(nao_behaviour)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    actionlib
    rospy
    message_generation
)

add_service_files(
  FILES
  Default.srv
  sayText.srv
)

catkin_python_setup()
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    actionlib
)

set( nodes 
    src/nao_behaviour/nao_behaviour_node
    src/nao_behaviour/nao_speek_node
)

catkin_install_python(PROGRAMS
    ${nodes}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
