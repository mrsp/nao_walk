cmake_minimum_required(VERSION 2.8.3)
project(nao_walk)
set (CMAKE_CXX_STANDARD 11)

add_compile_options(-std=c++11)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  genmsg
  actionlib_msgs
  actionlib
  message_generation
  tf
  humanoid_nav_msgs
  dynamic_reconfigure
)


#set(NAOqi_DIR "/home/master/naoqi/naoqi-sdk-2.1.4.13-linux64")
#find_package(NAOqi QUIET COMPONENTS
#    alcommon
#    alvision
#    alproxies
#    alerror
#    alvalue
#    qi
#    qitype
#    almath
#)
find_package(Boost REQUIRED COMPONENTS)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
  sayText.srv
  BallTracker.srv
   Tired.srv
   WarmHello.srv
   WakeUp.srv
   Crouch.srv
   DanceEvolution.srv
   EyeOfTheTiger.srv
   GangnamStyle.srv
   Macarena.srv
   TaiChi.srv
   Vangelis.srv
   WakeUp.srv
 )

## Generate actions in the 'action' folder
add_action_files(DIRECTORY action FILES behaviour.action speek.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)dynamic_reconfigure
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/GaitControl.cfg
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nao_walk
  CATKIN_DEPENDS  geometry_msgs roscpp actionlib actionlib_msgs message_runtime tf humanoid_nav_msgs
)

###########
## Build ##
###########

# if( NAOqi_FOUND AND Boost_FOUND)
    #message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
    include_directories( include
        ${catkin_INCLUDE_DIRS}
        #${NAOqi_INCLUDE_DIRS}
    	${Boost_INCLUDE_DIRS})
    
    add_executable(nao_walk 
        src/gait_control.cpp 
        src/nao_walk_ros.cpp 
        src/socket_client.cpp
        src/joint_states_table.cpp
    )
    
    add_dependencies(nao_walk ${PROJECT_NAME}_gencfg)
    add_executable(nao_walk_dynamic_configure nodes/server.cpp)
    target_link_libraries(nao_walk
        ${catkin_LIBRARIES}
        #${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES})
    target_link_libraries(nao_walk_dynamic_configure ${catkin_LIBRARIES})
# else()
#     message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
# endif()

add_definitions( " -DBOOST_SIGNALS_NO_DEPRECATION_WARNING ")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nao_walk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
