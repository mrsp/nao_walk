cmake_minimum_required(VERSION 2.8.3)
project(nao_walk)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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

add_action_files(DIRECTORY action FILES behaviour.action speek.action)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

 generate_dynamic_reconfigure_options(
   cfg/GaitControl.cfg
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nao_walk
  CATKIN_DEPENDS  geometry_msgs roscpp actionlib actionlib_msgs message_runtime tf humanoid_nav_msgs
)

    include_directories( include
        ${catkin_INCLUDE_DIRS}
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
        ${Boost_LIBRARIES})
    target_link_libraries(nao_walk_dynamic_configure ${catkin_LIBRARIES})


add_definitions( " -DBOOST_SIGNALS_NO_DEPRECATION_WARNING ")