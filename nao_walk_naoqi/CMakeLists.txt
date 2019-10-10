cmake_minimum_required(VERSION 2.8)
project(nao_walkF)

find_package(qibuild)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -O3  -march=atom -DBOOST_SIGNALS_NO_DEPRECATION_WARNING")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -DDEBUG")
set(_srcs
main.cpp
MotionDefines.h
LowLevelPlanner.cpp
LowLevelPlanner.h
WalkEngine.cpp
WalkEngine.h
PCThread.cpp
PCThread.h
NAOKinematics.cpp
NAOKinematics.h
RobotParameters.cpp
RobotParameters.h
robot_consts.cpp
robot_consts.h
Common.hpp
KinematicsDefines.h
SystemMutex.hpp
KMat.hpp
KWalkMat.h
LeakyIntegrator.h
LeakyIntegrator.cpp
JointSSKF.h
JointSSKF.cpp
CoMEKF.h
CoMEKF.cpp
PostureController.h
PostureController.cpp
CoMAdmittanceControl.h
CoMAdmittanceControl.cpp
MPCDCM.h
MPCDCM.cpp
MovingAverageFilter.h
MovingAverageFilter.cpp
FeetEngine.h
FeetEngine.cpp
matlog.h
matlog.cpp
CoM_Admittance.h
CoM_Admittance.cpp
butterworthLPF.h
butterworthLPF.cpp
butterworthHPF.h
butterworthHPF.cpp
TrajectoryPlanner.h
TrajectoryPlanner.cpp
COPKF.h
COPKF.cpp
SocketServer.cpp
DelayedObserverDCM.h
DelayedObserverDCM.cpp
ZMPDistributor.h
ZMPDistributor.cpp
FootPolygon.h
FootPolygon.cpp
Array.hh
Array.cc
QuadProg++.hh
QuadProg++.cc
StepAdjustment.h
StepAdjustment.cpp
Stepplanner2D.h
Stepplanner2D.cpp
LIPMThread.h
LIPMThread.cpp
)



#Create the KWalk Module
qi_create_lib(nao_walkF SHARED ${_srcs} SUBFOLDER naoqi)


qi_use_lib(nao_walkF ALCOMMON ALMEMORYFASTACCESS ALPROXIES)
