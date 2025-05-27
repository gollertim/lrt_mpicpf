cmake_minimum_required(VERSION 2.8.3)
project(lrt_mpipfc)
set(CMAKE_PROJECT_VERSION 2.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DCMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  controller_interface
  control_msgs
  franka_hw
  hardware_interface
  kdl_parser
  realtime_tools
  robot_state_publisher
  std_msgs
  tf
  urdf
  actionlib
)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Franka 0.9.0 REQUIRED)
find_package(gazebo REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "disable yaml tests")
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "disable yaml tools")
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "disable yaml contrib")


catkin_package(
  LIBRARIES lrt_mpic
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rospy
  controller_interface
  control_msgs
  franka_hw
  hardware_interface
  kdl_parser
  realtime_tools
  robot_state_publisher
  std_msgs
  tf
  urdf
  actionlib
  DEPENDS Franka
)

include_directories(
  include
  ../libs/grampc/include
  ../libs/grampc/cpp/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ../libs/grampc/cpp/libs
  ${Franka_LIBRARY_DIRS}
  ${pinocchio_LIBRARY_DIRS}
)

add_library(lrt_mpic
  # grampc problem description
  src/mpic_problem_description.cpp
  src/grampc_util_cpp.cpp

  # MPIC basics
  src/mpic_utils.cpp
  src/mpic_controller.cpp

  # Task Control State Machine
  src/mpic_state.cpp
  src/mpic_finflag.cpp
  src/mpic_exception.cpp
  src/mpic_statemachine.cpp

  # Gripper
  src/gripper.cpp

  # Task-Skill-MP framework
  src/task_framework.cpp
  src/skill_framework.cpp
  src/mp_library/mp_types.cpp
  src/mp_library/generic_mpic.cpp
  src/mp_library/mtc_mpic.cpp

  # Controller
  src/sim_mpic_controller.cpp
  src/real_mpic_controller.cpp
)

target_link_libraries(lrt_mpic
  grampc
  yaml-cpp
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${Franka_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${Threads_LIBRARIES}
  Threads::Threads
)

install(TARGETS lrt_mpic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES lrt_mpic_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
