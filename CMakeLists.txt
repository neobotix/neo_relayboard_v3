cmake_minimum_required(VERSION 2.8.3)
project(neo_relayboard_v3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  trajectory_msgs
  sensor_msgs
  neo_msgs
  neo_srvs
)

catkin_package(
  INCLUDE_DIRS include generated/include
  LIBRARIES relayboardv3
)

include_directories(
  include
  generated/include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB GENERATED generated/src/*.cpp)

add_library(relayboardv3 SHARED
  ${GENERATED}
  src/RelayBoardV3.cpp
)

target_link_libraries(relayboardv3
  pilot_public
  pilot_kinematics_bicycle_interface
  pilot_kinematics_differential_interface
  pilot_kinematics_mecanum_interface
  pilot_kinematics_omnidrive_interface
  vnx_base
  ${catkin_LIBRARIES}
)

add_executable(relayboardv3_node
  src/relayboardv3_node.cpp
)

target_link_libraries(relayboardv3_node
  relayboardv3
  ${catkin_LIBRARIES}
)

add_subdirectory(pilot-public)

install(TARGETS relayboardv3 relayboardv3_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/ generated/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
