cmake_minimum_required(VERSION 3.20)
project(neo_relayboard_v3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(neo_actions2 REQUIRED)

set(dependencies
	rclcpp
	rclcpp_action
	std_srvs
	std_msgs
	trajectory_msgs
	sensor_msgs
	neo_msgs2
	neo_srvs2
	neo_actions2
)

ament_export_include_directories(
	include
	generated/include
)

# VNX headers and library are supplied by the system installation.
file(GLOB GENERATED generated/src/*.cpp)

add_library(relayboardv3 SHARED
	${GENERATED}
	src/RelayBoardV3.cpp
)

target_include_directories(relayboardv3 PUBLIC
	include/
	generated/include/
)

target_link_libraries(relayboardv3 PUBLIC
	pilot_public
	pilot_kinematics_bicycle_interface
	pilot_kinematics_differential_interface
	pilot_kinematics_mecanum_interface
	pilot_kinematics_omnidrive_interface
	vnx_base
)

add_executable(relayboardv3_node
	src/relayboardv3_node.cpp
)

target_link_libraries(relayboardv3 PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  std_srvs::std_srvs
  std_msgs::std_msgs
  trajectory_msgs::trajectory_msgs
  sensor_msgs::sensor_msgs
  neo_msgs2::neo_msgs2
  neo_srvs2::neo_srvs2
  neo_actions2::neo_actions2
)

ament_export_dependencies(
	${dependencies}
)

target_link_libraries(relayboardv3_node
	relayboardv3
)


add_subdirectory(pilot-public)

install(TARGETS relayboardv3
    EXPORT export_relayboardv3
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${PROJECT_NAME})
install(TARGETS relayboardv3_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_libraries(relayboardv3)

ament_package()
