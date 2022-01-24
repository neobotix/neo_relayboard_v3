cmake_minimum_required(VERSION 2.6)
project(neo_relayboard_v3)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
#	nav_msgs
	roscpp
#	tf
#	std_msgs
	sensor_msgs
	trajectory_msgs
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
#		std_msgs
#		tf
#		nav_msgs
		sensor_msgs
		trajectory_msgs
	INCLUDE_DIRS
		include
		generated/include
)

file(GLOB GENERATED generated/src/*.cpp)

add_library(relayboardv3 SHARED
	${GENERATED}
	src/RelayBoardV3.cpp
)

target_include_directories(relayboardv3 PUBLIC
	include/
	generated/include/
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

target_link_libraries(relayboardv3
	pilot_public
	pilot_kinematics_differential_interface
	pilot_kinematics_mecanum_interface
	pilot_kinematics_omnidrive_interface
	${catkin_LIBRARIES}
)

add_executable(relayboardv3_node
	src/relayboardv3_node.cpp
)

add_dependencies(relayboardv3_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(relayboardv3_node
	relayboardv3
)


add_subdirectory(pilot-public)


install(TARGETS relayboardv3 relayboardv3_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


