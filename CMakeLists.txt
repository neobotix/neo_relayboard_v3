cmake_minimum_required(VERSION 2.6)
project(neo_relayboard_v3)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

file(GLOB GENERATED generated/src/*.cpp)

add_library(relayboardv3 SHARED
	${GENERATED}
	src/RelayBoardV3.cpp
)

target_include_directories(relayboardv3 PUBLIC
	include/
	generated/include/
)

target_link_libraries(relayboardv3
	pilot_public
)

add_executable(relayboardv3_node
	src/relayboardv3_node.cpp
)

target_link_libraries(relayboardv3_node
	relayboardv3
)


add_subdirectory(pilot-public)


