######################################################
#  Title: Dynamic Control for Robotics (Expedition)  #
######################################################

cmake_minimum_required(VERSION 3.1)
project(val_dynacore)

add_definitions(-O3)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin )
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
)
find_package(Eigen3 REQUIRED)
catkin_package(
	CATKIN_DEPENDS roscpp rospy tf geometry_msgs nav_msgs sensor_msgs std_msgs
	DEPENDS EIGEN3
	INCLUDE_DIRS . RobotSystems ExternalSource
	LIBRARIES dynacore_utils Valkyrie_model
)
include_directories(${catkin_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(${EIGEN3_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories ("/usr/local/include")
include_directories ("${PROJECT_SOURCE_DIR}/")

include_directories ("${PROJECT_SOURCE_DIR}/RobotSystems")
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource/QuadProgpp")
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource/ParamHandler/include")
include_directories ("${PROJECT_SOURCE_DIR}/Tasks")
include_directories ("${PROJECT_SOURCE_DIR}/IKModule")
include_directories ("${PROJECT_SOURCE_DIR}/PotentialFieldControllers")
include_directories ("${PROJECT_SOURCE_DIR}/PotentialFieldControllers/PotentialFields")
include_directories ("${PROJECT_SOURCE_DIR}/PotentialFieldControllers/Controllers")
include_directories ("${PROJECT_SOURCE_DIR}/PotentialFieldControllers/ControllersCore")
include_directories ("${PROJECT_SOURCE_DIR}/nodes")


#set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#------------------------------------------------------------------------
#    Directory Selection
#------------------------------------------------------------------------
add_subdirectory (ExternalSource)
add_subdirectory (RobotSystems)
add_subdirectory (Utils)
add_subdirectory (Tasks)
add_subdirectory (IKModule)
add_subdirectory (PotentialFieldControllers)
add_subdirectory (nodes)
add_subdirectory (tests)

#------------------------------------------------------------------------
# Configuration
#------------------------------------------------------------------------
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)

#------------------------------------------------------------------------
# Printout
#------------------------------------------------------------------------
message ("")
message ("-[ MAKE INFORMATION ]----------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message ("--------------------------------------------------------------")
message ("")

install(FILES ${PROJECT_SOURCE_DIR}/Configuration.h DESTINATION "${INSTALL_INCLUDE_DIR}")
