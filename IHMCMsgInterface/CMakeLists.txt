#---------------------------------------------------------------------------
# Dependencies on ROS and IHMC controller_msgs
#---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  sensor_msgs
  std_msgs
  #controller_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp tf geometry_msgs sensor_msgs #controller_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

#---------------------------------------------------------------------------
# Add subdirectories
#---------------------------------------------------------------------------
add_subdirectory (ihmc_utils)
add_subdirectory (ihmc_nodes)
add_subdirectory (ihmc_tests)

include_directories ("${PROJECT_SOURCE_DIR}/ihmc_utils")
include_directories ("${PROJECT_SOURCE_DIR}/ihmc_nodes")
