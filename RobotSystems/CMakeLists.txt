add_subdirectory(Valkyrie)

install(FILES RobotSystem.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")



#-------------------------------------------------
# Robot Utilities
#-------------------------------------------------

if (WIN32)
  add_library(dynacore_robot SHARED
    robot_utils.h robot_utils.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(dynacore_robot SHARED ${sources} ${headers})
endif(UNIX)

install(TARGETS dynacore_robot DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/RobotSystems")
