#include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")

if (WIN32)
  add_library(dynacore_tasks SHARED
    task.h task.cpp
    task_6dpose.h task_6dpose.cpp
    task_position.h task_position.cpp
    task_orientation.h task_orientation.cpp
    task_joint_config.h task_joint_config.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE cppheaders *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(dynacore_tasks SHARED ${sources} ${headers} ${cppheaders})
endif(UNIX)

install(TARGETS dynacore_tasks DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} DESTINATION "${INSTALL_INCLUDE_DIR}/Tasks")
