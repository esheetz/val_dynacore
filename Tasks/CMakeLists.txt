#include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")

if (WIN32)
  add_library(dynacore_tasks SHARED
    task.h task.cpp
    task_6dpose.h task_6dpose.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE cppheaders *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(dynacore_tasks SHARED ${sources} ${headers} ${cppheaders})
endif(UNIX)

install(TARGETS dynacore_tasks DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} DESTINATION "${INSTALL_INCLUDE_DIR}/Tasks")

add_executable(task_test task_test.cpp)
target_link_libraries(task_test dynacore_rbdl
                                dynacore_urdf
                                dynacore_utils
                                Valkyrie_model
                                dynacore_tasks)
