include_directories ("${PROJECT_SOURCE_DIR}/Utils")

if (WIN32)
  add_library(ihmc_msg_utils SHARED
    ihmc_msg_utilities.hpp ihmc_msg_utilities.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(ihmc_msg_utils SHARED ${sources} ${headers})
endif(UNIX)

install(TARGETS ihmc_msg_utils DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/utils")
