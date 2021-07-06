if (WIN32)
  add_library(controllers SHARED
    potential_field_controller.h potential_field_controller.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(controllers SHARED ${sources} ${headers})
endif(UNIX)

install(TARGETS controllers DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PotentialFields")
