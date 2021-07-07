include_directories ("${PROJECT_SOURCE_DIR}/Utils")

if (WIN32)
  add_library(potential_fields SHARED
    potential_field.h potential_field.cpp
    attractive_potential_field.h attractive_potential_field.cpp
    repulsive_potential_field.h repulsive_potential_field.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(potential_fields SHARED ${sources} ${headers})
endif(UNIX)

install(TARGETS potential_fields DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PotentialFields")
