include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")

if (WIN32)
  add_library(dynacore_utils SHARED
    wrap_eigen.hpp wrap_eigen.cpp
    utilities.hpp utilities.cpp
    pseudo_inverse.hpp pseudo_inverse.cpp
)
endif(WIN32)

if (UNIX)
FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library(dynacore_utils SHARED ${sources} ${headers} ${cppheaders})
endif(UNIX)

install(TARGETS dynacore_utils DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils")
