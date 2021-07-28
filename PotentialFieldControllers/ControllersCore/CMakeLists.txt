FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(controllers_core SHARED ${sources} ${headers})
target_link_libraries(controllers_core
                      controllers
                      potential_fields
                      dynacore_utils
                      dynacore_robot
                      Valkyrie_model
                      dynacore_tasks
                      dynacore_ik)

install(TARGETS controllers_core DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/ControllersCore")
