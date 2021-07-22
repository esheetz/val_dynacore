FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(controllers SHARED ${sources} ${headers})
target_link_libraries(controllers
                      potential_fields
                      dynacore_utils
                      dynacore_robot
                      Valkyrie_model
                      dynacore_tasks
                      dynacore_ik)

install(TARGETS controllers DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Controllers")
