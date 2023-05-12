FILE(GLOB_RECURSE cppheaders *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(dynacore_ik SHARED ${sources} ${cppheaders})
target_link_libraries(dynacore_ik
                      dynacore_tasks
                      dynacore_utils
                      dynacore_robot
                      Valkyrie_model
                      dynacore_rbdl
                      dynacore_urdf
                      quadprog
                      ${catkin_LIBRARIES})

install(TARGETS dynacore_ik DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} DESTINATION "${INSTALL_INCLUDE_DIR}/IKModule")

