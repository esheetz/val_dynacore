FILE(GLOB_RECURSE cppheaders *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(dynacore_ik SHARED ${sources} ${cppheaders})
target_link_libraries(dynacore_ik dynacore_rbdl
dynacore_urdf
quadprog
dynacore_utils
Valkyrie_model
dynacore_tasks
)

install(TARGETS dynacore_ik DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${cppheaders} DESTINATION "${INSTALL_INCLUDE_DIR}/IKModule")

