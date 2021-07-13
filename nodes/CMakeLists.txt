#--------------------------------------------------------------
# IK Test Node:
# for testing IK Module on pre-defined tasks
#--------------------------------------------------------------
add_executable(ik_module_test_node ik_module_test_node.cpp)
target_link_libraries(ik_module_test_node ${catkin_LIBRARIES}
                                          dynacore_rbdl
                                          dynacore_urdf
                                          dynacore_utils
                                          Valkyrie_model
                                          dynacore_tasks
                                          dynacore_ik)

#--------------------------------------------------------------
# Controller Test Node:
# for testing potential field controllers
#--------------------------------------------------------------
add_executable(controller_test_node controller_test_node.cpp)
target_link_libraries(controller_test_node ${catkin_LIBRARIES}
                                           dynacore_rbdl
                                           dynacore_urdf
                                           dynacore_utils
                                           Valkyrie_model
                                           dynacore_tasks
                                           dynacore_ik
                                           potential_fields
                                           controllers)
