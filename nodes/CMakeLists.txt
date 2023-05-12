#-----------------------------------------------------------------------
# IK Test Node:
# for testing IK Module on pre-defined tasks
#-----------------------------------------------------------------------
add_executable(ik_module_test_node ik_module_test_node.cpp)
target_link_libraries(ik_module_test_node dynacore_tasks
                                          dynacore_ik
                                          dynacore_utils
                                          dynacore_robot
                                          Valkyrie_model
                                          dynacore_rbdl
                                          dynacore_urdf
                                          quadprog
                                          ${catkin_LIBRARIES})

#-----------------------------------------------------------------------
# Controller Test Node:
# for testing potential field controllers
#-----------------------------------------------------------------------
add_executable(controller_test_node controller_test_node.cpp)
target_link_libraries(controller_test_node dynacore_tasks
                                           dynacore_ik
                                           potential_fields
                                           controllers
                                           controllers_core
                                           dynacore_utils
                                           dynacore_robot
                                           Valkyrie_model
                                           dynacore_rbdl
                                           dynacore_urdf
                                           ${catkin_LIBRARIES})

#-----------------------------------------------------------------------
# Controller Manager Node:
# for running arbitrary (compositions of) controllers
# TODO: NOT USED RIGHT NOW
#-----------------------------------------------------------------------
#add_executable(controller_manager_node controller_manager_node.cpp)
#target_link_libraries(controller_manager_node dynacore_tasks
#                                              dynacore_ik
#                                              potential_fields
#                                              controllers
#                                              dynacore_utils
#                                              dynacore_robot
#                                              Valkyrie_model
#                                              dynacore_rbdl
#                                              dynacore_urdf
#                                              ${catkin_LIBRARIES})

#-----------------------------------------------------------------------
# Controller Reference Publisher Node:
# for publishing reference poses for different controller types
#-----------------------------------------------------------------------
add_executable(controller_reference_publisher_node controller_reference_publisher_node.cpp)
target_link_libraries(controller_reference_publisher_node dynacore_utils
                                                          ${catkin_LIBRARIES})

#-----------------------------------------------------------------------
# Set Valkyrie Standing Node:
# for setting Valkyrie to a standing pose; for RViz testing only
#-----------------------------------------------------------------------
add_executable(set_valkyrie_standing_node set_valkyrie_standing_node.cpp)
target_link_libraries(set_valkyrie_standing_node dynacore_utils
                                                 dynacore_robot
                                                 Valkyrie_model
                                                 dynacore_rbdl
                                                 dynacore_urdf
                                                 ${catkin_LIBRARIES})

#-----------------------------------------------------------------------
# Pelvis Transform Broadcaster Node:
# for broadcasting pelvis transforms; for RViz testing only
#-----------------------------------------------------------------------
add_executable(pelvis_transform_broadcaster_node pelvis_transform_broadcaster_node.cpp)
target_link_libraries(pelvis_transform_broadcaster_node dynacore_utils
                                                        ${catkin_LIBRARIES})
