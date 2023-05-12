#---------------------------------------------------------------------
# Robot Model Test:
# for robot state, joint limits, Jacobians
#---------------------------------------------------------------------
add_executable(robot_model_test robot_model_test.cpp)
target_link_libraries(robot_model_test dynacore_tasks
                                       dynacore_ik
                                       dynacore_utils
                                       dynacore_robot
                                       Valkyrie_model
                                       dynacore_rbdl
                                       dynacore_urdf
                                       ${catkin_LIBRARIES})

#---------------------------------------------------------------------
# IK Test:
# for IK module
#---------------------------------------------------------------------
add_executable(ik_test ik_test.cpp)
target_link_libraries(ik_test dynacore_tasks
                              dynacore_ik
                              dynacore_utils
                              dynacore_robot
                              Valkyrie_model
                              dynacore_rbdl
                              dynacore_urdf
                              quadprog
                              ${catkin_LIBRARIES})

#---------------------------------------------------------------------
# Task Test:
# for tasks (generic, 6DPose, JointConfig)
#---------------------------------------------------------------------
add_executable(task_test task_test.cpp)
target_link_libraries(task_test dynacore_tasks
                                dynacore_utils
                                dynacore_robot
                                Valkyrie_model
                                dynacore_rbdl
                                dynacore_urdf
                                ${catkin_LIBRARIES})

#---------------------------------------------------------------------
# QuadProg++ Test:
# for simple quadratic programming example
#---------------------------------------------------------------------
add_executable(quadprog_test quadprog_test.cpp)
target_link_libraries(quadprog_test quadprog)

#---------------------------------------------------------------------
# Potential Field Test:
# for potential field hierarchy used for controllers
#---------------------------------------------------------------------
add_executable(potential_field_test potential_field_test.cpp)
target_link_libraries(potential_field_test potential_fields
                                           dynacore_utils
                                           ${catkin_LIBRARIES})
