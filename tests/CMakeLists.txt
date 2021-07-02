#---------------------------------------------------------------------
# Robot Model Test:
# for robot state, joint limits, Jacobians
#---------------------------------------------------------------------
add_executable(robot_model_test robot_model_test.cpp)
target_link_libraries(robot_model_test dynacore_rbdl
                                dynacore_urdf                                
                                dynacore_utils
                                Valkyrie_model
                                dynacore_tasks
                                dynacore_ik)

#---------------------------------------------------------------------
# IK Test:
# for IK module
#---------------------------------------------------------------------
add_executable(ik_test ik_test.cpp)
target_link_libraries(ik_test dynacore_rbdl
                              dynacore_urdf
                              dynacore_utils
                              Valkyrie_model
                              dynacore_tasks
                              dynacore_ik)

#---------------------------------------------------------------------
# Task Test:
# for tasks (generic, 6DPose, JointConfig)
#---------------------------------------------------------------------
add_executable(task_test task_test.cpp)
target_link_libraries(task_test dynacore_rbdl
                                dynacore_urdf
                                dynacore_utils
                                Valkyrie_model
                                dynacore_tasks)

#---------------------------------------------------------------------
# QuadProg++ Test:
# for simple quadratic programming example
#---------------------------------------------------------------------
add_executable(quadprog_test quadprog_test.cpp)
target_link_libraries(quadprog_test quadprog)
