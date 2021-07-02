#---------------------------------------------------------------------
# IHMC Utils Test:
# for IHMC message utilities tests
#---------------------------------------------------------------------
add_executable(ihmc_msg_utils_test ihmc_msg_utils_test.cpp)
target_link_libraries(ihmc_msg_utils_test ihmc_msg_utils
                                          dynacore_utils)
