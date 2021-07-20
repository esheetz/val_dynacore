/**
 * Utilities for Robot Systems
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <iostream>
#include <stdio.h>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <Utils/wrap_eigen.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <RobotSystem.hpp>

namespace RobotUtils {

    // JACOBIAN FUNCTIONS

    /*
     * NOTE ON THE PURPOSE OF JACOBIAN FUNCTIONS
     * RBDL Jacobians have the convention of being rotational elements first, then linear elements
     * so each column is [dwx/dq; dwy/dq; dwz/dq; dx/dq; dy/dq; dz/dq]
     * these functions can be used to invert RBDL Jacobian to be linear then rotational
     */

    /*
     * get the Jacobian from the robot model
     * either full 6xn Jacobian and nx6 Jacobian pseudoinverse
     * or 3xn linear and angular Jacobians and nx3 linear and angular Jacobian pseudoinverses
     */
    void getRobotModelJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                dynacore::Matrix& _J, dynacore::Matrix& _Jinv);
    void getRobotModelJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                dynacore::Matrix& _Jlin, dynacore::Matrix& _Jang,
                                dynacore::Matrix& _Jlin_inv, dynacore::Matrix& _Jang_inv);

    /*
     * get the robot model Jacobian for the commanded joints based on commanded_joint_indices_
     * either full 6xn Jacobian and nx6 Jacobian pseudoinverse
     * or 3xn linear and angular Jacobians and nx3 linear and angular Jacobian pseudoinverses
     */
    void getCommandedJointJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                    std::vector<int> commanded_joint_indices,
                                    dynacore::Matrix& _J, dynacore::Matrix& _Jinv);
    void getCommandedJointJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                    std::vector<int> commanded_joint_indices,
                                    dynacore::Matrix& _Jlin, dynacore::Matrix& _Jang,
                                    dynacore::Matrix& _Jlin_inv, dynacore::Matrix& _Jang_inv);

    // JOINT NAME AND INDEX CONTAINERS

    /*
     * creates a map from joint indices to joint names
     * @param joint_indices, the vector of joint indices
     * @param joint_names, the vector of joint names
     * @param joint_indices_to_names, the map to be populated
     * @return boolean indicating if zip was successful
     * @post joint_indices_to_names updated based on joint indices and names
     */
    bool zipJointIndicesNames(std::vector<int> joint_indices, std::vector<std::string> joint_names,
                              std::map<int, std::string>& joint_indices_to_names);

    /*
     * unpacks map from joint indices to joint names
     * @param joint_indices_to_names, the map of joint indices to joint names
     * @param joint_indices, the vector of joint indices to be populated
     * @param joint_names, the vector of joint names to be populated
     * @return boolean indicating if unzip was successfull
     * @post joint_indices and joint_names updated based on map
     */
    bool unzipJointIndicesNames(std::map<int, std::string> joint_indices_to_names,
                                std::vector<int>& joint_indices, std::vector<std::string>& joint_names);
    bool unzipJointIndicesNames(std::vector<std::pair<int, std::string>> joint_indices_to_names,
                                std::vector<int>& joint_indices, std::vector<std::string>& joint_names);

} // end namespace RobotUtils