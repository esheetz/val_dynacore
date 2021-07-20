/**
 * Utilities for Robot Systems
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include "robot_utils.h"

namespace RobotUtils {

    // JACOBIAN FUNCTIONS

    void getRobotModelJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
        // create temporary Jacobian matrix
        dynacore::Matrix J_tmp;

        // get Jacobian from robot model
        robot_model->getFullJacobian(link_id, J_tmp);

        // get number DOFs and resize Jacobian matrix
        int ndofs = robot_model->getDimQdot();
        _J.resize(6, ndofs);

        // set linear Jacobian
        _J.block(0, 0, 3, ndofs) = J_tmp.block(3, 0, 3, ndofs);
        // set angular Jacobian
        _J.block(3, 0, 3, ndofs) = J_tmp.block(0, 0, 3, ndofs);

        // compute Jacobian pseudoinverse
        dynacore::pInv(_J, _Jinv);

        return;
    }

    void getRobotModelJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                dynacore::Matrix& _Jlin, dynacore::Matrix& _Jang,
                                dynacore::Matrix& _Jlin_inv, dynacore::Matrix& _Jang_inv) {
        // create temporary Jacobian matrix
        dynacore::Matrix J_tmp;

        // get Jacobian from robot model
        robot_model->getFullJacobian(link_id, J_tmp);

        // get number DOFs and resize Jacobian matrix
        int ndofs = robot_model->getDimQdot();
        _Jlin.resize(3, ndofs);
        _Jang.resize(3, ndofs);

        // set linear Jacobian
        _Jlin.block(0, 0, 3, ndofs) = J_tmp.block(3, 0, 3, ndofs);
        // set angular Jacobian
        _Jang.block(3, 0, 3, ndofs) = J_tmp.block(0, 0, 3, ndofs);

        // compute Jacobian pseudoinverse
        dynacore::pInv(_Jlin, _Jlin_inv);
        dynacore::pInv(_Jang, _Jang_inv);

        return;
    }

    void getCommandedJointJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                    std::vector<int> commanded_joint_indices,
                                    dynacore::Matrix& _J, dynacore::Matrix& _Jinv) {
        // create temporary Jacobian and Jacobian pseudoinverse matrices
        dynacore::Matrix J_tmp, Jinv_tmp;

        // get Jacobians
        getRobotModelJacobians(robot_model, link_id, J_tmp, Jinv_tmp);

        // resize matrix based on number of commanded joints
        _J.resize(6, commanded_joint_indices.size());

        // update columns of Jacobian to only include columns corresponding to commanded joints
        for( int i = 0 ; i < commanded_joint_indices.size() ; i++ ) {
            _J.col(i) = J_tmp.col(commanded_joint_indices[i]);
        }

        // compute Jacobian pseudoinverse
        dynacore::pInv(_J, _Jinv);

        return;
    }

    void getCommandedJointJacobians(std::shared_ptr<RobotSystem> robot_model, int link_id,
                                    std::vector<int> commanded_joint_indices,
                                    dynacore::Matrix& _Jlin, dynacore::Matrix& _Jang,
                                    dynacore::Matrix& _Jlin_inv, dynacore::Matrix& _Jang_inv) {
        // create temporary Jacobian and Jacobian pseudoinverse matrices
        dynacore::Matrix Jlin_tmp, Jlininv_tmp, Jang_tmp, Janginv_tmp;

        // get Jacobians
        getRobotModelJacobians(robot_model, link_id, Jlin_tmp, Jang_tmp, Jlininv_tmp, Janginv_tmp);

        // resize matrices based on number of commanded joints
        _Jlin.resize(3, commanded_joint_indices.size());
        _Jang.resize(3, commanded_joint_indices.size());

        // update columns of Jacobian to only include columns corresponding to commanded joints
        for( int i = 0 ; i < commanded_joint_indices.size() ; i++ ) {
            _Jlin.col(i) = Jlin_tmp.col(commanded_joint_indices[i]);
            _Jang.col(i) = Jang_tmp.col(commanded_joint_indices[i]);
        }

        // compute Jacobian pseudoinverse
        dynacore::pInv(_Jlin, _Jlin_inv);
        dynacore::pInv(_Jang, _Jang_inv);

        return;
    }

    // JOINT NAME AND INDEX CONTAINERS

    bool zipJointIndicesNames(std::vector<int> joint_indices, std::vector<std::string> joint_names,
                              std::map<int, std::string>& joint_indices_to_names) {
        // check if joint indices and names are the same size
        if( joint_indices.size() != joint_names.size() ) {
            std::printf("[ERROR] zipJointIndicesNames() -- sizes of joint indices (size=%ld) and joint names (size=%ld) do not match",
                        joint_indices.size(), joint_names.size());
            std::cout << std::endl;
            return false;
        }

        // same size, create map
        joint_indices_to_names.clear();
        for( int i = 0 ; i < joint_indices.size() ; i++ ) {
            joint_indices_to_names[joint_indices[i]] = joint_names[i];
        }

        return true;
    }

    bool unzipJointIndicesNames(std::map<int, std::string> joint_indices_to_names,
                                std::vector<int>& joint_indices, std::vector<std::string>& joint_names) {
        // create vectors
        joint_indices.clear();
        joint_names.clear();
        for( auto const& j : joint_indices_to_names ) {
            // j.first is joint index, j.second is joint name
            joint_indices.push_back(j.first);
            joint_names.push_back(j.second);
        }

        return true;
    }

    bool unzipJointIndicesNames(std::vector<std::pair<int, std::string>> joint_indices_to_names,
                                std::vector<int>& joint_indices, std::vector<std::string>& joint_names) {
        // create vectors
        joint_indices.clear();
        joint_names.clear();
        for( int i = 0 ; i < joint_indices_to_names.size() ; i++ ) {
            // [i].first is joint index, [i].second is joint name
            joint_indices.push_back(joint_indices_to_names[i].first);
            joint_names.push_back(joint_indices_to_names[i].second);
        }
        
        return true;
    }

} // end namespace RobotUtils