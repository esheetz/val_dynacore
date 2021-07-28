/**
 * Joint Configuration Task Class
 * Emily Sheetz, NSTGRO VTE 2021 
 **/

#include <Tasks/task_joint_config.h>

// CONSTRUCTORS/DESTRUCTORS
TaskJointConfig::TaskJointConfig() {
    // initialize parameters for task
    initializeTaskParameters();
}

TaskJointConfig::TaskJointConfig(std::shared_ptr<RobotSystem> robot_model_in, std::vector<int> joint_idxs_in, std::map<int, std::string> joint_indices_to_names_in) {
    // set joints and robot
    joint_indices_ = joint_idxs_in;
    joint_indices_to_names_ = joint_indices_to_names_in;
    robot_model_ = robot_model_in;

    // initialize parameters for task
    initializeTaskParameters();
}

TaskJointConfig::~TaskJointConfig() {
}

// GETTERS/SETTERS
void TaskJointConfig::getTarget(dynacore::Vector& ref_q_out) {
    ref_q_out = ref_q_;
    return;
}

void TaskJointConfig::setTarget(dynacore::Vector ref_q_in) {
    ref_q_ = ref_q_in;
    return;
}

void TaskJointConfig::initializeTaskParameters() {
    // set task name
    task_name_ = "TaskJointConfig";

    // set task dimension
    task_dim_ = joint_indices_.size();

    // initialize current configuration
    curr_q_.resize(joint_indices_.size());
    curr_q_.setZero();

    // initialize reference configuration
    ref_q_.resize(joint_indices_.size());
    ref_q_.setZero();

    return;
}

// TASK RELATED COMPUTATIONS
void TaskJointConfig::computeTaskResidual(dynacore::Vector& r_task) {
    // update current configuration of robot
    robot_model_->getCurrentQ(curr_fullq_);

    // set configuration of relevant joints
    curr_q_.resize(joint_indices_.size());
    for( int i = 0 ; i < joint_indices_.size() ; i++ ) {
        curr_q_[i] = curr_fullq_[joint_indices_[i]];
    }

    if( debug_ ) {
        for( int i = 0 ; i < joint_indices_.size() ; i++ ) {
            printf("Joint name=%s, idx=%d, current config=%f, target config=%f",
                    joint_indices_to_names_[joint_indices_[i]].c_str(), joint_indices_[i],
                    curr_q_[i], ref_q_[i]);
            std::cout << std::endl;
        }
    }

    // compute joint difference
    residual_ = ref_q_ - curr_q_;

    // multiply by task gain
    residual_ *= kp_task_gain_;

    if( debug_ ) {
        dynacore::pretty_print(residual_, std::cout, "Task residual:");
    }

    // set input variable
    r_task = residual_;

    return;
}

void TaskJointConfig::computeTaskJacobian(dynacore::Matrix& J_task) {
    // get number DOFs and resize Jacobian matrix
    int ndofs = robot_model_->getDimQdot();
    J_task.resize(task_dim_, ndofs);

    // initialize to zero matrix
    J_task.setZero();

    // Jacobian has 1 entries wherever a controlled joint is
    for( int i = 0 ; i < joint_indices_.size() ; i++ ) {
        J_task(i, joint_indices_[i]) = 1;
    }

    if( debug_ ) {
        dynacore::pretty_print(J_task, std::cout, "Task Jacobian:");
    }

    return;
}
