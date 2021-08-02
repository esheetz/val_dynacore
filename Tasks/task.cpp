/**
 * Generic Task Class
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <Tasks/task.h>

// using Tasks::Task;

// CONSTRUCTORS/DESTRUCTORS
Task::Task() {
    std::cout << "[" << getTaskName() << "] Constructed" << std::endl;
}

Task::Task(std::shared_ptr<RobotSystem> robot_model_in) {
    robot_model_ = robot_model_in;
    std::cout << "[" << getTaskName() << "] Constructed" << std::endl;
}

Task::~Task() {
    std::cout << "[" << getTaskName() << "] Destroyed" << std::endl;
}

// GETTERS/SETTERS
void Task::getTarget(dynacore::Vect3& ref_pos_out) {
    // ref_pos_out = ref_pos_;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_pos_out)" << std::endl;
    return;
}

void Task::getTarget(dynacore::Quaternion& ref_quat_out) {
    // ref_quat_out = ref_quat_;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_quat_out)" << std::endl;
    return;
}

void Task::getTarget(dynacore::Vect3& ref_pos_out, dynacore::Quaternion& ref_quat_out) {
    // ref_pos_out = ref_pos_;
    // ref_quat_out = ref_quat_;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_pos_out, ref_quat_out)" << std::endl;
    return;
}

void Task::getTarget(dynacore::Vector& ref_q_out) {
    // ref_q_out = ref_q_;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "getTarget(ref_q_out)" << std::endl;
    return;
}

void Task::setTarget(dynacore::Vect3 ref_pos_in) {
    // ref_pos_ = ref_pos_in;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_pos_in)" << std::endl;
    return;
}

void Task::setTarget(dynacore::Quaternion ref_quat_in) {
    // ref_quat_ = ref_quat_in;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_quat_in)" << std::endl;
    return;
}

void Task::setTarget(dynacore::Vect3 ref_pos_in, dynacore::Quaternion ref_quat_in) {
    // ref_pos_ = ref_pos_in;
    // ref_quat_ = ref_quat_in;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_pos_in, ref_quat_in)" << std::endl;
    return;
}

void Task::setTarget(dynacore::Vector ref_q_in) {
    // ref_q_ = ref_q_in;
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "setTarget(ref_q_in)" << std::endl;
    return;
}

double Task::getTaskGain() {
    return kp_task_gain_;
}

void Task::setTaskGain(double kp_task_in) {
    kp_task_gain_ = kp_task_in;
    return;
}

double Task::getTaskWeight() {
    return w_task_weight_;
}

void Task::setTaskWeight(double w_task_in) {
    w_task_weight_ = w_task_in;
    return;
}

std::string Task::getTaskName() {
    return task_name_;
}

int Task::getTaskDimension() {
    return task_dim_;
}

void Task::initializeTaskParameters() {
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "initializeTaskParameters()" << std::endl;
    return;
}

// TASK RELATED COMPUTATIONS
void Task::computeTaskResidual(dynacore::Vector& r_task) {
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "computeTaskResidual(r_task)" << std::endl;
    return;
}

void Task::computeTaskVelocityResidual(dynacore::Vector& v_task, double dt) {
    // compute task residual
    dynacore::Vector r;
    computeTaskResidual(r);
    // computation below will use internal residual_; will be equivalent to residual r above

    // v = r/dt
    velocity_ = residual_;
    velocity_ /= dt;

    if( debug_ ) {
        dynacore::pretty_print(velocity_, std::cout, "Task velocity:");
    }

    // set input variable
    v_task = velocity_;

    return;
}

double Task::computeTaskCost(double dt) {
    // compute task velocity
    dynacore::Vector v;
    computeTaskVelocityResidual(v, dt);
    // computation below will use internal velocity_; will be equivalent to velocity v above

    // c = w * v^T * v
    double cost = w_task_weight_ * (static_cast<double>(velocity_.transpose() * velocity_));

    if( debug_ ) {
        std::cout << "Task cost: " << cost << std::endl;
    }

    return cost;
}

void Task::computeTaskJacobian(dynacore::Matrix& J_task) {
    std::cout << "[WARNING] Task " << task_name_ << " has no implementation for " << "computeTaskJacobian(J_task)" << std::endl;
    return;
}